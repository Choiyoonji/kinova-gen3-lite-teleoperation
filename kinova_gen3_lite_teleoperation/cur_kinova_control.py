#! /usr/bin/env python3
import sys
import os

from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.client_stubs.ActuatorCyclicClientRpc import ActuatorCyclicClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.messages import Session_pb2, ActuatorConfig_pb2, Base_pb2, BaseCyclic_pb2, Common_pb2
from kortex_api.RouterClient import RouterClientSendOptions

import numpy as np
import time
import threading
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Pinocchio for dynamics
import pinocchio as pin

# ==========================
# Helper conversions
# ==========================
DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

class Ros2Bridge(Node):
    def __init__(self, current_control_example):
        super().__init__('ros2_bridge')
        self.current_control_example = current_control_example

        # /target_position subscriber
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/target_position',
            self.target_callback,
            10
        )

        self.ma_window = 30
        self.ma_buffers = [deque(maxlen=self.ma_window) 
                           for _ in range(len(self.current_control_example.q_des_deg))]

        # /current_position publisher
        self.pub = self.create_publisher(Float32MultiArray, '/current_position', 10)

        # 주기적으로 current_position publish
        timer_period = 0.025  # 40Hz
        self.timer = self.create_timer(timer_period, self.publish_current_position)

    def target_callback(self, msg: Float32MultiArray): # 10hz
        if len(msg.data) != 7:
            self.get_logger().warning(
                f"Received target length {len(msg.data)} != actuator count "
                f"{len(self.current_control_example.q_des_deg)}"
            )
            return

        with self.current_control_example.lock:
            self.current_control_example.target_gripper = msg.data[-1]  # 그리퍼 목표각 바로 반영
            for i, deg_new in enumerate(msg.data[:-1]):
                # 2) 이동평균 버퍼에 추가
                self.ma_buffers[i].append(deg_new)

                # 3) 이동평균 계산(버퍼 평균)
                filt = float(np.mean(self.ma_buffers[i])) if len(self.ma_buffers[i]) > 0 else deg_new

                # 4) 필터 결과를 목표각으로 반영
                self.current_control_example.q_des_deg[i] = filt

        # self.get_logger().info("Updated target position (moving-average filtered).")

    def publish_current_position(self):
        """CurrentControlExample에서 current_position 가져와 ROS2로 publish"""
        msg = Float32MultiArray()
        processed_data = []
        with self.current_control_example.lock:
            curr = list(self.current_control_example.q_deg)
        for i in range(len(curr)):
            angle_deg = float(curr[i])
            processed_data.append((angle_deg + 180.0) % 360.0 - 180.0)

        processed_data.append(self.current_control_example.current_gripper)

        msg.data = processed_data
        self.pub.publish(msg)


class CurrentControlExample:
    def __init__(self, router, router_real_time):
        self.lock = threading.Lock()
        # Maximum allowed waiting time during actions (in seconds)
        self.ACTION_TIMEOUT_DURATION = 20

        # Create required services
        device_manager = DeviceManagerClient(router)
        
        self.actuator_config = ActuatorConfigClient(router)
        self.base = BaseClient(router)
        self.base_cyclic = BaseCyclicClient(router_real_time)

        self.base_command = BaseCyclic_pb2.Command()
        self.base_feedback = BaseCyclic_pb2.Feedback()
        self.base_custom_data = BaseCyclic_pb2.CustomData()

        # Detect all devices
        device_handles = device_manager.ReadAllDevices()
        self.ndof = self.base.GetActuatorCount().count

        self.base_command.ClearField("actuators")
        self.base_feedback.ClearField("actuators")
        for _ in range(self.ndof):
            self.base_command.actuators.add()
            self.base_feedback.actuators.add()

        # Add motor command to interconnect's cyclic
        self.base_command.frame_id = 0
        self.base_command.interconnect.command_id.identifier = 0
        self.base_command.interconnect.gripper_command.command_id.identifier = 0
        self.motorcmd = self.base_command.interconnect.gripper_command.motor_cmd.add()

        # Change send option to reduce max timeout at 3ms
        self.sendOption = RouterClientSendOptions()
        self.sendOption.andForget = False
        self.sendOption.delay_ms = 0
        self.sendOption.timeout_ms = 10

        self.cyclic_t_end = 30  #Total duration of the thread in seconds. 0 means infinite.
        self.cyclic_thread = {}

        self.kill_the_thread = False
        self.already_stopped = False
        self.cyclic_running = False

        # State buffers (deg, deg/s)
        self.q_deg = [0.0]*self.ndof
        self.dq_deg = [0.0]*self.ndof
        self.q_des_deg = [0.0]*self.ndof

        # Gripper percent (0~100)
        self.current_gripper = 0.0
        self.target_gripper = 0.0
        self.grip_vel = 30.0   # 0~100 (%/s)
        self.grip_force = 50.0 # 0~100 (%)

        # Pinocchio model
        urdf_path = os.path.join(
            os.path.dirname(__file__),
            "GEN3-LITE.urdf"
        )
        print(f"[INFO] Loading URDF: {urdf_path}")
        self.pin_model = pin.buildModelFromUrdf(urdf_path)
        self.pin_data = self.pin_model.createData()

        # PD gains in joint space (rad units)
        self.Kp = np.array([40.0]*self.ndof)  # conservative start
        self.Kd = np.array([2.0]*self.ndof)

        # Soft torque limits (Nm) — conservative defaults
        self.torque_limit = np.array([2.5, 2.5, 2.0, 1.5, 1.2, 1.0][:self.ndof])

        # === Motor-current mapping (per Kinova issue #165) ===
        # tau = I * Kt * N * eta => I = tau / (Kt*N*eta)
        Kt_full = np.array([0.0398, 0.0398, 0.0398, 0.0251, 0.0251, 0.0251]) # N·m/A
        N_full = np.array([30.0, 100.0, 30.0, 23.0, 23.0, 23.0])
        eta_full= np.array([0.90, 0.65, 0.90, 0.90, 0.90, 0.90])
        self.Kt = Kt_full[:self.ndof]
        self.N = N_full[:self.ndof]
        self.eta = eta_full[:self.ndof]


        # Derived current limits from torque limits (A)
        self.current_limit = self.torque_limit / (self.Kt * self.N * self.eta)


        # Cycle params
        self.t_sample = 0.001 # 200 Hz default

    # Create closure to set an event after an END or an ABORT
    def check_for_end_or_abort(self, e):
        """Return a closure checking for END or ABORT notifications

        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """
        def check(notification, e = e):
            print("EVENT : " + \
                Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check

    def MoveToHomePosition(self):
        # Make sure the arm is in Single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
    
        # Move arm to ready position
        print("Moving the arm to a safe position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == "kinova_home":
                action_handle = action.handle
        print("action_handle: ", action_handle)
        if action_handle == None:
            print("Can't reach safe position. Exiting")
            return False

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        self.base.ExecuteActionFromReference(action_handle)

        print("Waiting for movement to finish ...")
        finished = e.wait(self.ACTION_TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Cartesian movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

        return True

    def InitCyclic(self, sampling_time_cyclic, t_end, print_stats):

        if self.cyclic_running:
            return True

        # Move to Home position first
        if not self.MoveToHomePosition():
            return False

        print("Init Cyclic")
        sys.stdout.flush()

        base_feedback = self.SendCallWithRetry(self.base_cyclic.RefreshFeedback, 3)
        if base_feedback:
            self.base_feedback = base_feedback

            # Init command frame
            for x in range(self.ndof):
                self.base_command.actuators[x].flags = 1  # servoing
                self.base_command.actuators[x].position = self.base_feedback.actuators[x].position
                self.q_deg[x] = self.base_feedback.actuators[x].position
                self.q_des_deg[x] = self.base_feedback.actuators[x].position
                self.dq_deg[x] = self.base_feedback.actuators[x].velocity
                self.base_command.actuators[x].torque_joint = self.base_feedback.actuators[x].torque

            # Set arm in LOW_LEVEL_SERVOING
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)

            try:
                curr_gpos = base_feedback.interconnect.gripper_feedback.motor[0].position  # 0~100 (%)
            except Exception:
                curr_gpos = 0.0
            self.current_gripper = curr_gpos
            self.motorcmd.position = curr_gpos
            self.motorcmd.velocity = 0.0
            self.motorcmd.force    = self.grip_force

            # Send first frame
            self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)

            for i in range(self.ndof):
                control_mode_message = ActuatorConfig_pb2.ControlModeInformation()
                control_mode_message.control_mode = ActuatorConfig_pb2.ControlMode.Value('CURRENT')
                device_id = i + 1
                self.SendCallWithRetry(self.actuator_config.SetControlMode, 3, control_mode_message, device_id)

            # Init cyclic thread
            self.cyclic_t_end = t_end
            self.cyclic_thread = threading.Thread(target=self.RunCyclic, args=(sampling_time_cyclic, print_stats))
            self.cyclic_thread.daemon = True
            self.cyclic_thread.start()
            return True

        else:
            print("InitCyclic: failed to communicate")
            return False

    def _tau_to_current(self, tau_nm: np.ndarray) -> np.ndarray:
        """Map desired joint torques (Nm) -> motor currents (A)."""
        I = tau_nm / (self.Kt * self.N * self.eta)
        return np.clip(I, -self.current_limit, self.current_limit)
    
    def RunCyclic(self, t_sample, print_stats):
        self.cyclic_running = True
        print("Run Cyclic")
        sys.stdout.flush()
        cyclic_count = 0  # Counts refresh
        stats_count = 0  # Counts stats prints
        failed_cyclic_count = 0  # Count communication timeouts

        t_now = time.time()
        t_cyclic = t_now  # cyclic time
        t_stats = t_now  # print  time
        t_init = t_now  # init   time
 
        ma_size = 50
        ma_deque = [deque(maxlen=ma_size) for _ in range(self.ndof)]

        cmd_id = 1

        while not self.kill_the_thread:
            t_now = time.time()

            # Cyclic Refresh
            if (t_now - t_cyclic) >= t_sample:
                dt = 0.001
                t_cyclic = t_now

                with self.lock:
                    # === 그리퍼 명령도 같은 프레임에 실어서 보냄 (0~100%) ===
                    self.motorcmd.position = self.target_gripper
                    self.motorcmd.velocity = self.grip_vel
                    self.motorcmd.force    = self.grip_force
                    
                    q  = np.array(self.q_deg, dtype=float) * DEG2RAD
                    dq = np.array(self.dq_deg, dtype=float) * DEG2RAD
                    q_des = np.array(self.q_des_deg, dtype=float) * DEG2RAD
                    dq_des = np.zeros_like(dq)  # desire zero velocity

                # 3) Desired joint acceleration from PD in velocity form
                ddq_des = self.Kp*(q_des - q) + self.Kd*(dq_des - dq)

                # 4) RNEA to get full torque (gravity + coriolis + inertia)
                tau = pin.rnea(self.pin_model, self.pin_data, q, dq, ddq_des)

                # 5) Soft-limit torques
                tau = np.clip(tau, -self.torque_limit, self.torque_limit)

                # Convert to currents (A) and clamp
                I_cmd = self._tau_to_current(tau)
                # print(I_cmd)
                for i in range(self.ndof):
                    self.base_command.actuators[i].position = self.base_feedback.actuators[i].position
                    # self.base_command.actuators[i].current_motor = float(I_cmd[i])
                    self.base_command.actuators[i].current_motor = 0.0

                # Incrementing identifier ensure actuators can reject out of time frames
                self.base_command.frame_id += 1
                if self.base_command.frame_id > 65535:
                    self.base_command.frame_id = 0
                for i in range(self.ndof):
                    self.base_command.actuators[i].command_id = self.base_command.frame_id

                # interconnect/그리퍼 command_id도 같이 갱신
                self.base_command.interconnect.command_id.identifier = self.base_command.frame_id
                self.base_command.interconnect.gripper_command.command_id.identifier = self.base_command.frame_id

                # Frame is sent
                try:
                    self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)
                    with self.lock:
                        self.q_deg = [self.base_feedback.actuators[i].position
                                      for i in range(self.ndof)]
                        self.dq_deg = [self.base_feedback.actuators[i].velocity
                                       for i in range(self.ndof)]
                        self.q_des_deg = self.q_deg.copy()  # avoid large jumps

                    try:
                        self.current_gripper = self.base_feedback.interconnect.gripper_feedback.motor[0].position
                    except Exception:
                        pass

                except:
                    failed_cyclic_count = failed_cyclic_count + 1
                cyclic_count = cyclic_count + 1

            # Stats Print
            if print_stats and ((t_now - t_stats) > 1):
                t_stats = t_now
                stats_count = stats_count + 1
                
                print("current position: ", self.q_deg, end="\r")
                print("target position: ", self.q_des_deg, end="\r")
                cyclic_count = 0
                failed_cyclic_count = 0
                sys.stdout.flush()

        self.cyclic_running = False
        return True

    def StopCyclic(self):
        print ("Stopping the cyclic and putting the arm back in position mode...")
        if self.already_stopped:
            return

        # Kill the  thread first
        if self.cyclic_running:
            self.kill_the_thread = True
            self.cyclic_thread.join()
        
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        self.cyclic_t_end = 0.1

        self.already_stopped = True
        
        print('Clean Exit')

    @staticmethod
    def SendCallWithRetry(call, retry,  *args):
        i = 0
        arg_out = []
        while i < retry:
            try:
                arg_out = call(*args)
                break
            except:
                i = i + 1
                continue
        if i == retry:
            print("Failed to communicate")
        return arg_out

def main():
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    parser = argparse.ArgumentParser()
    parser.add_argument("--cyclic_time", type=float, default=0.001)
    parser.add_argument("--duration", type=int, default=30)
    parser.add_argument("--print_stats", default=True, type=lambda x: (str(x).lower() not in ['false','0','no']))
    args = utilities.parseConnectionArguments(parser)

    # ROS2 init
    rclpy.init()

    with utilities.DeviceConnection.createTcpConnection(args) as router:
        with utilities.DeviceConnection.createUdpConnection(args) as router_real_time:

            example = CurrentControlExample(router, router_real_time)

            # ROS2 bridge 노드 생성
            ros2_node = Ros2Bridge(example)

            success = example.InitCyclic(args.cyclic_time, args.duration, args.print_stats)

            if success:
                try:
                    while example.cyclic_running:
                        # ROS2 spin & Kinova 제어 동시 실행
                        rclpy.spin_once(ros2_node, timeout_sec=0.05)
                except KeyboardInterrupt:
                    pass

                example.StopCyclic()

    ros2_node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
