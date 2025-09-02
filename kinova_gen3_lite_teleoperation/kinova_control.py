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

class Ros2Bridge(Node):
    def __init__(self, torque_example):
        super().__init__('ros2_bridge')
        self.torque_example = torque_example

        # /target_position subscriber
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/target_position',
            self.target_callback,
            10
        )

        self.ma_window = 20
        self.ma_buffers = [deque(maxlen=self.ma_window) 
                           for _ in range(len(self.torque_example.target_position))]

        # /current_position publisher
        self.pub = self.create_publisher(Float32MultiArray, '/current_position', 10)

        # 주기적으로 current_position publish
        timer_period = 0.025  # 40Hz
        self.timer = self.create_timer(timer_period, self.publish_current_position)

    def target_callback(self, msg: Float32MultiArray): # 10hz
        if self.torque_example.cyclic_running is False:
            self.get_logger().warning("Cyclic not running, cannot update target.")
            return
        if len(msg.data) != 7:
            self.get_logger().warning(
                f"Received target length {len(msg.data)} != actuator count "
                f"{len(self.torque_example.target_position)}"
            )
            return

        with self.torque_example.lock:
            self.torque_example.target_gripper = msg.data[-1]  # 그리퍼 목표각 바로 반영
            for i, deg_new in enumerate(msg.data[:-1]):
                # 2) 이동평균 버퍼에 추가
                self.ma_buffers[i].append(deg_new)

                # 3) 이동평균 계산(버퍼 평균)
                filt = float(np.mean(self.ma_buffers[i])) if len(self.ma_buffers[i]) > 0 else deg_new

                # 4) 필터 결과를 목표각으로 반영
                self.torque_example.target_position[i] = filt

        # self.get_logger().info("Updated target position (moving-average filtered).")

    def publish_current_position(self):
        """TorqueExample에서 current_position 가져와 ROS2로 publish"""
        if self.torque_example.cyclic_running is False:
            self.get_logger().warning("Cyclic not running, cannot update target.")
            return
        msg = Float32MultiArray()
        processed_data = []
        with self.torque_example.lock:
            curr = list(self.torque_example.current_position)
        for i in range(len(curr)):
            angle_deg = float(curr[i])
            processed_data.append((angle_deg + 180.0) % 360.0 - 180.0)

        processed_data.append(self.torque_example.current_gripper)

        msg.data = processed_data
        self.pub.publish(msg)

class FilterAndRateLimiter:
    """
    입력 목표에 EMA 필터를 적용한 후,
    실시간 속도 및 가속도 제한을 수행합니다.
    """
    def __init__(self, max_velocity, max_acceleration, dt=0.001, alpha=0.1):
        # 1. 필터 관련 파라미터 및 상태 변수
        if not (0.0 < alpha <= 1.0):
            raise ValueError("alpha 값은 0보다 크고 1보다 작거나 같아야 합니다.")
        self.alpha = alpha  # 필터링 강도 (작을수록 부드러움)
        self.smoothed_target = 0.0 # 필터링된 목표 위치를 저장

        # 2. 속도/가속도 제한 관련 파라미터 및 상태 변수
        self.max_v = abs(max_velocity)
        self.max_a = abs(max_acceleration)
        self.dt = dt
        self.prev_velocity = 0.0

    def initialize_target(self, position):
        """필터의 초기 위치를 현재 로봇 위치로 설정하여 시작 시 튀는 현상 방지"""
        self.smoothed_target = position

    def compute(self, current_pos, final_target):
        """
        필터링과 속도/가속도 제한을 모두 적용하여
        다음 스텝의 목표 위치를 계산합니다.
        """
        # --- 단계 1: EMA 필터로 목표 위치를 부드럽게 만들기 ---
        self.smoothed_target = self.alpha * final_target + (1.0 - self.alpha) * self.smoothed_target

        # --- 단계 2: 부드러워진 목표(smoothed_target)를 사용하여 속도/가속도 제한 적용 ---
        error = self.smoothed_target - current_pos
        desired_velocity = error / self.dt

        v_limited_by_vel = np.clip(desired_velocity, -self.max_v, self.max_v)

        max_delta_v = self.max_a * self.dt
        v_limited_by_accel = np.clip(
            v_limited_by_vel,
            self.prev_velocity - max_delta_v,
            self.prev_velocity + max_delta_v
        )
        
        final_limited_velocity = v_limited_by_accel
        next_target_pos = current_pos + np.clip(final_limited_velocity * self.dt, -2.0, 2.0)

        self.prev_velocity = final_limited_velocity
        
        return next_target_pos

class TorqueExample:
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
        self.actuator_count = self.base.GetActuatorCount().count

        self.base_command.ClearField("actuators")
        self.base_feedback.ClearField("actuators")
        for _ in range(self.actuator_count):
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

        self.current_position = [0.0] * self.actuator_count
        self.target_position = [0.0] * self.actuator_count

        self.current_gripper = 0.0
        self.target_gripper = 0.0
        self.grip_vel    = 20.0    # 0~100 (%)
        self.grip_force  = 50.0    # 0~100 (%)

        self.limiter = []
        for i in range(self.actuator_count):
            self.limiter.append(FilterAndRateLimiter(max_velocity=900.0, max_acceleration=18000.0, dt=0.001, alpha=0.1))

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
            for x in range(self.actuator_count):
                self.base_command.actuators[x].flags = 1  # servoing
                self.base_command.actuators[x].position = self.base_feedback.actuators[x].position
                self.current_position[x] = self.base_feedback.actuators[x].position
                self.target_position[x] = self.base_feedback.actuators[x].position
                self.limiter[x].initialize_target(self.base_feedback.actuators[x].position)

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

            # Init cyclic thread
            self.cyclic_t_end = t_end
            self.cyclic_thread = threading.Thread(target=self.RunCyclic, args=(sampling_time_cyclic, print_stats))
            self.cyclic_thread.daemon = True
            self.cyclic_thread.start()
            return True

        else:
            print("InitCyclic: failed to communicate")
            return False

    def RunCyclic(self, t_sample, print_stats):
        self.cyclic_running = True
        print("Run Cyclic!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")
        sys.stdout.flush()
        cyclic_count = 0  # Counts refresh
        stats_count = 0  # Counts stats prints
        failed_cyclic_count = 0  # Count communication timeouts

        t_now = time.time()
        t_cyclic = t_now  # cyclic time
        t_stats = t_now  # print  time
        t_init = t_now  # init   time
        ma_size = 300  # moving average size
        ma_deque = [deque(maxlen=ma_size) for _ in range(self.actuator_count)]

        print("Running torque control example for {} seconds".format(self.cyclic_t_end))
        while not self.kill_the_thread:
            t_now = time.time()

            # Cyclic Refresh
            if (t_now - t_cyclic) >= t_sample:
                dt = 0.001
                t_cyclic = t_now

                with self.lock:
                    deg_list = []
                    for i in range(self.actuator_count):
                        curr = (self.current_position[i] + 180.0) % 360.0 - 180.0
                        tgt  = self.target_position[i]
                        # max_step = 1.0
                        # step = np.clip(tgt - curr, -max_step, max_step)
                        # deg = curr + step
                        # # ma_deque[i].append(deg)
                        # deg = float(np.mean(ma_deque[i])) if len(ma_deque[i]) > 0 else deg
                        # self.base_command.actuators[i].position = deg
                        limited_target = self.limiter[i].compute(curr, tgt)
                        self.base_command.actuators[i].position = limited_target

                    # === 그리퍼 명령도 같은 프레임에 실어서 보냄 (0~100%) ===
                    self.motorcmd.position = self.target_gripper
                    self.motorcmd.velocity = self.grip_vel
                    self.motorcmd.force    = self.grip_force

                # Incrementing identifier ensure actuators can reject out of time frames
                self.base_command.frame_id += 1
                if self.base_command.frame_id > 65535:
                    self.base_command.frame_id = 0
                for i in range(self.actuator_count):
                    self.base_command.actuators[i].command_id = self.base_command.frame_id

                # interconnect/그리퍼 command_id도 같이 갱신
                self.base_command.interconnect.command_id.identifier = self.base_command.frame_id
                self.base_command.interconnect.gripper_command.command_id.identifier = self.base_command.frame_id

                # Frame is sent
                try:
                    self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)
                    with self.lock:
                        self.current_position = [self.base_feedback.actuators[i].position
                                                for i in range(self.actuator_count)]
                        
                    try:
                        self.current_gripper = self.base_feedback.interconnect.gripper_feedback.motor[0].position
                    except Exception:
                        pass

                except:
                    failed_cyclic_count = failed_cyclic_count + 1
                cyclic_count = cyclic_count + 1

                # print("current position: ", self.current_position, end="\r")
                # print("target position: ", self.target_position, end="\r")
            # Stats Print
            if print_stats and ((t_now - t_stats) > 1):
                t_stats = t_now
                stats_count = stats_count + 1
                
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

            example = TorqueExample(router, router_real_time)

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
