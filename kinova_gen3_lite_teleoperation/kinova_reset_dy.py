#!/usr/bin/env python3

import sys
import os
import time
import threading
import numpy as np

# --- 시스템 경로 설정 ---
# sys.path.append(os.path.dirname(__file__))
# sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
# module_directory = os.path.expanduser("~/dynamixel_sdk")
# sys.path.append(module_directory)

# --- 다이나믹셀 SDK 및 Kinova API 임포트 ---
from dynamixel_sdk import *
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.packet_handler import PacketHandler
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2
import utilities

# =============================================================================
# --- 다이나믹셀 상수 정의 ---
# =============================================================================
DEVICENAME                  = '/dev/ttyACM0'  # 💻 실제 포트 이름으로 변경 필요
BAUDRATE                    = 1000000
PROTOCOL_VERSION            = 2.0

# --- 주소 및 길이 정의 (XL-330 기준) ---
XL_ADDR_TORQUE_ENABLE       = 64
XL_ADDR_OPERATION_MODE      = 11
XL_ADDR_GOAL_POSITION       = 116
XL_LEN_GOAL_POSITION        = 4

# --- 값 정의 ---
XL_TORQUE_ENABLE            = 1
XL_TORQUE_DISABLE           = 0
XL_POSITION_MODE            = 3  # 위치 제어 모드

# --- 모터 ID 및 동작 모드 설정 ---
XL_DXL_ID = [0, 1, 2, 3, 4, 5, 6]
# 모든 모터를 위치 제어 모드로 통일
OPERATION_MODES = {
    0: XL_POSITION_MODE, 1: XL_POSITION_MODE, 2: XL_POSITION_MODE,
    3: XL_POSITION_MODE, 4: XL_POSITION_MODE, 5: XL_POSITION_MODE,
    6: XL_POSITION_MODE
}

# --- ⭐ 마스터 로봇팔 시작 자세 설정 (Raw 값) ---
# [J1, J2, J3, J4, J5, J6, Gripper]
MASTER_HOME_POSITION_RAW = [int((23.568113+180)/360*4096), int((2.1066625+180)/360*4096), int((86.63719+180)/360*4096), int((-92.21565)/360*4096), int((-93.08729+180)/360*4096), int((100.589325)/360*4096), 1817]


# =============================================================================
# --- 개선된 DynamixelControl 클래스 ---
# =============================================================================
class DynamixelControl:
    """마스터 로봇팔(다이나믹셀) 제어를 위한 클래스"""
    def __init__(self):
        # XL 모터와 통신을 위한 포트 핸들러 및 패킷 핸들러 초기화
        self.port_handler_xl = PortHandler(DEVICENAME)
        self.packet_handler_xl = PacketHandler(PROTOCOL_VERSION)
                
        # XL 모터 포트 열기
        if self.port_handler_xl.openPort():            
            print("Successfully opened the XL port.")
        else:
            print("Failed to open the XL port.")
            return

        # XL 모터 통신 속도 설정
        if self.port_handler_xl.setBaudRate(BAUDRATE):
            print("Successfully set the XL baudrate.")
        else:
            print("Failed to set the XL baudrate.")
            return
        
        # torque disable
        for dxl_id in XL_DXL_ID:
            dxl_comm_result, dxl_error = self.packet_handler_xl.write1ByteTxRx(self.port_handler_xl, dxl_id, XL_ADDR_TORQUE_ENABLE, XL_TORQUE_DISABLE)

            if dxl_comm_result != COMM_SUCCESS:
                print("Failed to disable torque for XL Motor ID: {}".format(dxl_id))
                return
            else:
                print("Torque disabled for XL Motor ID: {}".format(dxl_id))
                
        # operation mode               
        for dxl_id in XL_DXL_ID:                 
            dxl_comm_result, dxl_error = self.packet_handler_xl.write1ByteTxRx(self.port_handler_xl, dxl_id, XL_ADDR_OPERATION_MODE, OPERATION_MODES[dxl_id])

            if dxl_comm_result != COMM_SUCCESS:
                print("Failed to change operation mode for XL Motor ID: {}".format(dxl_id))
                return
            else:
                print("Change operation mode to {} for XL Motor ID: {}".format(OPERATION_MODES[dxl_id], dxl_id))
        
        # torque enable
        for dxl_id in XL_DXL_ID:   
            dxl_comm_result, dxl_error = self.packet_handler_xl.write1ByteTxRx(self.port_handler_xl, dxl_id, XL_ADDR_TORQUE_ENABLE, XL_TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("Failed to enable torque for XL Motor ID: {}".format(dxl_id))
                return
            else:
                print("Torque enabled for XL Motor ID: {}".format(dxl_id))
        print("*****************")
                
        self.bulk_write = GroupBulkWrite(self.port_handler_xl, self.packet_handler_xl) 

    def _execute_bulk_write(self, bulk_write, error_message):
        """Helper: GroupBulkWrite를 실행하고 결과를 확인합니다."""
        dxl_comm_result = bulk_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"{error_message}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            return False
        return True

    def move_to_home_position(self, home_positions_raw):
        """지정된 Raw 값으로 마스터 로봇팔을 이동시킵니다."""
        print(f"[Master Arm] Sending to home position...")

        for i, dxl_id in enumerate(XL_DXL_ID):
            # 목표 위치 값을 4바이트 little-endian 배열로 변환
            goal_pos_bytes = [
                DXL_LOBYTE(DXL_LOWORD(home_positions_raw[i])),
                DXL_HIBYTE(DXL_LOWORD(home_positions_raw[i])),
                DXL_LOBYTE(DXL_HIWORD(home_positions_raw[i])),
                DXL_HIBYTE(DXL_HIWORD(home_positions_raw[i]))
            ]
            self.bulk_write.addParam(dxl_id, XL_ADDR_GOAL_POSITION, XL_LEN_GOAL_POSITION, bytearray(goal_pos_bytes))

        if not self._execute_bulk_write(self.bulk_write, "[Master Arm Error] Failed to send home position"):
            return
        
        print("[Master Arm] Home position command sent. Waiting for movement...")
        time.sleep(2.0) # 모터가 움직일 시간을 줍니다.

    def release_port(self):
        """통신 포트만 닫습니다. 모터 토크는 유지됩니다."""
        self.port_handler_xl.closePort()
        print("[Master Arm] Port closed. Torque remains ON.")


# =============================================================================
# --- Kinova 제어 함수 ---
# =============================================================================
TIMEOUT_DURATION = 30

def wait_for_action_end(base):
    e = threading.Event()
    def check(notification, e=e):
        if notification.action_event in [Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT]:
            e.set()
    handle = base.OnNotificationActionTopic(check, Base_pb2.NotificationOptions())
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(handle)
    return finished

def send_joint_angles(base, joint_angles_deg):
    """지정된 각도(degree)로 슬레이브 로봇팔을 이동시킵니다."""
    # joint_angles_rad = np.deg2rad(joint_angles_deg)  # Convert degrees to radians

    # Waypoint list 준비
    waypoints = Base_pb2.WaypointList()
    waypoints.duration = 0.0
    waypoints.use_optimal_blending = False

    waypoint = waypoints.waypoints.add()
    waypoint.name = "target_pose"
    waypoint.angular_waypoint.angles.extend(joint_angles_deg)
    waypoint.angular_waypoint.duration = 5.0  # seconds

    print("[Command] Executing angular waypoint trajectory...")
    base.ExecuteWaypointTrajectory(waypoints)

    if wait_for_action_end(base):
        print("[Result] Joint movement completed.")
    else:
        print("[Error] Timeout while waiting for joint movement to finish.")
def open_gripper(base):
    cmd = Base_pb2.GripperCommand()
    cmd.mode = Base_pb2.GRIPPER_POSITION
    finger = cmd.gripper.finger.add()
    finger.finger_identifier = 0
    finger.value = 0.0  # Fully open
    print("[Slave Arm] Opening gripper...")
    base.SendGripperCommand(cmd)
    time.sleep(2.0)
    print("[Slave Arm] Gripper opened.")


# =============================================================================
# --- 메인 실행 함수 ---
# =============================================================================
def main():
    args = utilities.parseConnectionArguments()
    master_arm = None

    try:
        # --- 1. 마스터 암 초기화 및 홈 포지션 이동 ---
        master_arm = DynamixelControl()
        master_arm.move_to_home_position(MASTER_HOME_POSITION_RAW)
        
        # --- 2. 슬레이브 암 초기화 및 리셋 포지션 이동 ---
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            base = BaseClient(router)
            
            # 슬레이브 로봇팔 목표 위치
            target_joint_angles_deg = [23.568113, 2.1066625, 86.63719, -92.21565, -93.08729, 100.589325]
            
            print("\n--- STARTING ROBOT RESET ---")
            start_time = time.time()

            send_joint_angles(base, target_joint_angles_deg)
            open_gripper(base)
            
            # 슬레이브가 움직이는 동안 마스터가 자세를 유지하도록 한 번 더 명령
            master_arm.move_to_home_position(MASTER_HOME_POSITION_RAW)

            cycle_time = time.time() - start_time
            print(f"\n--- ✅ RESET COMPLETE (Total Time: {cycle_time:.2f}s) ---")

    except Exception as e:
        print(f"❌ An error occurred: {e}")
    finally:
        # 스크립트 종료 시 토크는 유지하고 포트만 닫습니다.
        if master_arm:
            master_arm.release_port()

if __name__ == "__main__":
    main()
