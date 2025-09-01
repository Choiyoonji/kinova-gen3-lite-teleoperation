#!/usr/bin/env python3
import os
import sys
import time
from collections import deque
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
module_directory = os.path.expanduser("~/dynamixel_sdk")
sys.path.append(module_directory)

# --- Dynamixel SDK ---
from dynamixel_sdk import *  # noqa

XL_DXL_JOINT = [0,1,2,3,4,5,6]
# =========================
# 하드웨어/프로토콜 설정값
# =========================
DEVICENAME = "/dev/ttyACM0"   # 실제 포트로 변경
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

# 주소/길이 (XL-330 기준)
XL_ADDR_TORQUE_ENABLE     = 64
XL_ADDR_PRESENT_POSITION  = 132
XL_LEN_PRESENT_POSITION   = 4

# 값
XL_TORQUE_ENABLE  = 1
XL_TORQUE_DISABLE = 0

# =========================
# 보정/스케일링 설정값
# =========================
# 예시: 홈 포즈 raw 값 (사용자가 이미 갖고 있는 값으로 교체하세요)
MASTER_HOME_POSITION_RAW = [268, 2072, 3032, 999, 989, 3191, 1781]  # 7개 사용
MASTER_JOINT_ZERO_RAW = np.array(MASTER_HOME_POSITION_RAW[:len(XL_DXL_JOINT)])
MASTER_RAW_PER_DEGREE = 4096.0 / 360.0
JOINT_DIRECTION_MAPPING = np.array([1, 1, 1, 1, 1, 1, 1], dtype=float)  # 부호/감도 조정 시 수정

def wrap_to_180_array(deg_arr: np.ndarray) -> np.ndarray:
    return (deg_arr + 180.0) % 360.0 - 180.0

# =========================
# Dynamixel Bulk Reader
# =========================
class DynamixelReader:
    """마스터 다이나믹셀 팔에서 6개 관절+그리퍼 raw 위치를 BulkRead로 읽는 간단 클래스."""
    def __init__(self, device=DEVICENAME, baudrate=BAUDRATE):
        self.port_handler = PortHandler(device)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.bulk_read = GroupBulkRead(self.port_handler, self.packet_handler)

        if not self.port_handler.openPort():
            raise IOError(f"Failed to open port: {device}")
        if not self.port_handler.setBaudRate(baudrate):
            raise IOError(f"Failed to set baudrate: {baudrate}")

        # BulkRead 파라미터 등록
        for dxl_id in XL_DXL_JOINT:
            if not self.bulk_read.addParam(dxl_id, XL_ADDR_PRESENT_POSITION, XL_LEN_PRESENT_POSITION):
                raise RuntimeError(f"[ID:{dxl_id}] Failed to add bulk read param")

        print(f"✅ Connected to Dynamixel ({device}, {baudrate}). Torque state unchanged.")

    def disable_all_torque(self):
        """필요 시 토크 해제(안전용). 기본은 사용 안 함."""
        bulk_write = GroupBulkWrite(self.port_handler, self.packet_handler)
        for dxl_id in XL_DXL_JOINT:
            bulk_write.addParam(dxl_id, XL_ADDR_TORQUE_ENABLE, 1, [XL_TORQUE_DISABLE])
        dxl_comm_result = bulk_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to disable torque: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
        else:
            print("🔌 Torque DISABLED for all joints.")

    def read_positions_raw(self) -> List[int]:
        """6개 관절 raw 위치 리스트 반환. 읽기 실패시 None 반환."""
        dxl_comm_result = self.bulk_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(self.packet_handler.getTxRxResult(dxl_comm_result))
            return None

        raw = []
        for dxl_id in XL_DXL_JOINT:
            if self.bulk_read.isAvailable(dxl_id, XL_ADDR_PRESENT_POSITION, XL_LEN_PRESENT_POSITION):
                pos = self.bulk_read.getData(dxl_id, XL_ADDR_PRESENT_POSITION, XL_LEN_PRESENT_POSITION)
                raw.append(pos)
            else:
                print(f"[ID:{dxl_id}] Failed to get position data.")
                return None
        return raw

    def close(self):
        try:
            self.port_handler.closePort()
        except Exception:
            pass
        print("Dynamixel port closed.")

# =========================
# ROS 2 Node
# =========================
class MasterDxlBridge(Node):
    """
    - 주기적으로 Dynamixel 6관절 raw 읽기 → deg로 변환 → /master/joint_angles_deg publish
    - /slave/joint_angles_deg 구독 → 수신값 콘솔 출력
    """
    def __init__(self,
                 device=DEVICENAME,
                 baudrate=BAUDRATE,
                 pub_topic="/target_position",
                 sub_topic="/current_position",
                 rate_hz=50.0):
        super().__init__("master_dxl_bridge")
        self.get_logger().info("MasterDxlBridge starting...")

        self.reader = DynamixelReader(device=device, baudrate=baudrate)

        # 필요시 토크 해제(데이터만 읽을 때 안전). 기본은 off.
        self.reader.disable_all_torque()

        self.pub = self.create_publisher(Float32MultiArray, pub_topic, 10)
        self.sub = self.create_subscription(Float32MultiArray, sub_topic, self.sub_callback, 10)

        self.timer = self.create_timer(1.0 / rate_hz, self._on_timer)

        # 이동평균(옵션): 노이즈 줄이고 following fault 방지에 조금 도움
        self.ma_window = 3
        self.buffers = [deque(maxlen=self.ma_window) for _ in XL_DXL_JOINT]

        # 상태
        self.last_deg = [0.0]*len(XL_DXL_JOINT)

        self.get_logger().info(f"Publishing to {pub_topic}, subscribing from {sub_topic} at {rate_hz} Hz.")

    def sub_callback(self, msg: Float32MultiArray):
        vals = [float(x) for x in msg.data]
        self.get_logger().info(f"[SUB] /slave/joint_angles_deg: {np.round(vals, 2).tolist()}")

    def _on_timer(self):
        raw: List[int] = self.reader.read_positions_raw()
        if raw is None:
            self.get_logger().warning("Failed to read master joints.")
            return

        # 유효 범위 보정(부호 있는 32비트 값으로 변환 — 일부 환경에서 필요)
        raw_arr = np.array(raw, dtype=np.int64)
        wrap_mask = raw_arr >= 2**31
        raw_arr[wrap_mask] -= 2**32

        # 0점 보정 + 스케일링(+ 방향맵)
        s_di = np.array([180, -180, 180, 0, 180, 0, -160])
        delta_raw = raw_arr
        deg = (delta_raw / MASTER_RAW_PER_DEGREE) * JOINT_DIRECTION_MAPPING
        # [-180,180) 래핑
        deg = wrap_to_180_array(deg+s_di)
        deg[-1] = np.clip(deg[-1]*5, 0, 100)  # 그리퍼는 0~100%로 제한

        # 이동평균(옵션)
        # for i in range(len(deg)):
        #     self.buffers[i].append(float(deg[i]))
        #     deg[i] = float(np.mean(self.buffers[i]))

        self.last_deg = deg.tolist()

        # Publish
        msg = Float32MultiArray()
        msg.data = self.last_deg
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.reader.close()
        finally:
            super().destroy_node()

def main():
    rclpy.init()
    node = None
    try:
        node = MasterDxlBridge(
            device=DEVICENAME,
            baudrate=BAUDRATE,
            pub_topic="/target_position",
            sub_topic="/current_position",
            rate_hz=50.0,  # 20~50Hz 권장
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
