import os, sys
import time

sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

module_directory = os.path.expanduser("~/dynamixel_sdk")
sys.path.append(module_directory)

from dynamixel_sdk import *
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.packet_handler import PacketHandler

from dynamixel_sdk.robotis_def import *

DEVICENAME = '/dev/ttyACM0'
# DEVICENAME = '/dev/ttyUSB0'

#**********************Xl330-M280-T(PROTOCOL_VERSION 2.0)**************************#
# Write Data
XL_ADDR_TORQUE_ENABLE          = 64
XL_ADDR_OPERATION_MODE         = 11
XL_ADDR_GOAL_POSITION          = 116
XL_LEN_GOAL_POSITION           = 4         # Data Byte Length
XL_ADDR_GOAL_CURRENT           = 102
XL_LEN_GOAL_CURRENT            = 2         # Data Byte Length

# Read Data
XL_ADDR_START = 126
XL_TOTAL_READ_LEN = 10
XL_OFFSET_CURRENT = 0
XL_OFFSET_VELOCITY = 2
XL_OFFSET_POSITION = 6

# Value
XL_TORQUE_ENABLE               = 1                 # Value for enabling the torque
XL_TORQUE_DISABLE              = 0                 # Value for disabling the torque
XL_CURRENT_MODE = 0
XL_POSITION_MODE = 3
XL_CURRENT_BASED_POSITION_MODE = 5

# Motor setting
XL_OPERATION_MODE = [XL_CURRENT_MODE, # 0 
                     XL_CURRENT_MODE, # 1
                     XL_CURRENT_MODE, # 2
                     XL_CURRENT_MODE, # 3
                     XL_CURRENT_MODE, # 4
                     XL_CURRENT_MODE, # 5                                          
                     XL_CURRENT_BASED_POSITION_MODE] # 6(gripper)

BAUDRATE                    = 1000000
PROTOCOL_VERSION            = 2.0

DOF = 6
XL_DXL_ID = [0, 1, 2, 3, 4, 5, 6]
XL_DXL_JOINT = XL_DXL_ID[:DOF]
XL_DXL_GRIPPER = XL_DXL_ID[DOF]

class DynamixelControl:
    def __init__(self): 
        self.dynamixel_setting() 

    def dynamixel_setting(self): # 다이나믹셀 세팅
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
        
        # BulkRead & Write setting
        self.bulk_read = GroupBulkRead(self.port_handler_xl, self.packet_handler_xl)
        for dxl_id in XL_DXL_ID:
            if not self.bulk_read.addParam(dxl_id, XL_ADDR_START, XL_TOTAL_READ_LEN):
                print(f"[ID:{dxl_id}] Failed to add param")
                exit()

    def read_degree(self, dxl_id):
        # BulkRead
        if self.bulk_read.txRxPacket() != COMM_SUCCESS:
            print("BulkRead communication failed")
            return None
        
        if not self.bulk_read.isAvailable(dxl_id, XL_ADDR_START, XL_TOTAL_READ_LEN):
            print(f"[ID:{dxl_id}] Data not available")
            return None
        
        # position
        dxl_value = self.bulk_read.getData(dxl_id, XL_ADDR_START + XL_OFFSET_POSITION, 4)
        if dxl_value >= 2147483648:
            dxl_value -= 4294967296
        
        return dxl_value
    
    def close_port(self):
        """다이나믹셀 통신 포트를 닫습니다."""
        self.port_handler_xl.closePort()
        print("Dynamixel port closed.")


def main(args=None):
    dynamixel_control = DynamixelControl()
    # 추가된 close_port 테스트
    try:
        while True:
            # 예시: ID 0의 각도 읽기
            angle = dynamixel_control.read_degree(0)
            if angle is not None:
                print(f"ID 0 Angle: {angle}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping.")
    finally:
        dynamixel_control.close_port()

if __name__ == '__main__':
    # main()
    pass