import time
import numpy as np
from dynamixel_sdk import *

# --- XM430 기본 설정 ---
ADDR_TORQUE_ENABLE      = 64
ADDR_CURRENT_LIMIT      = 38
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132

PROTOCOL_VERSION        = 2.0
DXL_ID_LIST             = [1, 2, 3, 4, 10]   # 모터 ID
BAUDRATE                = 1000000
DEVICENAME              = '/dev/ttyUSB0'

TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0

# --- 개별 목표 각도 (degree 단위) ---
TARGET_POSITIONS_DEG = [180, 180, 165, 180, 90]
TARGET_POSITIONS_RAD = [np.deg2rad(deg) for deg in TARGET_POSITIONS_DEG]

# --- 개별 전류 제한 (Decimal 단위) ---
CURRENT_LIMITS = [160, 800, 300, 50, 80]   # 모터별 전류 제한 값

# --- 포트 및 패킷 핸들러 생성 ---
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    raise Exception("포트 열기 실패")

if not portHandler.setBaudRate(BAUDRATE):
    raise Exception("Baudrate 설정 실패")

# 각 모터 토크 활성화 + 개별 전류 제한 설정
for dxl_id, current_limit in zip(DXL_ID_LIST, CURRENT_LIMITS):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_CURRENT_LIMIT, current_limit)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

def rad_to_raw(rad):
    val = int((rad / (2 * np.pi)) * 4095)
    return max(0, min(4095, val))

def get_present_position(dxl_id):
    pos_raw, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    pos_rad = (pos_raw / 4095) * 2 * np.pi
    return pos_rad

def send_goal_position(dxl_id, rad):
    pos_val = rad_to_raw(rad)
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, pos_val)

# 제어 루프
dt = 0.05
tolerance = np.deg2rad(0.5)

while True:
    all_reached = True
    for dxl_id, target_rad, target_deg in zip(DXL_ID_LIST, TARGET_POSITIONS_RAD, TARGET_POSITIONS_DEG):
        present_pos = get_present_position(dxl_id)
        error = target_rad - present_pos

        send_goal_position(dxl_id, target_rad)

        print(f"[ID {dxl_id}] 현재 위치: {np.rad2deg(present_pos):.2f}°, 목표: {target_deg}°, 오차: {np.rad2deg(error):.2f}°")

        if abs(error) >= tolerance:
            all_reached = False

    if all_reached:
        print("모든 모터가 목표 위치에 도달했습니다.")
        break

    time.sleep(dt)

# 토크 비활성화 및 포트 닫기
#for dxl_id in DXL_ID_LIST:
#    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
portHandler.closePort()