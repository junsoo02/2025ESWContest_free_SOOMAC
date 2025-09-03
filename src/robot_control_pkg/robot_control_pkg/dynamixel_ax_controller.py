import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

from dynamixel_sdk import *

# --- XM430 기본 설정 ---
ADDR_TORQUE_ENABLE      = 64
ADDR_CURRENT_LIMIT      = 38
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_PRESENT_CURRENT    = 126
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY     = 112
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_OPERATING_MODE  = 11

PROTOCOL_VERSION        = 2.0
DXL_ID_LIST             = [1, 2, 3, 4, 10]   # 모터 ID
BAUDRATE                = 1000000
DEVICENAME              = '/dev/ttyUSB0'

OP_MODE_CURR_POSITION    = 5 
TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0

# --- 개별 전류 제한 (Decimal 단위) ---
CURRENT_LIMITS = [160, 1000, 700, 150, 55]   # 모터별 전류 제한 값

class DynamixelAxController(Node):
    # Node init
    def __init__(self):
        super().__init__('dynamixel_ax_controller')
        
        # --- 포트 및 패킷 핸들러 생성 ---
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            raise Exception("포트 열기 실패")

        if not self.portHandler.setBaudRate(BAUDRATE):
            raise Exception("Baudrate 설정 실패")

        # 각 모터 토크 활성화 + 개별 전류 제한 설정
        for dxl_id, current_limit in zip(DXL_ID_LIST, CURRENT_LIMITS):
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, OP_MODE_CURR_POSITION)
            self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, ADDR_CURRENT_LIMIT, current_limit)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, 4)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_PROFILE_VELOCITY, 90)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        self.packetHandler.write2ByteTxRx(self.portHandler, 3, ADDR_POSITION_P_GAIN, 1400)
        self.packetHandler.write2ByteTxRx(self.portHandler, 3, ADDR_POSITION_I_GAIN, 50)
        self.packetHandler.write2ByteTxRx(self.portHandler, 2, ADDR_POSITION_P_GAIN, 2000)
        self.packetHandler.write2ByteTxRx(self.portHandler, 2, ADDR_POSITION_I_GAIN, 50)
        self.packetHandler.write4ByteTxRx(self.portHandler, 4, ADDR_PROFILE_ACCELERATION, 15)

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/Topic_target_angle',
            self.callback,
            10)
        self.subscription

        self.position_publisher = self.create_publisher(Float32MultiArray, '/Topic_current_position', 10)
        self.current_publisher = self.create_publisher(Float32, '/Topic_current', 10)

        self.timer = self.create_timer(0.05, self.publish_positions)
        self.timer_current = self.create_timer(0.05, self.publish_current)
        
        self.get_logger().info("다이나믹셀 연결 성공")
        
        self.first_angle = None
        
    # 각도 변환 함수    
    def angle_to_position(self, angle):
        position = []
        
        for i in range(5):
          if angle[i] < 0 :
              angle[i] = 0
          elif angle[i] > 360:
              angle[i] = 360
          position.append(int(angle[i]* (4095/360)))    
        
        return position
    
    def position_to_angle(self, positions):
        angles = []
        for pos in positions:
            if pos == -1:
                angles.append(-1.0)  # 읽기 실패 시 -1
            else:
                angle = pos * (360.0 / 4095)
                angles.append(round(angle, 2))  # 소수점 둘째 자리까지
        return angles
    
    def read_present_positions(self):
        positions = []
        for motor_id in DXL_ID_LIST:
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor_id, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn(f"[ID:{motor_id}] 현재 위치 읽기 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                positions.append(-1)
            elif dxl_error != 0:
                self.get_logger().warn(f"[ID:{motor_id}] 에러 발생: {self.packetHandler.getRxPacketError(dxl_error)}")
                positions.append(-1)
            else:
                positions.append(dxl_present_position)
        return positions
    
    # Callback 함수
    def callback(self, msg):
        angle = msg.data
        position = self.angle_to_position(angle)
        new_position = [p for i, p in enumerate(position) if i != 4]
        
        self.get_logger().info(f'수신한 각도 : {angle}')

        tolerance = 15

        i = 0

        while True:
            all_reached = True
            raw_positions = self.read_present_positions()
            new_raw_position = [p for i, p in enumerate(raw_positions) if i != 4]
            for dxl_id, target_pos, present_pos in zip([1, 2, 3, 4], new_position, new_raw_position):
                
                error = target_pos - present_pos

                goal_pos = int(target_pos)  # float -> int 변환
                if dxl_id == 3 and goal_pos < 686:
                    goal_pos = 686
                    self.get_logger().info('Motor 3: Not reached')
                    error = 0
                
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, goal_pos)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().error('다이나믹셀 Target Angle 전송 실패')
                elif dxl_error != 0:
                    self.get_logger().error('명령 에러 발생')
                else:
                    self.get_logger().info('목표 각도 전송 성공')

                pos_list = [present_pos, target_pos, error]
                ang_list = self.position_to_angle(pos_list)

                print(f"[ID {dxl_id}] 현재 위치: {ang_list[0]}°, 목표: {ang_list[1]}°, 오차: {ang_list[2]}°")
                i = i + 1
                self.get_logger().info(f"루프 반복 횟수: {i}")

                if abs(error) >= tolerance:
                    all_reached = False

            self.packetHandler.write4ByteTxRx(self.portHandler, 10, ADDR_GOAL_POSITION, position[4])

            if all_reached:
                print("모든 모터가 목표 위치에 도달했습니다.")
                break
    
    def publish_positions(self):
        raw_positions = self.read_present_positions()
        angles = self.position_to_angle(raw_positions)
        msg = Float32MultiArray()
        msg.data = angles
        self.position_publisher.publish(msg)
        self.get_logger().info(f"현재 위치(degree) 발행: {angles}")

    def publish_current(self):   
        msg = Float32()
        current_mA = 0.0
        dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
            self.portHandler, 10, ADDR_PRESENT_CURRENT
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"{self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.get_logger().error(f"{self.packetHandler.getRxPacketError(dxl_error)}")
        else:
            current_mA = dxl_present_current * 2.69
            self.get_logger().info(
                f"Present Current(raw): {dxl_present_current}, "
                f"Present Current(mA): {current_mA:.2f} mA"
            )
        msg.data = current_mA
        self.current_publisher.publish(msg)
            
    def destroy_node(self):
        self.portHandler.closePort()
        super().destroy_node()
        
# Main 함수 
def main(args=None):
    rclpy.init(args=args)
    node = DynamixelAxController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()