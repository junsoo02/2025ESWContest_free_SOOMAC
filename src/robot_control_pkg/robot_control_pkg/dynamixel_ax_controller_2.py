import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from dynamixel_sdk import *

# 다이나믹셀 ID 설정
DXL_ID_List         = [1,2,3,4,10] # 사용할 Dynamixel의 아이디를 기입해주세요
# 다이나믹셀 Port 설정
DXL_PORT            = "/dev/ttyUSB0" # 사용할 Dynamixel의 포트를 기입해주세요

GOAL_POSITION       = 116       # AX-12 Goal Position Register
PRESENT_POSITION    = 132       # AX-12 Present Position Register
ADDR_TORQUE_ENABLE      = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY     = 112

PROFILE_ACCELERATION = 5
PROFILE_VELOCITY = 100

PROTOCOL_VERSION    = 2.0      # AX-12 Procotol Version
BAUDRATE            = 1000000  # AX-12 BaudRate

XC_ID_R = 4
XC_ID_L = 5

XC_PORT = '/dev/ttyUSB0'

XC_PROTOCOL_VERSION = 2.0
XC_BAUDRATE = 115200

XC_GOAL_POSITION = 116
XC_PRESENT_POSITION = 132

Mode = 1

class DynamixelAxController2(Node):
    # Node init
    def __init__(self):
        super().__init__('dynamixel_ax_controller_2')
        
        self.port_handler = PortHandler(DXL_PORT)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        
        if not self.port_handler.openPort():
            self.get_logger().error("포트를 열 수 없습니다.")
            
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("보드레이트 설정에 실패하였습니다.")
            
        if Mode == 2:
            self.xc_port = PortHandler(XC_PORT)
            self.xc_packet = PacketHandler(XC_PROTOCOL_VERSION)
            
            if not self.xc_port.openPort():
                self.get_logger().error("XC 모터 포트를 열 수 없습니다.")
            
            if not self.xc_port.setBaudRate(XC_BAUDRATE):
                self.get_logger().error("XC 모터 보드레이트 설정에 실패하였습니다.")
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/Topic_target_angle',
            self.callback,
            10)
        self.subscription

        self.position_publisher = self.create_publisher(Float32MultiArray, '/Topic_current_position', 10)

        self.timer = self.create_timer(0.05, self.publish_positions)
        
        self.get_logger().info("다이나믹셀 연결 성공")
        
        self.first_angle = None
        
        self.id_list_12310 = DXL_ID_List[:-2] + DXL_ID_List[-1:]
        
        for motor_id in self.id_list_12310:
            # Disable torque to change settings
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)

            # Set Profile Acceleration
            self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION)

            # Set Profile Velocity
            self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)

            # Enable torque after settings
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 1)
            
        # Disable torque to change settings
        self.packet_handler.write1ByteTxRx(self.port_handler, 4, ADDR_TORQUE_ENABLE, 0)

        # Set Profile Acceleration
        self.packet_handler.write4ByteTxRx(self.port_handler, 4, ADDR_PROFILE_ACCELERATION, 50)

        # Set Profile Velocity
        self.packet_handler.write4ByteTxRx(self.port_handler, 4, ADDR_PROFILE_VELOCITY, 125)

        # Enable torque after settings
        self.packet_handler.write1ByteTxRx(self.port_handler, 4, ADDR_TORQUE_ENABLE, 1)
        
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
    
    # Callback 함수
    def callback(self, msg):
        angle = msg.data
        position = self.angle_to_position(angle)
        
        self.get_logger().info(f'수신한 각도 : {angle}')
        
        for i, motor_id in enumerate(DXL_ID_List):
            goal_pos = int(position[i])  # float -> int 변환
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, GOAL_POSITION, goal_pos)
            if dxl_comm_result != COMM_SUCCESS:
              self.get_logger().error('다이나믹셀 Target Angle 전송 실패')
            elif dxl_error != 0:
              self.get_logger().error('명령 에러 발생')
            else:
              self.get_logger().info('목표 각도 전송 성공')
        
    def read_present_positions(self):
        positions = []
        for motor_id in [1,2,3,4,10]:
            dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().warn(f"[ID:{motor_id}] 현재 위치 읽기 실패: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
                positions.append(-1)
            elif dxl_error != 0:
                self.get_logger().warn(f"[ID:{motor_id}] 에러 발생: {self.packet_handler.getRxPacketError(dxl_error)}")
                positions.append(-1)
            else:
                positions.append(dxl_present_position)
        return positions
    
    def position_to_angle(self, positions):
        angles = []
        for pos in positions:
            if pos == -1:
                angles.append(-1.0)  # 읽기 실패 시 -1
            else:
                angle = pos * (360.0 / 4095)
                angles.append(round(angle, 2))  # 소수점 둘째 자리까지
        return angles
    
    def publish_positions(self):
        raw_positions = self.read_present_positions()
        angles = self.position_to_angle(raw_positions)
        msg = Float32MultiArray()
        msg.data = angles
        self.position_publisher.publish(msg)
        self.get_logger().info(f"현재 위치(degree) 발행: {angles}")   
            
    def destroy_node(self):
        self.port_handler.closePort()
        super().destroy_node()
        
# Main 함수 
def main(args=None):
    rclpy.init(args=args)
    node = DynamixelAxController2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()