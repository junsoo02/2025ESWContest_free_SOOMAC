import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32,Bool
import serial
import time

class LiftControllerNode(Node):
    def __init__(self):
        super().__init__('lift_controller_node')
        
          # 시리얼 연결 (아두이노 포트와 속도 확인)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)
        
        # 전역 변수
        self.lift_position = 0  # 초기 위치는 0
        self.lift_moving = False
        
        # 구독자(Subscriber) 설정
        self.subscription_lift = self.create_subscription(Int32, 'scout_lift', self.lift_callback, 10)
        self.subscription_control = self.create_subscription(Int32, 'scout_control', self.scout_control_callback, 10)
        
        # 발행자(Publisher) 설정
        self.publisher_next_step1 = self.create_publisher(Int32, 'next_step1', 10)
        self.scout_realsense_pub = self.create_publisher(Int32, 'scout_realsense', 10)

        #bool topic
        self.lift_position_pub = self.create_publisher(Bool, 'lift_position', 10)
        
    def lift_callback(self, data):
        command = data.data
        
        if self.lift_moving:
            self.get_logger().info("리프트가 이미 작동 중입니다. 새로운 명령을 무시합니다.")
            return
        
        if command == 33:
            # 리프트가 내려갈 때 (33일 때만 리얼센스 작동)
            if self.lift_position > 0:
                self.get_logger().info("(down)")
                self.lift_position -= 1
                self.lift_moving = True
                self.get_logger().info(f"리프트가 아래로 이동합니다. 현재 위치: {self.lift_position}")
                self.ser.write(b'44\n')
                time.sleep(1.8)

                #bool false
                bool_msg = Bool()
                bool_msg.data = False
                self.lift_position_pub.publish(bool_msg)
                self.get_logger().info("lift_position 토픽에 False를 발행했습니다.")


                self.lift_moving = False
                self.scout_realsense_pub.publish(Int32(data=44))
                self.get_logger().info("리프트 하강 완료, scout_realsense에 44를 보냈습니다.")
            else:
                self.get_logger().info("리프트가 가장 낮은 위치(0)에 있어 더 이상 내려갈 수 없습니다.")
        
        elif command == 55:
            if self.lift_position > 0:
                self.get_logger().info("(down)")
                self.lift_position -= 1
                self.lift_moving = True
                self.get_logger().info(f"리프트가 아래로 이동합니다. 현재 위치: {self.lift_position}")
                self.ser.write(b'44\n')
                time.sleep(2.0)
                self.lift_moving = False
                
                # 55 명령을 받았을 때만 다음 스텝으로 30을 보냄
                num_signals = 5  #5번 반복
                for i in range(num_signals):
                    self.publisher_next_step1.publish(Int32(data=30))
                    self.get_logger().info(f"next_step1 토픽으로 30을 보냈습니다. (횟수: {i+1})")
            else:
                self.get_logger().info("리프트가 가장 낮은 위치(0)에 있어 더 이상 내려갈 수 없습니다.")
        

        elif command == 44:
            # 리프트가 올라갈 때 (1에서 더 이상 올라가지 않도록)
            if self.lift_position < 1:
                self.get_logger().info("(up)")
                self.lift_position += 1
                self.lift_moving = True
                self.get_logger().info(f"리프트가 위로 이동합니다. 현재 위치: {self.lift_position}")
                self.ser.write(b'33\n')
                time.sleep(2.0)

                #bool false
                bool_msg = Bool()
                bool_msg.data = True
                self.lift_position_pub.publish(bool_msg)
                self.get_logger().info("lift_position 토픽에 False를 발행했습니다.")


                self.lift_moving = False

                self.scout_realsense_pub.publish(Int32(data=44))
                self.get_logger().info("리프트 상승 완료, scout_realsense에 44를 보냈습니다.")
                
            else:
                self.get_logger().info("리프트가 가장 높은 위치(1)에 있어 더 이상 올라갈 수 없습니다.")
        else:
            self.get_logger().info(f"알 수 없는 명령 코드: {command}")


    def scout_control_callback(self, data):
        command = data.data
        
        if command == 21:
            self.get_logger().info("리프트 작동 중지")
            self.lift_moving = False
            msg = Int32()
            msg.data = 10
            
            
            num_signals = 5  
            for i in range(num_signals):
                self.publisher_next_step1.publish(msg)
                self.get_logger().info("next_step1 토픽으로 10을 보냈습니다.")


def main(args=None):
    rclpy.init(args=args)
    lift_controller_node = LiftControllerNode()
    rclpy.spin(lift_controller_node)
    lift_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
