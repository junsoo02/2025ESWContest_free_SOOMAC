#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
import time

class ScoutController(Node):
    def __init__(self):
        super().__init__('scout_controller')

        # === 1. Publisher 및 Subscriber 설정 ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.realsense_pub = self.create_publisher(Int32, '/scout_realsense', 10)
        self.lift_pub = self.create_publisher(Int32, '/scout_lift', 10)
        self.next_step_pub = self.create_publisher(Int32, '/next_step1', 10)

        self.drink_code_sub = self.create_subscription(Int32, '/drink_code', self.drink_code_callback, 10)
        self.scout_control_sub = self.create_subscription(Int32, '/scout_control', self.scout_control_callback, 10)

        # === 2. 변수 초기화 ===
        self.i = 1
        self.state = 'IDLE'
        self.scout_control_21_received = False
        self.move_timer = None
        
        self.get_logger().info('Scout Controller Node has started.')
        self.get_logger().info(f'Initial State: {self.state}, i: {self.i}')

    def drink_code_callback(self, msg):
        # <루프 a> drink_code에서 14를 받으면 초기화
        if msg.data == 14:
            self.get_logger().info('Received 14 from drink_code. Initializing Loop A.')
            self.i = 1
            self.state = 'IDLE'
            self.scout_control_21_received = False
        
        # drink_code에서 13을 받으면 최종 이동 및 로봇팔 제어 시작
        if msg.data == 13 and self.state == 'WAITING_FOR_13':
            self.get_logger().info('Received 13 from drink_code. Starting final movement.')
            self.perform_final_movement()
    
    def scout_control_callback(self, msg):
        # scout_control 토픽 22 받음 (21 신호 받기 전까지)
        if msg.data == 22 and not self.scout_control_21_received and self.state == 'IDLE':

            if self.i < 6:
                self.get_logger().info(f'Received 22 from scout_control. Current i: {self.i}')
                self.handle_scout_movement()
            else:
                self.get_logger().warn('Scout has completed all 6 zones. Ignoring 22 signal.')


        # 리얼센스 객체 인식 완료 신호
        if msg.data == 21 and not self.scout_control_21_received:
            self.get_logger().info('Received 21 from scout_control. Realsense operation complete.')
            self.scout_control_21_received = True
            self.stop_scout_and_send_to_arm()
    
    def handle_scout_movement(self):
        

        cmd = Twist()
        
        # 구역 2,4: 스캇 전진 및 리얼센스 작동
        if self.i in [2, 4]:
            self.get_logger().info(f'i={self.i}: Moving forward for 1.3 seconds.')
            self.state = 'MOVING_FORWARD'
            self.start_move_timer(duration=1, speed=0.15)

        # 구역 1,5: 리프트 작동
        elif self.i in [1, 5]:
            self.get_logger().info(f'i={self.i}: Sending 44 to scout_lift.')
            msg = Int32(data=44)
            self.lift_pub.publish(msg)
            self.get_logger().info(f'i={self.i + 1}')
            self.state = 'IDLE' # 다음 22 신호를 기다림


        # 구역 3: 리프트 작동
        elif self.i == 3:
            self.get_logger().info(f'i={self.i}: Sending 33 to scout_lift.')
            msg = Int32(data=33)
            self.lift_pub.publish(msg)
            self.get_logger().info(f'i={self.i + 1}')
            self.state = 'IDLE' # 다음 22 신호를 기다림
            
        # 6구역에서 인식 xx일 경우 제자리로 돌아간다 
        elif self.i ==6:
            self.self.get_logger().info(f'i={self.i}: Sending 33 to scout_lift.')
            msg = Int32(data=33)
            self.lift_pub.publish(msg)
            self.get_logger().info(f'i={self.i - 6}')
            self.state = 'IDLE' # 다음 22 신호를 기다림
            
        
        self.i += 1
    
    def start_move_timer(self, duration, speed):
        # 기존 타이머가 있으면 취소
        if self.move_timer:
            self.move_timer.cancel()
        
        self.move_start_time = self.get_clock().now().nanoseconds / 1e9
        self.move_duration = duration
        self.move_speed = speed
        self.move_timer = self.create_timer(0.05, self.move_callback)

    def move_callback(self):
        elapsed_time = (self.get_clock().now().nanoseconds / 1e9) - self.move_start_time
        cmd = Twist()
        
        if elapsed_time < self.move_duration:
            cmd.linear.x = self.move_speed
            self.cmd_pub.publish(cmd)
        else:
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd)
            self.move_timer.cancel()
            self.move_timer = None
            self.state = 'IDLE' # 다음 22 신호를 기다림
            self.get_logger().info('Movement complete. Stopping.')
            
            # 이동이 끝나면 리얼센스 작동
            if self.i - 1 in [2,4]:
                self.get_logger().info('Movement finished. Starting Realsense.')
                msg = Int32(data=44)
                self.get_logger().info(f'i: {self.i}')

                self.realsense_pub.publish(msg)

    def stop_scout_and_send_to_arm(self):
        self.get_logger().info('Stopping Scout and sending 20 to next_step1.')
        self.cmd_pub.publish(cmd)
        cmd = Twist()
        cmd.linear.x = 0.0
        
        
        msg = Int32(data=20)
        self.next_step_pub.publish(msg)
        
        # 다음 단계인 drink_code 13 신호를 기다림
        self.state = 'WAITING_FOR_13'

    def perform_final_movement(self):
        # drink_code 13 신호 수신 후 최종 동작
        
        # 구역 1
        if self.i == 1:
            self.get_logger().info(f'i={self.i}: Sending 30 (5x) and 40 (5x) to next_step1.')
            for _ in range(5):
                self.next_step_pub.publish(Int32(data=30))
                time.sleep(0.1)
            for _ in range(5):
                self.next_step_pub.publish(Int32(data=40))
                time.sleep(0.1)
        
        # 구역 2
        elif self.i == 2:
            self.get_logger().info(f'i={self.i}: Sending 40 (5x) to next_step1 and 55 to scout_lift (down).')
            for _ in range(5):
                self.next_step_pub.publish(Int32(data=40))
            self.lift_pub.publish(Int32(data=55))
        
        # 구역 3: 뒤로 1초 이동
        elif self.i == 3:
            self.get_logger().info(f'i={self.i}: Moving backward for 1 seconds and sending 55 to scout_lift (down).')
            self.start_move_timer(duration=1, speed=-0.15)
            self.lift_pub.publish(Int32(data=55))

        # 구역 4: 뒤로 1초 이동
        elif self.i == 4:
            self.get_logger().info(f'i={self.i}: Moving backward for 1 seconds and sending 30 (5x) to next_step1.')
            self.start_move_timer(duration=1, speed=-0.15)
            for _ in range(5):
                self.next_step_pub.publish(Int32(data=30))
                
        
        # 구역 5: 뒤로 2초 이동
        elif self.i == 5:
            self.get_logger().info(f'i={self.i}: Moving backward for 2 seconds and sending 30 (5x) to next_step1.')
            self.start_move_timer(duration=2, speed=-0.15)
            for _ in range(5):
                self.next_step_pub.publish(Int32(data=30))
                
        
        # 구역 6: 뒤로 2초 이동
        elif self.i == 6:
            self.get_logger().info(f'i={self.i}: Moving backward for 2 seconds and sending 55 to scout_lift (down).')
            self.start_move_timer(duration=2, speed=-0.15)
            self.lift_pub.publish(Int32(data=55))
        
        # 스캇 이동 완료 후 next_step1 토픽으로 40 5회 보냄
        # 모든 동작 완료 후 루프 종료
        self.get_logger().info('Final movement and actions are complete. Sending 40 (5x) to next_step1.')
        for _ in range(5):
            self.next_step_pub.publish(Int32(data=40))
            
        
        self.state = 'IDLE'
        self.scout_control_21_received = False
        self.get_logger().info('Loop A complete. Waiting for new commands (14 from drink_code).')


def main(args=None):
    rclpy.init(args=args)
    node = ScoutController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
