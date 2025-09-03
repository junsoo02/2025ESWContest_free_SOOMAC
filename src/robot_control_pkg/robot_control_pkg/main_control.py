import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, Int32, Bool

# 링크 길이(mm)
L1, L2, L3, L4 = 9.5, 23.5, 25.0, 7.0

TH3 = 210.0


# ---------------------- 로봇 기구학 함수 ----------------------

def dh_parameter(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])


def position(a1, a2, a3, a4):
    def rad_ch(a_n): return np.deg2rad(a_n - 180)

    Cam_array = np.array([
        [1, 0, 0, 5.0],
        [0, 1, 0, 6.2],
        [0, 0, 1, -5.0],
        [0, 0, 0, 1]
    ])
    
    theta = 1.5
    th = np.deg2rad(theta)
    
    Cam_tilt = np.array([
        [np.cos(th), -np.sin(th), 0, 0],
        [np.sin(th), np.cos(th), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T1 = dh_parameter(rad_ch(a1), L1, 0, np.pi / 2)
    T2 = dh_parameter(rad_ch(a2) + np.pi / 2, 0, L2, 0)
    T3 = dh_parameter(np.deg2rad(a3 - TH3), 0, L3, 0)
    T4 = dh_parameter(rad_ch(a4), 0, 0, 0) @ Cam_array @ Cam_tilt
    return T1 @ T2 @ T3 @ T4


# ---------------------- ROS 노드 클래스 ----------------------

class MainControl(Node):
    def __init__(self):
        super().__init__('main_control')
        self.publish_arr = self.create_publisher(Float32MultiArray, 'Topic_base_array', 10)
        self.pub_drink = self.create_publisher(Int32, 'drink_code', 10)
        self.pub_stop = self.create_publisher(Bool, 'stop_process', 10)
        self.pub_drink_pos = self.create_publisher(Float32MultiArray, 'drink_pos', 10)
        self.subscription = self.create_subscription(Float32MultiArray, '/Topic_current_position', self.listener_callback, 10)
        self.current_subscription = self.create_subscription(Float32, '/Topic_current', self.current_callback, 10)
        self.sub_drink = self.create_subscription(Float32MultiArray, 'drink_position_base', self.drink_callback, 10)
        self.sub_scout_lift = self.create_subscription(Int32, 'next_step1', self.callback_scout_lift, 10)
        self.drink_sub = self.create_subscription(Int32, 'ultrasonic_sensor', self.callback_drink, 10)
        self.angle_sign = self.create_subscription(Int32, 'angle_sign', self.callback_sign, 10)


        self.timer_drink = self.create_timer(0.5, self.publish_drink)
        self.timer_array = self.create_timer(0.05, self.publish_array)
        self.timer_drink = self.create_timer(0.5, self.publish_drink)
        self.timer_array = self.create_timer(0.05, self.publish_array)
        self.timer_pos = self.create_timer(0.05, self.drink_pos_pub)
        self.timer_drop = self.create_timer(0.2, self.publish_drop)
        # add new publish timers

        self.i = 0
        self.ia = 0
        self.positions = None
        self.drink = None
        self.drink_pos = None
        self.code_sl = None

        self.lift = False
        self.scout = False
        self.drop = False


    def listener_callback(self, msg):
        self.positions = msg.data

    def current_callback(self, msg):
        current = msg.data
        if self.ia >= 8 and self.ia < 12:
            if current < 100:
                self.drop = True
            else:
                self.drop = False

    def publish_drop(self):
        msg = Bool()
        if self.ia >= 8 and self.ia < 12:
            msg.data = self.drop
            self.pub_stop.publish(msg)
            if msg.data == True:
                self.get_logger().info(f'Drink dropped')

    def callback_drink(self, msg):
        if self.drink is None and msg.data != 0 and self.i == 0:
            self.drink = msg.data - 1
            self.get_logger().info(f'Drink detected: {self.drink}')
            
            if self.drink is not None:
                self.i = 1
            
    def publish_drink(self):
        msg = Int32()
        self.get_logger().info(f'self.i: {self.i}')
        
        if self.i == 1:
            msg.data = self.drink
            self.pub_drink.publish(msg)
            
        if self.i == 2:
            msg.data = 11
            self.pub_drink.publish(msg)
            self.get_logger().info(f'code sent: {msg.data}')
            
        if self.i == 5:
            msg.data = 12
            self.pub_drink.publish(msg)
            self.get_logger().info(f'code sent: {msg.data}')

        if self.i == 6:
            msg.data = 13
            self.pub_drink.publish(msg)
            self.get_logger().info(f'code sent: {msg.data}')

        if self.i == 7:
            msg.data = 14
            self.pub_drink.publish(msg)
            self.get_logger().info(f'code sent: {msg.data}')
            
    #callback_scout_lift - (self.i == 2일 때 실행) 두 publisher에서 신호를 받으면 둘 다 받았다고 인식 후 self.i = 3으로 변경
    def callback_scout_lift(self, msg):
        if self.i == 2:
            self.code_sl = msg.data
            
            if self.code_sl == 10:
                self.lift = True
            elif self.code_sl == 20:
                self.scout = True
            self.get_logger().info(f'S_L sent: {self.code_sl}')

            if self.lift == True and self.scout == True:
                self.i = 3
                self.get_logger().info('Lift and Scout Done')
                self.lift = False
                self.scout = False
                self.code_sl = 0

        if self.i == 6:
            self.code_sl = msg.data

            if self.code_sl == 30:
                self.lift = True
            elif self.code_sl == 40:
                self.scout = True

            if self.lift == True and self.scout == True:
                self.i = 7
                self.lift = False
                self.scout = False
                self.code_sl = 0

                    
    def callback_sign(self, msg):
        self.ia = msg.data
        
        if self.i == 1 and self.ia == 4:
            self.i = 2

        if self.ia >= 8 and self.ia < 12:
            self.get_logger().info(f"~~~로봇이 음료를 옮기고 있습니다~~~")

        if self.i == 5 and self.ia == 11:
            self.i = 6

        if self.i == 7 and self.ia == 15:
            self.i = 0
            self.ia = 0
            
            self.positions = None
            self.drink = None
            self.drink_pos = None
            self.code_sl = None

            self.lift = False
            self.scout = False
            self.drop = False
        

    def publish_array(self):
        if self.positions is None:
            return
        if self.i == 3:
            
            a1, a2, a3, a4, a5 = self.positions
            pos = position(a1, a2, a3, a4)
            self.get_logger().info(f"Array: {pos}")
            msg = Float32MultiArray()
            msg.data = np.round(pos, 2).flatten().tolist()
            self.publish_arr.publish(msg)

            if msg.data is not None:
                self.i = 4

    def drink_callback(self, msg):
        if len(msg.data) == 3 and self.i == 4:
            self.get_logger().info(f"음료 좌표: {msg.data}")
            self.drink_pos = msg.data
            self.i = 5 # self.i == 5일 때 좌표를 angle_publisher로 송신

    def drink_pos_pub(self):
        if self.i == 5:
            msg = Float32MultiArray()
            msg.data = self.drink_pos
            self.pub_drink_pos.publish(msg)

# ---------------------- 메인 함수 ----------------------

def main(args=None):
    rclpy.init(args=args)
    node = MainControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()