import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray, Int32, Bool
from scipy.optimize import fsolve

# -------------------------
# 로봇 팔 파라미터
# -------------------------
L1, L2, L3, L4 = 9.5, 23.5, 25.0, 12.0
TH3 = 210.0


# -------------------------
# 수학적 유틸 함수
# -------------------------
def dh_parameter(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct],
        [ st,  ct*ca, -ct*sa, a*st],
        [  0,     sa,     ca,    d],
        [  0,      0,      0,    1]
    ])


def position(a1, a2, a3, a4):
    def rad_ch(a_n): return np.deg2rad(a_n - 180)

    Cam_array = np.array([
        [1, 0, 0, 5.0],
        [0, 1, 0, 6.2],
        [0, 0, 1, 0.0],
        [0, 0, 0, 1]
    ])

    theta = 7
    th = np.deg2rad(theta)
    Cam_tilt = np.array([
        [np.cos(th), -np.sin(th), 0, 0],
        [np.sin(th),  np.cos(th), 0, 0],
        [0,           0,          1, 0],
        [0,           0,          0, 1]
    ])

    T1 = dh_parameter(rad_ch(a1), L1, 0, np.pi / 2)
    T2 = dh_parameter(rad_ch(a2) + np.pi / 2, 0, L2, 0)
    T3 = dh_parameter(np.deg2rad(a3 - TH3), 0, L3, 0)
    T4 = dh_parameter(rad_ch(a4), 0, 0, 0) @ Cam_array @ Cam_tilt
    return T1 @ T2 @ T3 @ T4


def point_trajectory(x, y, z):
    global length_n, length_z
    length_n = np.sqrt(x**2 + y**2) - L4
    length_z = z - L1

    def equations(vars):
        t1, t2 = vars
        eq1 = (L2*np.sin(t1) + L3*np.sin(t1+t2)) - length_n
        eq2 = (L2*np.cos(t1) + L3*np.cos(t1+t2)) - length_z
        return [eq1, eq2]

    sol = fsolve(equations, [0, 0])
    theta1_sol, theta2_sol = sol

    t1, t2 = np.degrees(theta1_sol), np.degrees(theta2_sol)
    t1 = (t1 % 360) - 360 if t1 > 180 else t1
    t2 = (t2 % 360) - 360 if t2 > 180 else t2

    th_z = np.arctan2(y, x)
    if t2 < 0:
        t1 = 2*np.rad2deg(np.arctan2(length_n, length_z)) - t1
        t2 = -t2

    return np.array([
        (th_z/np.pi)*180.0 + 180.0,   # th0
        180.0 - t1,                   # th1
        TH3 - t2                      # th2
    ])


# -------------------------
# ROS2 Node
# -------------------------
class AnglePublisher(Node):
    def __init__(self):
        super().__init__('angle_publisher')

        # Publisher & Subscriber
        self.publisher = self.create_publisher(Float32MultiArray, '/Topic_target_angle', 10)
        self.publisher_sign = self.create_publisher(Int32, 'angle_sign', 10)
        self.sub_drink = self.create_subscription(Float32MultiArray, 'drink_pos', self.drink_callback, 10)
        self.sub_positions = self.create_subscription(Float32MultiArray, '/Topic_current_position', self.listener_callback, 10)
        self.sub_drink_code = self.create_subscription(Int32, 'drink_code', self.drink_code_callback, 10)
        self.lift_positions = self.create_subscription(Bool, 'lift_position', self.lift_callback, 10)

        # State
        self.i, self.n = 0, 0
        self.timer_angle = self.create_timer(0.05, self.publish_angle)
        self.timer_sign = self.create_timer(0.05, self.publish_sign)

        # Arm Angles
        self.angle_1, self.angle_2, self.angle_3 = 180.0, 180.0, TH3
        self.angle_4, self.angle_5 = 90.0, 0.0

        # Buffers
        self.positions = None
        self.present_angle = None
        self.drink = None
        self.drink_pos = None

        self.lift = False

    # -------------------------
    # 콜백들
    # -------------------------
    def drink_code_callback(self, msg):
        if self.i == 0 and msg.data is not None:
            self.drink = msg.data
            self.get_logger().info(f"수신한 음료: {self.drink}")
            self.i = 1
        elif self.i == 4 and msg.data == 12:
            self.i = 5
        elif self.i == 11 and msg.data == 14:
            self.i = 12

        self.get_logger().info(f"self.i: {self.i}")

    def lift_callback(self, msg):
        self.lift = msg.data

    def listener_callback(self, msg):
        if self.i in (1, 2):
            self.positions = msg.data
            self.i = 2
            if self.positions is not None:
                self.present_angle = self.positions
                self.positions = None
                self.angle_1, self.angle_2, self.angle_3, self.angle_4 = self.present_angle[:4]
                self.i = 3  # 테스트 시 self.i = 4

    def drink_callback(self, msg):
        if self.i == 5 and len(msg.data) == 3 and msg.data[2] > -20:
            self.get_logger().info(f"음료 좌표: {msg.data}")
            self.drink_pos = msg.data
            self.i = 6

    def publish_sign(self):
        msg = Int32()
        msg.data = self.i
        if self.i in (4, 8, 11, 12, 15):
            self.publisher_sign.publish(msg)
        if self.i == 15:  # 초기화
            self.positions = None
            self.present_angle = None
            self.drink = None
            self.drink_pos = None
            self.i = 0

    # -------------------------
    # 공통 동작 함수
    # -------------------------
    def move_to_position(self, msg, x, y, z,
                         g_lift=0, ws_tilt=0,
                         update_angle5=False, angle5_value=0,
                         next_i=None, duration=100):
        """공통된 로봇 팔 이동 동작"""
        self.publisher.publish(msg)
        a1, a2, a3 = point_trajectory(x, y, z)

        self.angle_1 = a1
        self.angle_2 = a2 + g_lift
        self.angle_3 = a3
        self.angle_4 = 180.0 - (90 - ((180 - a2) + (TH3 - a3))) - ws_tilt

        if update_angle5:
            self.angle_5 = angle5_value

        if self.n >= duration:
            if next_i is not None:
                self.i = next_i
            self.n = 0
        else:
            self.n += 1

    # -------------------------
    # 주 타이머 콜백
    # -------------------------
    def publish_angle(self):
        if self.i in (0, 1, 2, 5, 11, 15):
            return

        msg = Float32MultiArray()
        msg.data = [float(self.angle_1),
            float(self.angle_2),
            float(self.angle_3),
            float(self.angle_4),
            float(self.angle_5)]

        n_y, g_lift, ws_tilt = 0.8, 0, 0

        if self.i == 3:
            self.move_to_position(msg, 0, L4, 22.2, g_lift, ws_tilt = 30,
                                  update_angle5=True, next_i=4)
        elif self.i == 4:
            a1, a2, a3 = point_trajectory(0, L4, 22.2)
            if self.lift == False:
                self.angle_4 = 180.0 - (90 - ((180 - a2) + (TH3 - a3))) - 30
                self.publisher.publish(msg)
            if self.lift == True:
                self.angle_4 = 180.0 - (90 - ((180 - a2) + (TH3 - a3)))
                self.publisher.publish(msg)    
        elif self.i == 6:
            x, y, z = self.drink_pos
            self.move_to_position(msg, n_y*x, y-5, z+4, g_lift = 7 , ws_tilt = 0, next_i=7, duration=50)
        elif self.i == 7:
            x, y, z = self.drink_pos
            self.move_to_position(msg, n_y*x, y+3, z+4,
                                  g_lift = 7, ws_tilt = 0,
                                  update_angle5=(self.n == 50),
                                  angle5_value=140,
                                  next_i=8, duration=100)
        elif self.i == 8:
            x, y, z = self.drink_pos
            self.move_to_position(msg, n_y*x, y-7, z+10, g_lift = 7, ws_tilt = 0, next_i=9)
        elif self.i == 9:
            self.move_to_position(msg, 0, L4, 22.2, g_lift, ws_tilt, next_i=10)
        elif self.i == 10:
            self.move_to_position(msg, 0, -L4, 22.2, g_lift, ws_tilt, next_i=11)
        elif self.i == 12:
            self.lift = False
            self.move_to_position(msg, -self.drink*7 + 17.5, -32, 25.0,
                                  g_lift, ws_tilt = 20,
                                  update_angle5=(self.n == 100),
                                  angle5_value=0,
                                  next_i=13, duration=200)
                                  
        elif self.i == 13:
            self.move_to_position(msg, 0, -L4, 22.2, g_lift, ws_tilt, next_i=14)
        elif self.i == 14:
            self.move_to_position(msg, 0, L4, 22.2, g_lift, ws_tilt, next_i=15)


# -------------------------
# 메인
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = AnglePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
