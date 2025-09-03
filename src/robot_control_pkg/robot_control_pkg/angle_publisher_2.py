import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from scipy.optimize import fsolve
import time

L1 = 9.5
L2 = 23.5
L3 = 25.0
L4 = 8.0

TH3 = 210.0

Mode = 1

def dh_parameter(theta, d, a, alpha):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    T = np.array([[ ct, -st*ca,  st*sa, a*ct],
                  [ st,  ct*ca, -ct*sa, a*st],
                  [  0,      sa,     ca,    d],
                  [  0,       0,      0,    1]])
    return T

def position(a1, a2, a3, a4):
    def rad_ch(a_n): return np.deg2rad(a_n - 180)

    Cam_array = np.array([
        [1, 0, 0, 5.0],
        [0, 1, 0, 5.0],
        [0, 0, 1, -3.0],
        [0, 0, 0, 1]
    ])
    
    theta = -15
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

def draw_circle(O,r):
    
    theta = np.empty((0, 3))

    for i in range(201):
        
        if i <= 100:
          x = (O[0]-r) + i*(2*r/100)
          y = np.sqrt(r**2 - (x - O[0])**2) + O[1]
        else:
          x = (O[0]+r) - (i-100)*(2*r/100)
          y = -np.sqrt(r**2 - (x - O[0])**2) + O[1]
        
        length = (np.sqrt(x**2 + y**2) - L3)/2
        
        th_z = np.arctan2(y,x)
        th_a = np.arccos(length/L2)
        
        th0 = (th_z / np.pi) * 180.0 + 150.0 # 식 찾기
        th1 = 150.0 - (th_a / np.pi)*180.0
        th2 = 150.0 + (th_a / np.pi)*180.0
        th_v = np.array([th0,th1,th2])
        theta = np.vstack([theta, th_v])

    return theta

def linear_trajectory(xi,yi,zi,xf,yf,zf):
    
    theta = np.empty((0, 3))
    
    xr = np.linspace(xi, xf, 100)
    yr = np.linspace(yi, yf, 100)
    zr = np.linspace(zi, zf, 100)
    
    global length_n, length_z
    
    for i in range(100):
        x = xr[i]
        y = yr[i]
        z = zr[i]
        
        length_n = (np.sqrt(x**2 + y**2) - L4)
        length_z = z - L1
        
        def equations(vars):
          theta1, theta2 = vars
          eq1 = (L2 * np.sin(theta1) + L3 * np.sin(theta1 + theta2)) - length_n
          eq2 = (L2 * np.cos(theta1) + L3 * np.cos(theta1 + theta2)) - length_z
          return [eq1, eq2]

        # 초기 추정값
        initial_guess = [0, 0]

        # 수치 해 찾기
        solution = fsolve(equations, initial_guess)
        theta1_sol, theta2_sol = solution

        theta_1 = np.degrees(theta1_sol) % 360
        theta_2 = np.degrees(theta2_sol) % 360

        if theta_1 > 180:
            theta_1 = theta_1 - 360
    
        if theta_2 > 180:
            theta_2 = theta_2 - 360
        
        th_z = np.arctan2(y,x)
        
        if theta_2 < 0:
            theta_1 = theta_1 + theta_2
            theta_2 = -theta_2
        
        th0 = (th_z / np.pi) * 180.0 + 180.0
        th1 = 180.0 - theta_1
        th2 = TH3 - theta_2
        
        th_v = np.array([th0,th1,th2])
        theta = np.vstack([theta, th_v])
        
    return theta

def polynomial_trajectory(xi,yi,zi,xf,yf,zf):
    
    theta = np.empty((0, 3))
    
    length_in = (np.sqrt(xi**2 + yi**2) - L4)
    length_iz = zi - L1
    
    def equations_i(vars):
          theta1, theta2 = vars
          eq1 = (L2 * np.sin(theta1) + L3 * np.sin(theta1 + theta2)) - length_in
          eq2 = (L2 * np.cos(theta1) + L3 * np.cos(theta1 + theta2)) - length_iz
          return [eq1, eq2]

    # 초기 추정값
    initial_guess = [0, 0]

    # 수치 해 찾기
    solution_i = fsolve(equations_i, initial_guess)
    theta1_sol, theta2_sol = solution_i
    
    theta_1 = np.degrees(theta1_sol) % 360
    theta_2 = np.degrees(theta2_sol) % 360

    if theta_1 > 180:
        theta_1 = theta_1 - 360
    
    if theta_2 > 180:
        theta_2 = theta_2 - 360
        
    if theta_2 < 0:
            theta_1 = theta_1 + theta_2
            theta_2 = -theta_2
        
    length_fn = (np.sqrt(xf**2 + yf**2) - L4)
    length_fz = zf - L1
    
    def equations_f(vars):
          theta3, theta4 = vars
          eq1 = (L2 * np.sin(theta3) + L3 * np.sin(theta3 + theta4)) - length_fn
          eq2 = (L2 * np.cos(theta3) + L3 * np.cos(theta3 + theta4)) - length_fz
          return [eq1, eq2]
      
    solution_f = fsolve(equations_f, initial_guess)
    theta3_sol, theta4_sol = solution_f
    
    theta_3 = np.degrees(theta3_sol) % 360
    theta_4 = np.degrees(theta4_sol) % 360
    
    if theta_3 > 180:
        theta_3 = theta_3 - 360
    
    if theta_4 > 180:
        theta_4 = theta_4 - 360
        
    if theta_4 < 0:
            theta_3 = theta_3 + theta_4
            theta_4 = -theta_4
        
    th_zi = np.arctan2(yi,xi)
    th_zf = np.arctan2(yf,xf)
    
    thi0 = (th_zi / np.pi) * 180.0 + 180.0
    thi1 = 180.0 - theta_1
    thi2 = TH3 - theta_2
    
    thf0 = (th_zf / np.pi) * 180.0 + 180.0
    thf1 = 180.0 - theta_3
    thf2 = TH3 - theta_4
    
    t = 20 # 루프 간격(현재 0.2s) X 각도의 개수 (현재 25회)
    
    def coef(i,f):
        
        Tf = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [1, t, t**2, t**3],
            [0, 1, 2*t, 3*(t**2)]
        ])
        Th = np.array([
            [i],
            [0],
            [f],
            [0]
        ])
        T_inv = np.linalg.inv(Tf)
        Coef = T_inv @ Th
        
        return Coef
    
    for i in range(401):
        Coef_0 = coef(thi0,thf0)
        Coef_1 = coef(thi1,thf1)
        Coef_2 = coef(thi2,thf2)
        
        th_0 = Coef_0[0,0] + Coef_0[1,0]*(0.05*i) + Coef_0[2,0]*(0.05*i)**2 + Coef_0[3,0]*(0.05*i)**3
        th_1 = Coef_1[0,0] + Coef_1[1,0]*(0.05*i) + Coef_1[2,0]*(0.05*i)**2 + Coef_1[3,0]*(0.05*i)**3
        th_2 = Coef_2[0,0] + Coef_2[1,0]*(0.05*i) + Coef_2[2,0]*(0.05*i)**2 + Coef_2[3,0]*(0.05*i)**3
        
        th_v = np.array([th_0,th_1,th_2])
        theta = np.vstack([theta, th_v])
        
    return theta

def heart_trajectory():
    def heart(time, period=10.0):
        """
        시간 기반 하트 궤적 함수
        :param time: 현재 시간 (초)
        :param period: 하트 한 바퀴 그리는 주기 (초)
        :return: (x, y)
        """
        # 0 ~ 2π 범위로 매핑
        theta = 2 * np.pi * (time % period) / period
        
        x = L4
        y = 16 * (np.sin(theta) ** 3)
        z = (13 * np.cos(theta) -
            5 * np.cos(2 * theta) -
            2 * np.cos(3 * theta) -
            np.cos(4 * theta))
        
        # 크기 축소 및 위치 이동 (로봇 작업 공간 맞추기)
        scale = 0.8  # 로봇 팔 길이에 맞춰서 조정 (m 단위라면 1cm 스케일)
        y = scale * y
        z = scale * z + 38  # y축 방향으로 살짝 올림 (예: 바닥에 닿지 않게)
        
        return x, y, z
    
    theta = np.empty((0, 3))
    
    for i in range(100):
        x, y, z = heart(0.1*i,10.0)
        
        length_n = (np.sqrt(x**2 + y**2) - L4)
        length_z = z - L1
        
        def equations(vars):
          theta1, theta2 = vars
          eq1 = (L2 * np.sin(theta1) + L3 * np.sin(theta1 + theta2)) - length_n
          eq2 = (L2 * np.cos(theta1) + L3 * np.cos(theta1 + theta2)) - length_z
          return [eq1, eq2]

        # 초기 추정값
        initial_guess = [0, 0]

        # 수치 해 찾기
        solution = fsolve(equations, initial_guess)
        theta1_sol, theta2_sol = solution

        theta_1 = np.degrees(theta1_sol) % 360
        theta_2 = np.degrees(theta2_sol) % 360

        if theta_1 > 180:
            theta_1 = theta_1 - 360
    
        if theta_2 > 180:
            theta_2 = theta_2 - 360
        
        th_z = np.arctan2(y,x)
        
        if theta_2 < 0:
            theta_1 = 2 * np.rad2deg(np.arctan2(length_n,length_z)) - theta_1
            theta_2 = -theta_2
        
        th0 = (th_z / np.pi) * 180.0 + 180.0
        th1 = 180.0 - theta_1
        th2 = TH3 - theta_2
        
        th_v = np.array([th0,th1,th2])
        theta = np.vstack([theta, th_v])
        
    return theta

def point_trajectory(x,y,z):
    
    theta = np.empty((0, 3))
    
    global length_n, length_z
        
    length_n = (np.sqrt(x**2 + y**2) - L4)
    length_z = z - L1
    
    def equations(vars):
        theta1, theta2 = vars
        eq1 = (L2 * np.sin(theta1) + L3 * np.sin(theta1 + theta2)) - length_n
        eq2 = (L2 * np.cos(theta1) + L3 * np.cos(theta1 + theta2)) - length_z
        return [eq1, eq2]

    # 초기 추정값
    initial_guess = [0, 0]

    # 수치 해 찾기
    solution = fsolve(equations, initial_guess)
    theta1_sol, theta2_sol = solution

    theta_1 = np.degrees(theta1_sol) % 360
    theta_2 = np.degrees(theta2_sol) % 360

    if theta_1 > 180:
        theta_1 = theta_1 - 360

    if theta_2 > 180:
        theta_2 = theta_2 - 360
    
    th_z = np.arctan2(y,x)
    
    if theta_2 < 0:
        theta_1 = 2 * np.rad2deg(np.arctan2(length_n,length_z)) - theta_1
        theta_2 = -theta_2
    
    th0 = (th_z / np.pi) * 180.0 + 180.0
    th1 = 180.0 - theta_1
    th2 = TH3 - theta_2

    theta = np.array([th0,th1,th2])
        
    return theta

class AnglePublisher2(Node):
    def __init__(self):
        super().__init__('angle_publisher_2')
        self.publisher = self.create_publisher(Float32MultiArray, '/Topic_target_angle', 10)
        self.publish_arr = self.create_publisher(Float32MultiArray, 'Topic_base_array', 10)
        self.sub_drink = self.create_subscription(Float32MultiArray, 'drink_position_base', self.drink_callback, 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/Topic_current_position',
            self.listener_callback,
            10)
        self.subscription
        self.i = 0
        self.n = 0
        self.timer = self.create_timer(0.05, self.publish_angle)
        self.timer_array = self.create_timer(0.05, self.publish_array)
        self.angle_1 = 150.0
        self.angle_2 = 150.0
        self.angle_3 = 150.0
        self.angle_4 = 150.0
        self.angle_5 = 110.0
        
        self.a_1 = None
        self.a_2 = None
        self.a_3 = None
        self.a_4 = None
        
        self.positions = None
        self.present_angle = None
        self.arr = None
        
        self.drink_pos = None

    def listener_callback(self, msg):
        self.positions = msg.data
        
        if self.i == 0:
             
            self.present_angle = self.positions
            self.angle_1 = self.present_angle[0]
            self.angle_2 = self.present_angle[1]
            self.angle_3 = self.present_angle[2]
            self.angle_4 = self.present_angle[3]
            
            if self.positions is not None:
                self.i = 1
                
        if self.i == 3:
            a1, a2, a3, a4, a5 = self.positions
            arr = position(a1, a2, a3, a4)
            self.arr = np.round(arr, 2).flatten().tolist()
            self.i = 5 # If regular loop wanted, must change into self.i = 4
            
    def publish_array(self):
        if self.i == 4:
        
            msg = Float32MultiArray()
            msg.data = self.arr
            self.get_logger().info(f'변환 행렬: {msg.data}')
            self.publish_arr.publish(msg)
            
    def drink_callback(self, msg):
        if self.i == 4 and len(msg.data) == 3 and msg.data[2] > -20:
            self.get_logger().info(f"음료 좌표: {msg.data}")
            self.i = 5
            self.drink_pos = msg.data
            
    
    def publish_angle(self):
        msg = Float32MultiArray()
        angles = [self.angle_1, self.angle_2, self.angle_3, self.angle_4, self.angle_5]
        msg.data = [float(a) for a in angles]
        
        self.get_logger().info(f'모터 1로 송출한 각도: {msg.data[0]}')
        self.get_logger().info(f'모터 2로 송출한 각도: {msg.data[1]}')
        self.get_logger().info(f'모터 3로 송출한 각도: {msg.data[2]}')
        self.get_logger().info(f'모터 4로 송출한 각도: {msg.data[3]}') 
        
        g_lift = 0 # 머리가 어디 있는가에 따라 바뀌도록 설정 (8.5)
        
        ws_tilt = 30
        
        n_y = 0.6
        
        if self.i == 1:
            
            self.publisher.publish(msg)
            
            self.angle_1 = 180
            self.angle_2 = 180 + g_lift
            self.angle_3 = TH3
            self.angle_4 = 90
            self.angle_5 = 30
            
            time.sleep(0.1)
            
            if self.n == 100:
                self.i = 2
                self.n = 0
                
            self.n = self.n + 1

        elif self.i == 2:
            self.publisher.publish(msg)
            
            a1, a2, a3 = point_trajectory(L4, 0, 42.0)
            
            self.angle_1 = a1
            self.angle_2 = a2 + g_lift
            self.angle_3 = a3
            self.angle_4 = 180.0 - (90 - ((180-a2) + (TH3-a3)))
            
            time.sleep(0.1)
            
            if self.n == 50:
                self.i = 100
                self.n = 0
                
            self.n = self.n + 1

        elif self.i == 100:
            self.publisher.publish(msg)
            
            course = heart_trajectory()
            
            self.angle_1 = course[self.n,0]
            self.angle_2 = course[self.n,1]
            self.angle_3 = course[self.n,2]
            self.angle_4 = 180.0 - (90 - ((180-course[self.n,1]) + (TH3-course[self.n,2])))
            
            time.sleep(0.1)
            
            if self.n == 99:
                self.i = 101
                self.n = 0
                
            self.n = self.n + 1

        elif self.i == 101:
            self.publisher.publish(msg)
            
            course = point_trajectory(L4,0,22.2)
            
            self.angle_1 = course[0]
            self.angle_2 = course[1]
            self.angle_3 = course[2]
            self.angle_4 = 180.0 - (90 - ((180-course[1]) + (TH3-course[2])))
            
            time.sleep(0.1)
            
            if self.n == 101:
                self.i = 4
                self.n = 0
                
            self.n = self.n + 1
            
        '''elif self.i == 2:
            self.publisher.publish(msg)
            
            a1, a2, a3 = point_trajectory(L4, 0, 25.0)
            
            self.angle_1 = a1
            self.angle_2 = a2 + g_lift
            self.angle_3 = a3
            self.angle_4 = 180.0 - (90 - ((180-a2) + (TH3-a3))) - ws_tilt
            
            time.sleep(0.1)
            
            if self.n == 150:
                self.i = 3
                self.n = 0
                
            self.n = self.n + 1
            
        elif self.i == 5:
            self.publisher.publish(msg)
            
            x, y, z = 20,20,10 # If regular loop wanted, must change into self.drink_pos
            
            a1, a2, a3 = point_trajectory(x , n_y * y, z + 10)
            
            self.angle_1 = a1
            self.angle_2 = a2 + g_lift
            self.angle_3 = a3
            self.angle_4 = 180.0 - (90 - ((180-a2) + (TH3-a3)))
            
            self.get_logger().info(f'음료 좌표: {self.drink_pos}') 
            
            time.sleep(0.1)
            
            if self.n == 100:
                self.i = 6
                self.n = 0
                
            self.n = self.n + 1
            
        elif self.i == 6:
            self.publisher.publish(msg)
            
            x, y, z = 20,20,10
            
            a1, a2, a3 = point_trajectory(x , n_y * y, z + 10)
            
            self.angle_1 = a1
            self.angle_2 = a2 + g_lift
            self.angle_3 = a3
            self.angle_4 = 180.0 - (90 - ((180-a2) + (TH3-a3)))
            
            time.sleep(0.1)
            
            if self.n == 100:
                self.angle_5 = 110
                
            elif self.n == 200:
                self.i = 7
                self.n = 0
                
            self.n = self.n + 1
            
        elif self.i == 7:
            self.publisher.publish(msg)
            
            x, y, z = 20,20,10
            
            a1, a2, a3 = point_trajectory(x , n_y * y, z + 20)
            
            self.angle_1 = a1
            self.angle_2 = a2 + g_lift
            self.angle_3 = a3
            self.angle_4 = 180.0 - (90 - ((180-a2) + (TH3-a3)))
            
            time.sleep(0.1)
            
            if self.n == 50:
                self.i = 8
                self.n = 0
                
            self.n = self.n + 1
            
        elif self.i == 8:
            self.publisher.publish(msg)
            
            #x, y, z = self.drink_pos
            
            a1, a2, a3 = point_trajectory(L4, 0, 25.0)
            
            self.angle_1 = a1
            self.angle_2 = a2 + g_lift
            self.angle_3 = a3
            self.angle_4 = 180.0 - (90 - ((180-a2) + (TH3-a3)))
            
            time.sleep(0.1)
            
            if self.n == 100:
                self.i = 9
                self.n = 0
                
            self.n = self.n + 1
        '''            
        
            
def main(args=None):
    rclpy.init(args=args)
    node = AnglePublisher2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  