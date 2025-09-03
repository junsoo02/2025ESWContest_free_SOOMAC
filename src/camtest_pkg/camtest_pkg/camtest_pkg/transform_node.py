import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class CoordinateTransformNode(Node):
    def __init__(self):
        super().__init__('coordinate_transform_node')

        # 가장 최근 데이터 저장
        self.latest_position_cam = None  # [x, y, z]
        self.T_base_from_cam = None      # 4x4 변환 행렬

        # 구독자 생성
        self.create_subscription(Float32MultiArray, 'drink_position', self.position_callback, 10)
        self.create_subscription(Float32MultiArray, 'Topic_base_array', self.transform_callback, 10)

        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Float32MultiArray, 'drink_position_base', 10)

        self.get_logger().info("Coordinate Transform Node using Frame Muiplication started.")

    def position_callback(self, msg):
        self.latest_position_cam = msg.data 
        self.try_transform()

    def transform_callback(self, msg):
        if len(msg.data) != 16:
            self.get_logger().warn("Transform matrix must have 16 elements.")
            return
        self.T_base_from_cam = np.array(msg.data).reshape((4, 4))
        self.try_transform()

    def try_transform(self):
        if self.latest_position_cam is None or self.T_base_from_cam is None:
            return

        x, y, z = self.latest_position_cam

        # 1️⃣ 객체 위치를 표현하는 4x4 프레임 행렬 생성
        T_obj_from_cam = np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])

        # 2️⃣ 카메라 프레임 기준 객체 프레임 → 베이스 프레임 기준으로 변환
        T_obj_from_base = np.dot(self.T_base_from_cam, T_obj_from_cam)

        # ✅ 여기 추가하면 됨
        self.get_logger().info(f"T_base_from_cam:\n{self.T_base_from_cam}")
        self.get_logger().info(f"T_obj_from_cam:\n{T_obj_from_cam}")
        self.get_logger().info(f"T_obj_from_base:\n{T_obj_from_base}")

        # 3️⃣ 변환된 객체 위치 추출 (맨 마지막 열의 [x, y, z])
        x_base, y_base, z_base = T_obj_from_base[0:3, 3]

        # 4️⃣ 퍼블리시
        msg = Float32MultiArray()
        msg.data = [x_base, y_base, z_base]
        self.publisher_.publish(msg)

        self.get_logger().info(
            f"[FRAME × FRAME] Transformed: X={x_base:.2f}cm, Y={y_base:.2f}cm, Z={z_base:.2f}cm"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
