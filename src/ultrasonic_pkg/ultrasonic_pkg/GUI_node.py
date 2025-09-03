import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, String, Bool

class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')

        # 퍼블리시: IN/OUT 버튼, 재고 초기화
        self.in_out_pub = self.create_publisher(String, '/in_out', 10)
        self.stock_init_pub = self.create_publisher(Int32MultiArray, '/stock_init', 10)

        # 구독: 메인 테스크 퍼블리시
        self.drink_code_sub = self.create_subscription(Int32, '/drink_code', self.drink_code_callback, 10)
        self.stop_process_sub = self.create_subscription(Bool, '/stop_process', self.stop_process_callback, 10)
        self.inventory_sub = self.create_subscription(Int32MultiArray, '/inventory', self.inventory_callback, 10)

    def drink_code_callback(self, msg):
        code = msg.data
        if 0 <= code <= 5:
            print(f"drink_code 수신 → 수행 슬롯 인덱스: {code}")
        elif code == 14:
            print("drink_code 수신 → 테스크 사이클 종료, 로봇 정지")

    def stop_process_callback(self, msg):
        if msg.data:
            print("ALERT: 음료수를 떨어뜨렸습니다!")

    def inventory_callback(self, msg):
        print(f"현재 재고 상태: {msg.data}")

    # GUI 버튼 이벤트
    def press_in_button(self):
        self.in_out_pub.publish(String(data="IN"))

    def press_out_button(self):
        self.in_out_pub.publish(String(data="OUT"))

    def initialize_stock(self, stock_list):
        msg = Int32MultiArray()
        msg.data = stock_list
        self.stock_init_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

