import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class UltrasonicSerialNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_serial_node')
        self.publisher_ = self.create_publisher(Int32, 'ultrasonic_sensor', 10)
        
        # Arduino가 연결된 시리얼 포트
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        
        self.timer = self.create_timer(0.1, self.read_serial)  # 10Hz

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line.isdigit():  # 센서 번호만 publish
                msg = Int32()
                msg.data = int(line)
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published sensor: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

