import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
import time

class BeepPublisher(Node):
    def __init__(self):
        super().__init__('beep_publisher')
        self.publisher = self.create_publisher(UInt16, '/beep', 10)
        self.get_logger().info("Beep Publisher Started...")

    def send_beep_signal(self, value):
        """ ส่งเสียง Beep หรือปิดเสียงไปยังหัวข้อ /beep """
        beep_msg = UInt16()
        beep_msg.data = value  # 1 = เปิดเสียง, 0 = ปิดเสียง
        self.publisher.publish(beep_msg)
        status = "เปิดเสียง (Beep)"
        if value == 0 :
            status = "ปิดเสียง (Beep)"
        self.get_logger().info(f'{status} ไปที่หัวข้อ /beep')

def main():
    rclpy.init()
    node = BeepPublisher()
    while rclpy.ok():
        node.send_beep_signal(1)
        time.sleep(2)  # ส่ง Beep ทุก 2 วินาที
        node.send_beep_signal(0)
        time.sleep(2)  # ส่ง Beep ทุก 2 วินาที

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
