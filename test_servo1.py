import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class ServoPublisher(Node):
    def __init__(self):
        super().__init__('Servol_publisher')
        self.pub_Servo1 = self.create_publisher(Int32, "/servo_s1", 20)
        self.get_logger().info("Servol Publisher Started...")

    def send_servo_signal(self, value):
        """ ส่งสัญญาณ หรือเปิดเงื่อนไขไปที่หัวข้อ /servo1 """
        s1_init_angle = Int32()
        s1_init_angle.data = value  # 1 = เปิด, 0 = ปิด
        self.pub_Servo1.publish(s1_init_angle)
        self.get_logger().info(f"ส่งค่า {value} ไปที่หัวข้อ /servo1")


def main():
    rclpy.init()
    node = ServoPublisher()
    
    while rclpy.ok():
        node.send_servo_signal(0)
        time.sleep(2)  # ส่งค่า 0 แล้วพัก 2 วินาที
        node.send_servo_signal(-50)
        time.sleep(2)  # ส่งค่า -50 แล้วพัก 2 วินาที

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
