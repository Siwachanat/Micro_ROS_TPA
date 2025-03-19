import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("🚀 CmdVel Publisher Started...")

    def send_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x   # ความเร็วเชิงเส้น (m/s)
        msg.angular.z = angular_z  # ความเร็วเชิงมุม (rad/s)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent cmd_vel: Linear={linear_x} m/s, Angular={angular_z} rad/s")

def main():
    rclpy.init()
    node = CmdVelPublisher()

    # ขับไปข้างหน้าด้วยความเร็ว 0.1 m/s และหมุน 0.5 rad/s เป็นเวลา 10 วินาที
    for _ in range(10):
        node.send_cmd_vel(0.0, 0.5)  
        time.sleep(1)  

    # หยุดการเคลื่อนที่
    node.send_cmd_vel(0.0, 0.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
