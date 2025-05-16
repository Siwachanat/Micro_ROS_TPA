#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, UInt16
from geometry_msgs.msg import Twist
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.publisher = self.create_publisher(UInt16, '/beep', 2)
        self.pub_Servo1 = self.create_publisher(Int32, "/servo_s1", 20)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)
        self.waypoints = []  # Now stores tuples: (x, y, z, w, is_delivery_point)
        self.current_index = 0
        self.send_servo_signal(0)
        self.human_warning = 0
        self.reached = 0

    def send_goal(self, x, y, z, w):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w

        self.get_logger().info(f'กำลังส่ง Goal: x={x}, y={y}, z={z}, w={w}')
        self.client.wait_for_server()
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal ถูกปฏิเสธ')
            return

        self.get_logger().info('Goal ถูกยอมรับ')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        x, y, z, w, is_delivery_point = self.waypoints[self.current_index]
        self.get_logger().info(f'Waypoint {self.current_index + 1} ถึงแล้ว')
        self.get_logger().info(f' ตรวจพบมนุษย์ = {self.human_warning}')
        self.reached = 1

        if is_delivery_point:
            self.send_servo_signal(-50)
            time.sleep(2)
            self.send_beep_signal(1)
            time.sleep(2)
            self.send_servo_signal(0)
            self.send_beep_signal(0)
        else:
            self.get_logger().info("Waypoint นี้เป็นแค่ทางผ่าน (no servo action)")

        self.waypoint_reached_callback()

    def waypoint_reached_callback(self):
        self.current_index += 1

        if self.current_index < len(self.waypoints):
            x, y, z, w, _ = self.waypoints[self.current_index]
            self.send_goal(x, y, z, w)
        else:
            self.get_logger().info("เดินทางครบทุก Waypoint แล้ว!")
            self.stop_robot()
            rclpy.shutdown()

    def stop_robot(self):
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info("หยุดหุ่นยนต์ก่อนทำการ shutdown")

    def send_beep_signal(self, value):
        beep_msg = UInt16()
        beep_msg.data = value
        self.publisher.publish(beep_msg)
        status = "เปิดเสียง (Beep)" if value == 1 else "ปิดเสียง (Beep)"
        self.get_logger().info(f'{status} ไปที่หัวข้อ /beep')

    def send_servo_signal(self, value):
        s1_init_angle = Int32()
        s1_init_angle.data = value
        self.pub_Servo1.publish(s1_init_angle)
        self.get_logger().info(f'{value} ไปที่หัวข้อ /servo1')

    def add_waypoint(self, x, y, z, w, is_delivery_point=True):
        self.waypoints.append((x, y, z, w, is_delivery_point))
        wp_type = "จุดส่งของ" if is_delivery_point else "ทางผ่าน"
        self.get_logger().info(f'เพิ่ม Waypoint: x={x}, y={y}, z={z}, w={w} ({wp_type})')

def main(args=None):
    rclpy.init(args=None)
    node = WaypointNavigator()

    # เพิ่ม Waypoints: (x, y, z, w, is_delivery_point)
    node.add_waypoint(0.952, 0.026, 0.267, 0.963, True)       # ส่งของ
    node.add_waypoint(0.400, 0.500, 0.0, 1.0, False)          # ทางผ่าน
    node.add_waypoint(0.764, -0.337, -0.246, 0.904, True)     # ส่งของ
    node.add_waypoint(-0.200, 0.400, 0.707, 0.707, False)     # ทางผ่าน
    node.add_waypoint(-0.018, -0.075, 0.522, 0.852, True)     # ส่งของ

    if node.waypoints:
        first_waypoint = node.waypoints[0]
        node.send_goal(*first_waypoint[:4])

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

