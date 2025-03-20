#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool,UInt16
from geometry_msgs.msg import Twist  # เพิ่มการควบคุม cmd_vel
from yahboomcar_msgs.msg import PointArray
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import math
import subprocess
      
class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.publisher = self.create_publisher(UInt16, '/beep', 2)  # Publisher สำหรับเสียง
        self.pub_Servo1 = self.create_publisher(Int32,"/servo_s1" , 20) # Publisher Servo1
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)  # Publisher สำหรับหยุดหุ่นยนต์
        self.pub_pose = self.create_subscription(PointArray,"/mediapipe/points",self.poseDetector_callback,3)
        #self.pub_pose = None 
        self.waypoints = []  # รายการ Waypoints
        self.current_index = 0
        self.send_servo_signal(0)
        self.human_warning = 0 # ตรวจสอบการตรวจพบมนุษย์
        self.reached = 0 # ตรวจสอบการถึงพิกัด
        self.loop_poseDetector = 0 # นับรอบการตรวจสอบมนุษย์
        
    def send_goal(self, x, y,z,w):
        """ ส่ง Goal Pose ไปยัง Nav2 """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # คำนวณ quaternion จาก yaw (หมุนกลับ)
        #q_x, q_y, q_z, q_w = self.quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w

        self.get_logger().info(f'กำลังส่ง Goal: x={x}, y={y}, z={z},w={w}')
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
        """ เมื่อถึง Waypoint จะส่งเสียง Beep และปิดหลังจากนั้น """
        self.get_logger().info(f'Waypoint {self.current_index + 1} ถึงแล้ว')
        self.get_logger().info(f' ตรวจพบมนุษย์ = {self.human_warning}')
        self.reached = 1 # ถึงจุดหมายแล้ว

    def waypoint_reached_callback(self):
        """ ไปยัง Waypoint ถัดไปหรือสิ้นสุดการทำงาน """
        self.current_index += 1

        if self.current_index < len(self.waypoints):
            x, y, z, w = self.waypoints[self.current_index]
            self.send_goal(x, y, z, w)
        else:
            self.get_logger().info("เดินทางครบทุก Waypoint แล้้ว!")
            self.stop_robot()  # หยุดหุ่นยนต์
            rclpy.shutdown()

    def stop_robot(self):
        """ หยุดหุ่นยนต์โดยการส่งคำสั่งให้มีความเร็วเป็น 0 """
        stop_msg = Twist()  # สร้างข้อความหยุดหุ่นยนต์
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info("หยุดหุ่นยนต์ก่อนทำการ shutdown")

    def send_beep_signal(self, value):
        """ ส่งเสียง Beep หรือปิดเสียงไปยังหัวข้อ /beep """
        beep_msg = UInt16()
        beep_msg.data = value  # 1 = เปิดเสียง, 0 = ปิดเสียง
        self.publisher.publish(beep_msg)
        status = "เปิดเสียง (Beep)" if value == 1 else "ปิดเสียง (Beep)"
        self.get_logger().info(f'{status} ไปที่หัวข้อ /beep')
    def poseDetector_callback(self,msg):
        
        if (self.reached == 1):
            self.loop_poseDetector += 1 
            """ ตรวจสอบมนุษย์ """
            self.get_logger().info(f' ตรวจพบมนุษย์ = {self.human_warning}')
            if len(msg.points) > 0 :
                self.human_warning += 1
            else:
                self.human_warning -= 2
            if self.human_warning < 0 :
                self.human_warning = 0
            if self.human_warning > 10 :
                self.human_warning = 10
        
        if(self.loop_poseDetector > 10): #ครบจำนวน 10 รอบ
            self.loop_poseDetector = 0
            self.reached = 0
            if(self.human_warning > 2):
                self.send_servo_signal(-50)
                time.sleep(2)  # ปล่อยลูกบาศเป็นเวลา 2 วินาที  
                self.send_beep_signal(1)
                time.sleep(2)  # เปิดเสียงเป็นเวลา 2 วินาที
            else:
                self.send_beep_signal(1)
                time.sleep(2)  # เปิดเสียงเป็นเวลา 2 วินาที
            
            self.send_servo_signal(0) # เตรียมพร้อม
            self.send_beep_signal(0)  # ปิดเสียง 
      
            # ไปยัง Waypoint ถัดไปหรือหยุดเมื่อถึง Waypoint สุดท้าย
            self.waypoint_reached_callback()
        
            
    def send_servo_signal(self, value):
        """ ส่งเสียง servo หรือปิดเสียงไปยังหัวข้อ /servo1 """
        s1_init_angle = Int32()
        s1_init_angle.data = value  # 1 = เปิดเสียง, 0 = ปิดเสียง
        #for i in range(4):
        self.pub_Servo1.publish(s1_init_angle)
        self.get_logger().info(f'{value} ไปที่หัวข้อ /servo1')

    def add_waypoint(self, x, y, z, w):
        """ เพิ่ม Waypoint ใหม่ """
        self.waypoints.append((x, y, z, w))
        self.get_logger().info(f'เพิ่ม Waypoint: x={x}, y={y}, z={z},w={w}')

def main(args=None):
    rclpy.init(args=None)
    node = WaypointNavigator()
    
    # เพิ่ม Waypoints (x,y,oz,ow)
    node.add_waypoint(0.952, 0.026, 0.267, 0.963)
    node.add_waypoint(0.764, -0.337, -0.246, 0.904)
    node.add_waypoint(-0.018, -0.075, 0.522, 0.852)

    # เริ่มการเดินทางไปยัง Waypoint แรก
    if node.waypoints:
        first_waypoint = node.waypoints[0]
        node.send_goal(*first_waypoint)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

