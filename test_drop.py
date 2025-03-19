import rclpy
from rclpy.node import Node
from yahboomcar_msgs.msg import PointArray
from std_msgs.msg import Int32
import time
class dropToHumanSubscriber(Node):
    def __init__(self):
        super().__init__('human_subscriber')
        self.pub_pose = self.create_subscription(PointArray,"/mediapipe/points",self.poseDetector_callback,3)
        self.pub_Servo1 = self.create_publisher(Int32,"/servo_s1",20)
        
         # ตั้งค่า Timer ให้ทำงานทุก 2 วินาที
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info("Listening to /mediapipe/points...")
        self.human_warning = 0 # ตรวจสอบการตรวจพบมนุษย์
    def poseDetector_callback(self,msg):
        """ ตรวจสอบมนุษย์ """
        self.get_logger().info(f' การตรวจพบมนุษย์ = {self.human_warning} จำนวนจุด = {len(msg.points)}')
        if len(msg.points) > 0 :
            self.human_warning += 1 # พบเพิ่มทีละ 1 
        else:
            self.human_warning -= 2 # เมื่อไม่พบลบครั้งละ 2
        if self.human_warning < 0 : # ป้องกันให้ไม่ต่ำไปกว่า 0
            self.human_warning = 0
        if self.human_warning > 10 : # ป้องกันให้มากไปกว่า 10
            self.human_warning = 10
    def send_servo_signal(self, value):
        """ ส่งเสียง servo หรือปิดเสียงไปยังหัวข้อ /servo1 """
        s1_init_angle = Int32()
        s1_init_angle.data = value  # 1 = เปิดเสียง, 0 = ปิดเสียง
        self.pub_Servo1.publish(s1_init_angle)
        self.get_logger().info(f'{value} ไปที่หัวข้อ /servo1')
    def timer_callback(self):
        if self.human_warning > 4: # ตรวจพบมากว่า 4 
            self.send_servo_signal(-50) # ปล่อยของ
        else:
            self.send_servo_signal(0) # เตรียมพร้อม
            

def main():
    rclpy.init()
    node = dropToHumanSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

