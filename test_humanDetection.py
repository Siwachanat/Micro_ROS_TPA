import rclpy
from rclpy.node import Node
from yahboomcar_msgs.msg import PointArray

class humanDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('human_subscriber')
        self.pub_pose = self.create_subscription(PointArray,"/mediapipe/points",self.poseDetector_callback,3)
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

def main():
    rclpy.init()
    node = humanDetectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
