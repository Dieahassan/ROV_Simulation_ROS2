#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped

class PressureToDepthNode(Node):
    def __init__(self):
        super().__init__('pressure_to_depth_node')
        self.sub = self.create_subscription(
            FluidPressure, 
            '/rov_2/depth_pose', 
            self.pressure_callback, 
            10)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/rov_2/pose_z', 
            10)

    def pressure_callback(self, msg):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'rov2/base_link' # أو base_link حسب المرجع لديك

        # حساب العمق z (ملاحظة: إذا كان المحاكي يعطيك الضغط المطلق Absolute Pressure)
        # P_atm = 101325.0
        # depth = (msg.fluid_pressure - P_atm) / (1000.0 * 9.81)
        
        # إذا كان المحاكي ينشر Relative Pressure أو Depth مباشر مقاساً بالباكال:
        depth = msg.fluid_pressure / (1000.0 * 9.81) 

        pose_msg.pose.pose.position.z = -abs(depth) # الاتجاه للأسفل سالب

        # تحديد التباين Covariance للعمق Z
        # المصفوفة 6x6 (36 عنصر)، العنصر رقم 14 يمثل Z
        cov = [0.0] * 36
        cov[14] = 0.01 # Z covariance
        pose_msg.pose.covariance = cov

        self.pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PressureToDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()