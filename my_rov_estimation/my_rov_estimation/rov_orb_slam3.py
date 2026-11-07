#!/usr/bin/env python3

"""
Python driver node for Monocular ORB-SLAM3.
Subscribes directly to `/rov_2/camera_front/image_color` and forwards
the image along with its extracted timestamp to the C++ SLAM node.
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MonoDriver(Node):
    def __init__(self, node_name="mono_py_node"):
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter("settings_name", "EuRoC")
        self.settings_name = str(self.get_parameter('settings_name').value)

        # Global state flags
        self.send_config = True  # Handshake flag

        # Output ROS 2 topic names (Forwarding to C++ ORB-SLAM3 node)
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings" 
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"

        # Input camera topic name
        self.camera_topic = "/rov_2/camera_front/image_color"

        # Setup publishers to C++ Node
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1)
        self.publish_img_msg_ = self.create_publisher(Image, self.pub_img_to_agent_name, 1)
        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_to_agent_name, 1)

        # Subscriber to receive handshake ACK from C++ node
        self.subscribe_exp_ack_ = self.create_subscription(
            String, self.sub_exp_ack_name, self.ack_callback, 10
        )

        # QoS profile setup to handle camera streams smoothly (Best Effort / Sensor Data)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber for your ROV camera topic
        self.sub_camera = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            qos_profile
        )

        self.exp_config_msg = self.settings_name
        self.get_logger().info(f"MonoDriver initialized. Listening to {self.camera_topic}")
        self.get_logger().info("Attempting handshake with C++ ORB-SLAM3 node...")

    def ack_callback(self, msg):
        """ Callback to receive handshake ACK from C++ node """
        self.get_logger().info(f"Got ack: {msg.data}")
        if msg.data == "ACK":
            self.send_config = False

    def handshake_with_cpp_node(self):
        """ Send experiment configuration to C++ node until ACK is received """
        if self.send_config:
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)

    def camera_callback(self, img_msg: Image):
        """
        Triggered every time a frame arrives from /rov_2/camera_front/image_color.
        Extracts the exact ROS timestamp from the image header and forwards both
        the timestamp and image to the SLAM node once handshake completes.
        """
        if not self.send_config:
            # 1. Extract ROS header timestamp (seconds + nanoseconds -> total nanoseconds)
            header_sec = img_msg.header.stamp.sec
            header_nanosec = img_msg.header.stamp.nanosec

            if header_sec == 0 and header_nanosec == 0:
                # Fallback if camera driver isn't setting ROS header timestamps
                timestamp_ns = float(self.get_clock().now().nanoseconds)
            else:
                timestamp_ns = float((header_sec * 1e9) + header_nanosec)

            # 2. Package timestamp into Float64 message
            timestep_msg = Float64()
            timestep_msg.data = timestamp_ns

            # 3. Publish timestamp first, followed by the image (preserving C++ node ordering)
            self.publish_timestep_msg_.publish(timestep_msg)
            self.publish_img_msg_.publish(img_msg)

            self.get_logger().info(f"Forwarded frame to SLAM | Timestamp (ns): {timestep_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    n = MonoDriver("mono_py_node")

    # Step 1: Handshake loop with C++ node
    while rclpy.ok() and n.send_config:
        n.handshake_with_cpp_node()
        rclpy.spin_once(n, timeout_sec=0.01)

    n.get_logger().info("Handshake complete. Stream active to SLAM node.")

    # Step 2: Spin node to continuously handle camera callbacks
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()