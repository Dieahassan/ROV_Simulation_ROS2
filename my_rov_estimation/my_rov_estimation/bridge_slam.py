#!/usr/bin/env python3

"""Forward the ROV stereo camera and IMU topics to SLAM topic names."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


class SlamSensorBridge(Node):
    def __init__(self):
        super().__init__('slam_sensor_bridge')

        topic_defaults = {
            'camera0_input': '/rov_2/camera_front/image_color',
            'camera0_output': '/cam0/image_raw',
            'camera1_input': '/rov_2/camera_front/image_color',
            'camera1_output': '/cam1/image_raw',
            'imu_input': '/rov_2/imu',
            'imu_output': '/imu0',
        }
        for name, default in topic_defaults.items():
            self.declare_parameter(name, default)

        topics = {
            name: self.get_parameter(name).get_parameter_value().string_value
            for name in topic_defaults
        }

        self.camera0_publisher = self.create_publisher(
            Image, topics['camera0_output'], 10)
        self.camera1_publisher = self.create_publisher(
            Image, topics['camera1_output'], 10)
        self.imu_publisher = self.create_publisher(
            Imu, topics['imu_output'], 10)

        self.camera0_subscription = self.create_subscription(
            Image,
            topics['camera0_input'],
            self.camera0_publisher.publish,
            qos_profile_sensor_data,
        )
        self.camera1_subscription = self.create_subscription(
            Image,
            topics['camera1_input'],
            self.camera1_publisher.publish,
            qos_profile_sensor_data,
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            topics['imu_input'],
            self.imu_publisher.publish,
            qos_profile_sensor_data,
        )

        for source, destination in (
            ('camera0_input', 'camera0_output'),
            ('camera1_input', 'camera1_output'),
            ('imu_input', 'imu_output'),
        ):
            self.get_logger().info(
                f"Forwarding {topics[source]} -> {topics[destination]}")


def main(args=None):
    rclpy.init(args=args)
    node = SlamSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
