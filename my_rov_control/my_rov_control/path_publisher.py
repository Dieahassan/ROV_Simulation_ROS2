#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class ROVPathTracker(Node):
    def __init__(self):
        super().__init__('rov_path_tracker')

        # Maximum pose history to retain (prevents memory leak)
        self.max_poses = 5000 

        # Create Path Message Object
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world_ned'  # Ensure frame matches RViz fixed frame

        # Subscribe to ORB-SLAM3 pose or ROV Pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/orb_slam3/camera_pose',  # Topic where PoseStamped is published
            self.pose_callback,
            10
        )

        # Publisher for Path message
        self.path_pub = self.create_publisher(Path, '/rov/path', 10)

    def pose_callback(self, pose_msg: PoseStamped):
        # Update frame ID matching current incoming pose
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.header.frame_id = pose_msg.header.frame_id
        print(self.path_msg.header.frame_id)

        # Append new pose to path
        self.path_msg.poses.append(pose_msg)

        # Limit stored pose history size
        if len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses.pop(0)

        # Publish the full updated path array
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ROVPathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()