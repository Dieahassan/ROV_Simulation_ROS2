#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import numpy as np 
import os
import cv2

# Use pupil_apriltags directly for clean Python bindings
from pupil_apriltags import Detector
from cv_bridge import CvBridge

class ImageProcessing(Node):
    def __init__(self):
        super().__init__('my_rov_apriltag_estimation')

        # Initialize the Image Publisher
        self.apiltag_image_publisher = self.create_publisher(Image, 'rov_2/estimation/apriltag', 10)
        
        # Subscriptions
        self.subscription_thruster_state = self.create_subscription(
            Image,
            '/rov_2/camera_front/image_color',
            self.camera_image_callback,
            10
        )

        self.camera_params = None
        self.subscription_camera_info = self.create_subscription(
            CameraInfo,
            '/rov_2/camera_front/camera_info',
            self.camera_info_callback,
            10
        )

        self._cv_bridge = CvBridge()

        # Tag size in meters
        self.tag_size = 0.160

        # Initialize AprilTag Detector directly
        self.apriltag_detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=2.0,        # Downsample by 2x (reduces contours)
            quad_sigma=0.8,           # Apply blur to reduce noise
            refine_edges=1,           # Enable edge refinement
            debug=0
        )

        self.get_logger().info('MOLA AprilTag Estimation Node Initialized')

    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera parameters from CameraInfo message"""
        if self.camera_params is None:
            # K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.camera_params = [
                msg.k[0],  # fx
                msg.k[4],  # fy
                msg.k[2],  # cx
                msg.k[5]   # cy
            ]
            self.get_logger().info(f"Camera intrinsics received: {self.camera_params}")

    def estimate_tag_pose(self, corners, camera_params, tag_size):
        """Estimate 4x4 pose matrix using OpenCV solvePnP."""
        fx, fy, cx, cy = camera_params
        
        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float64)
        
        dist_coeffs = np.zeros(5) # Assuming zero distortion
        
        # 3D coordinates of tag corners in the tag local frame (centered at origin)
        s = tag_size / 2.0
        object_points = np.array([
            [-s, -s, 0],
            [ s, -s, 0],
            [ s,  s, 0],
            [-s,  s, 0]
        ], dtype=np.float64)
        
        image_points = corners.astype(np.float64)
        
        # Solve Perspective-n-Point
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, K, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        
        if not success:
            return None
            
        R, _ = cv2.Rodrigues(rvec)
        
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = R
        pose_matrix[:3, 3] = tvec.flatten()
        
        return pose_matrix

    def camera_image_callback(self, mola_image_msg: Image):
        try: 
            cv_image = self._cv_bridge.imgmsg_to_cv2(mola_image_msg, desired_encoding='bgr8')
        except Exception as e: 
            self.get_logger().error(f"Error converting image: {e}")
            return
        
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        apriltag_results = self.apriltag_detector.detect(gray_img)

        for r in apriltag_results:
            # Extract bounding box corner coordinates
            (ptA, ptB, ptC, ptD) = r.corners
            ptA = (int(ptA[0]), int(ptA[1]))
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))

            # Draw bounding box
            cv2.line(cv_image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(cv_image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(cv_image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(cv_image, ptD, ptA, (0, 255, 0), 2)

            # Draw center point
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)

            # Draw tag ID
            tagId = str(r.tag_id)
            cv2.putText(cv_image, tagId, (ptA[0], ptA[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            
            # Estimate 3D pose and draw axes if camera parameters are ready
            if self.camera_params is not None:
                pose_matrix = self.estimate_tag_pose(r.corners, self.camera_params, self.tag_size)
                if pose_matrix is not None:
                    self._draw_pose_axes(cv_image, pose_matrix, self.camera_params, self.tag_size)

        # Publish output annotated image
        apriltag_img_msg = self._cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        apriltag_img_msg.header = mola_image_msg.header
        self.apiltag_image_publisher.publish(apriltag_img_msg)

    def _draw_pose_axes(self, image, pose_matrix, camera_params, tag_size):
        """Draw 3D coordinate axes on the detected tag"""
        axis_length = tag_size / 2.0
        opoints = np.array([
            [0, 0, 0],              # origin
            [axis_length, 0, 0],    # x-axis point
            [0, axis_length, 0],    # y-axis point
            [0, 0, -axis_length]    # z-axis point (pointing outward)
        ], dtype=np.float64).reshape(-1, 1, 3)
        
        fx, fy, cx, cy = camera_params
        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float64)
        
        rvec, _ = cv2.Rodrigues(pose_matrix[:3, :3])
        tvec = pose_matrix[:3, 3]
        
        dcoeffs = np.zeros(5)
        ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)
        ipoints = np.round(ipoints).astype(int).reshape(-1, 2)
        
        origin = tuple(ipoints[0])
        cv2.line(image, origin, tuple(ipoints[1]), (0, 0, 255), 3)    # X-axis (Red)
        cv2.line(image, origin, tuple(ipoints[2]), (0, 255, 0), 3)    # Y-axis (Green)
        cv2.line(image, origin, tuple(ipoints[3]), (255, 0, 0), 3)    # Z-axis (Blue)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessing()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()