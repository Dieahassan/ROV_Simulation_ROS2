#!/usr/bin/env python3
"""
Down-camera velocity estimator (visual odometry without feature points).

Uses phase correlation between consecutive frames of the downward camera
(works on low-texture floors, unlike ORB/KLT features) and converts the pixel
shift into metric body velocity using the altitude above the pool floor:

    altitude = pool_depth - rov_depth        (rov_depth from /rov_2/pose_z)
    metres   = pixels * altitude / focal_px  (pinhole, floor assumed flat and
                                              camera looking straight down)

Output: geometry_msgs/TwistWithCovarianceStamped (linear x/y only is valid),
to be fused by robot_localization as twist0.

NOTE: the image-axis -> body-axis mapping (sign_x, sign_y, swap_xy) depends on
how the camera is mounted. The defaults are a first guess and must be
calibrated against /rov_2/navigator/odometry (ground truth in the simulator).
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image


class DownCamOdom(Node):
    def __init__(self):
        super().__init__('down_cam_odom')

        # --- parameters -----------------------------------------------------
        self.declare_parameter('image_topic', '/rov_2/camera_down/image_color')
        self.declare_parameter('depth_topic', '/rov_2/pose_z')
        self.declare_parameter('output_topic', '/rov_2/down_cam/twist')
        self.declare_parameter('frame_id', 'rov2/base_link')

        self.declare_parameter('pool_depth', 10.0)       # metres (assumed, to be updated)
        self.declare_parameter('horizontal_fov_deg', 85.0)
        self.declare_parameter('min_altitude', 0.3)      # clamp, metres
        self.declare_parameter('downscale', 0.5)         # speed vs. precision
        self.declare_parameter('min_response', 0.05)     # phase-corr. confidence
        self.declare_parameter('max_dt', 0.5)            # drop pairs further apart (s)
        self.declare_parameter('max_speed', 2.0)         # reject outliers (m/s)

        # image axes -> body axes (calibrate against ground-truth odometry)
        self.declare_parameter('swap_xy', False)
        self.declare_parameter('sign_x', -1.0)
        self.declare_parameter('sign_y', -1.0)

        # covariance of the reported velocity: sigma^2 = base / response
        self.declare_parameter('base_variance', 0.0004)

        p = self.get_parameter
        self.pool_depth = p('pool_depth').value
        self.fov = math.radians(p('horizontal_fov_deg').value)
        self.min_alt = p('min_altitude').value
        self.downscale = p('downscale').value
        self.min_response = p('min_response').value
        self.max_dt = p('max_dt').value
        self.max_speed = p('max_speed').value
        self.swap_xy = p('swap_xy').value
        self.sign_x = p('sign_x').value
        self.sign_y = p('sign_y').value
        self.base_var = p('base_variance').value
        self.frame_id = p('frame_id').value

        # --- state ----------------------------------------------------------
        self.bridge = CvBridge()
        self.prev_gray = None
        self.prev_stamp = None
        self.window = None
        self.rov_depth = None  # positive down, metres

        # --- ROS interfaces -------------------------------------------------
        self.pub = self.create_publisher(
            TwistWithCovarianceStamped, p('output_topic').value, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, p('depth_topic').value,
            self.depth_cb, 10)
        # RELIABLE: the simulator publishes RELIABLE and best-effort drops most large frames
        img_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(
            Image, p('image_topic').value, self.image_cb, img_qos)

        self.get_logger().info(
            f"down_cam_odom ready | pool_depth={self.pool_depth} m (assumed) | "
            f"image={p('image_topic').value}")

    # ------------------------------------------------------------------
    def depth_cb(self, msg: PoseWithCovarianceStamped):
        # p2z.py publishes z negative downwards
        self.rov_depth = -msg.pose.pose.position.z

    # ------------------------------------------------------------------
    def _dbg(self, key, val=0.0):  # DBGTMP
        import time as _t
        d = self.__dict__.setdefault('_dbgd', {})
        c, sm = d.get(key, (0, 0.0)); d[key] = (c + 1, sm + val)
        now = _t.time(); last = self.__dict__.setdefault('_dbglast', now)
        if now - last >= 2.0:
            self._dbglast = now
            self.get_logger().info('[DBG] ' + ' '.join(
                f'{k}={c}' + (f'(avg {sm/c:.4f})' if sm else '') for k, (c, sm) in sorted(d.items())))
            self._dbgd = {}

    def image_cb(self, msg: Image):
        self._dbg('img')  # DBGTMP
        if self.rov_depth is None:
            self.get_logger().warn(
                'waiting for depth on /rov_2/pose_z (is p2z running?)',
                throttle_duration_sec=5.0)
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception:
            # colour encodings: convert manually
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        if self.downscale != 1.0:
            frame = cv2.resize(frame, None, fx=self.downscale, fy=self.downscale,
                               interpolation=cv2.INTER_AREA)
        gray = np.float32(frame)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.window is None or self.window.shape != gray.shape:
            self.window = cv2.createHanningWindow(
                (gray.shape[1], gray.shape[0]), cv2.CV_32F)

        if self.prev_gray is None or self.prev_gray.shape != gray.shape:
            self.prev_gray, self.prev_stamp = gray, stamp
            return

        dt = stamp - self.prev_stamp
        prev, self.prev_gray, self.prev_stamp = self.prev_gray, gray, stamp
        if dt <= 0.0 or dt > self.max_dt:
            self._dbg('bad_dt', dt)  # DBGTMP
            return

        # shift of the current frame relative to the previous one (pixels)
        import time as _t0; _a = _t0.time()  # DBGTMP
        (sx, sy), response = cv2.phaseCorrelate(prev, gray, self.window)
        self._dbg('pc_sec', _t0.time() - _a)  # DBGTMP
        if response < self.min_response:
            self._dbg('low_resp')  # DBGTMP
            self.get_logger().warn(
                f'low phase-correlation response ({response:.3f}), skipping',
                throttle_duration_sec=2.0)
            return

        # undo the downscale -> pixels of the original image
        sx /= self.downscale
        sy /= self.downscale

        width_px = gray.shape[1] / self.downscale
        focal_px = (width_px / 2.0) / math.tan(self.fov / 2.0)

        altitude = max(self.pool_depth - self.rov_depth, self.min_alt)
        metres_per_px = altitude / focal_px

        # scene moves opposite to the camera, sign_x / sign_y handle that and
        # the camera mounting orientation
        if self.swap_xy:
            sx, sy = sy, sx
        vx = self.sign_x * sx * metres_per_px / dt
        vy = self.sign_y * sy * metres_per_px / dt

        if math.hypot(vx, vy) > self.max_speed:
            self._dbg('outlier')  # DBGTMP
            self.get_logger().warn(
                f'rejected outlier velocity ({vx:.2f}, {vy:.2f}) m/s',
                throttle_duration_sec=2.0)
            return

        out = TwistWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.twist.twist.linear.x = float(vx)
        out.twist.twist.linear.y = float(vy)

        var = self.base_var / max(response, 1e-3)
        cov = [0.0] * 36
        cov[0] = var            # vx
        cov[7] = var            # vy
        cov[14] = 1e6           # vz (unused)
        cov[21] = cov[28] = cov[35] = 1e6  # angular (unused)
        out.twist.covariance = cov
        self.pub.publish(out)
        self._dbg('pub')  # DBGTMP

        self.get_logger().info(
            f'v=({vx:+.3f}, {vy:+.3f}) m/s | alt={altitude:.2f} m | '
            f'resp={response:.2f}', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = DownCamOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
