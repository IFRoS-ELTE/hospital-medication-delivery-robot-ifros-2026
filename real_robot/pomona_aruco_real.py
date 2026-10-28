#!/usr/bin/env python3
"""RealSense ArUco ID 0 position probe. Never publishes navigation commands.

Positions are marker centres, in metres. Consumers must check timestamps
and /pomona/aruco/detected; previous points are not cleared on marker loss.
Run inside Pomona's ROS container, with its existing camera bringup.
"""
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException


def rotation(q):
    v = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(v)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = v / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


class ArucoReal(Node):
    def __init__(self):
        super().__init__('pomona_aruco_real')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Real robot node requires use_sim_time=false')
        defaults = {
            'image_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'target_frame': 'base_link', 'marker_id': 0,
            'marker_size_m': 0.10, 'process_hz': 5.0,
            'max_age_s': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value
        self.target = get('target_frame')
        self.marker_id = get('marker_id')
        self.size = float(get('marker_size_m'))
        hz = float(get('process_hz'))
        self.max_age = float(get('max_age_s'))
        if not all(math.isfinite(v) and v > 0 for v in (self.size, hz, self.max_age)):
            raise ValueError('Size, rate and maximum age must be positive')
        self.period = 1.0 / hz
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.info = None
        self.last_test = -math.inf
        self.last_success = None
        self.last_stamp = None
        self.last_log = -math.inf
        self.camera_pub = self.create_publisher(PointStamped, '/pomona/aruco/point_camera', 1)
        self.base_pub = self.create_publisher(PointStamped, '/pomona/aruco/point_base', 1)
        self.detected_pub = self.create_publisher(Bool, '/pomona/aruco/detected', 1)
        self.info_sub = self.create_subscription(CameraInfo, get('camera_info_topic'), self.on_info, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image, get('image_topic'), self.on_image, qos_profile_sensor_data)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.status, clock=self.steady_clock)
        self.get_logger().info(f'Real ArUco: ID={self.marker_id}, size={self.size}m, target={self.target}; no goals or motion commands')

    def on_info(self, msg):
        self.info = msg

    def age(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def lose(self):
        self.last_success = None
        self.last_stamp = None
        self.detected_pub.publish(Bool(data=False))

    def status(self):
        fresh = (self.last_success is not None
                 and time.monotonic() - self.last_success < self.max_age
                 and -0.05 <= self.age(self.last_stamp) < self.max_age)
        self.detected_pub.publish(Bool(data=fresh))

    @staticmethod
    def point(header, frame, xyz):
        msg = PointStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = frame
        msg.point.x, msg.point.y, msg.point.z = map(float, xyz)
        return msg

    def on_image(self, msg):
        wall = time.monotonic()
        if wall - self.last_test < self.period:
            return
        self.last_test = wall
        stamp = Time.from_msg(msg.header.stamp)
        info = self.info
        if (info is None or not -0.05 <= self.age(stamp) < self.max_age
                or msg.width != info.width or msg.height != info.height
                or msg.header.frame_id != info.header.frame_id
                or info.distortion_model != 'plumb_bob'):
            self.lose()
            return
        try:
            matrix = np.array(info.k, dtype=float).reshape(3, 3)
            distortion = np.array(info.d, dtype=float)
            if (not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(distortion))
                    or matrix[0, 0] <= 0 or matrix[1, 1] <= 0):
                raise ValueError('Invalid camera calibration')
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
            indices = [] if ids is None else np.flatnonzero(ids.flatten() == self.marker_id)
            if len(indices) != 1:
                self.lose()
                return
            _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[int(indices[0])]], self.size, matrix, distortion)
            camera = tvecs[0, 0]
            if not np.all(np.isfinite(camera)) or camera[2] <= 0:
                raise ValueError('Invalid marker position')
            tf = self.buffer.lookup_transform(self.target, msg.header.frame_id, stamp).transform
            t = tf.translation
            base = rotation(tf.rotation) @ camera + np.array([t.x, t.y, t.z])
            if not np.all(np.isfinite(base)):
                raise ValueError('Invalid transformed position')
            if not -0.05 <= self.age(stamp) < self.max_age:
                self.lose()
                return
            self.camera_pub.publish(self.point(msg.header, msg.header.frame_id, camera))
            self.base_pub.publish(self.point(msg.header, self.target, base))
            self.last_success = time.monotonic()
            self.last_stamp = stamp
            self.status()
            if wall - self.last_log >= 1.0:
                self.get_logger().info(f'ID={self.marker_id} {self.target}: x={base[0]:.3f} y={base[1]:.3f} z={base[2]:.3f} m')
                self.last_log = wall
        except (TransformException, ValueError, cv2.error, RuntimeError) as exc:
            self.lose()
            if wall - self.last_log >= 2.0:
                self.get_logger().warning(str(exc))
                self.last_log = wall


def main():
    cv2.setNumThreads(1)
    rclpy.init()
    node = ArucoReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.lose()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
