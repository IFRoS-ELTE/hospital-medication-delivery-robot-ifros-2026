import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException

from pomona_rrt import Planner


class GridChecker:
    # Reuse geometry/collision functions without starting another planner.
    to_grid = Planner.to_grid
    point_free = Planner.point_free
    edge_free = Planner.edge_free

    def __init__(self, msg):
        info = msg.info
        q = info.origin.orientation
        values = [
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            q.x, q.y, q.z, q.w,
        ]
        if (
            not all(math.isfinite(v) for v in values)
            or info.resolution <= 0
            or info.width == 0 or info.height == 0
            or len(msg.data) != info.width * info.height
            or abs(q.x) > 1e-6 or abs(q.y) > 1e-6
        ):
            raise ValueError("Invalid planar map")

        norm = math.hypot(q.z, q.w)
        if norm < 1e-9:
            raise ValueError("Invalid map quaternion")
        z, w = q.z / norm, q.w / norm
        yaw = math.atan2(2*w*z, 1-2*z*z)
        c, s = math.cos(yaw), math.sin(yaw)

        self.res = info.resolution
        self.origin = np.array([
            info.origin.position.x, info.origin.position.y
        ])
        self.rotation = np.array([[c, -s], [s, c]])
        self.grid = np.asarray(msg.data, dtype=np.int16).reshape(
            info.height, info.width
        )
        self.blocked = np.argwhere(self.grid != 0)[:, ::-1]
        self.frame = msg.header.frame_id
        self.stamp = Time.from_msg(msg.header.stamp)


class Follower(Node):
    def __init__(self):
        super().__init__("pomona_follower")

        self.cmd_pub = self.create_publisher(
            Twist, "/scout_mini/cmd_vel", 1
        )

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Volatile: wait for a newly published path after startup.
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/pomona/inflated_map",
            self.on_map, map_qos
        )
        self.path_sub = self.create_subscription(
            Path, "/pomona/rrt_path", self.on_path, path_qos
        )
        self.valid_sub = self.create_subscription(
            Bool, "/pomona/path_valid", self.on_valid, 1
        )
        self.ready_sub = self.create_subscription(
            Bool, "/pomona/costmap_ready", self.on_ready, 1
        )

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.checker = None
        self.map_wall = 0.0
        self.path = None
        self.path_frame = ""
        self.index = 1
        self.valid = False
        self.ready = False
        self.valid_wall = 0.0
        self.ready_wall = 0.0
        self.path_wall = 0.0
        self.turning = True
        self.final_yaw = None
        self.final_aligning = False
        self.yaw_tolerance = 0.05
        self.last_state = None

        # Keep checking timeouts even if simulation time pauses.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            0.05, self.update, clock=self.steady_clock
        )
        self.get_logger().info(
            "Follower final-heading hysteresis version ready. Send a NEW goal."
        )

    def state(self, text):
        if text != self.last_state:
            self.get_logger().info(text)
            self.last_state = text

    def stop(self, reason=None):
        self.cmd_pub.publish(Twist())
        if reason:
            self.state(reason)

    def on_map(self, msg):
        try:
            self.checker = GridChecker(msg)
            self.map_wall = time.monotonic()
        except ValueError:
            self.checker = None
            self.stop("Stopped: invalid costmap")

    def on_valid(self, msg):
        self.valid = msg.data
        self.valid_wall = time.monotonic()
        if not msg.data:
            self.stop()

    def on_ready(self, msg):
        self.ready = msg.data
        self.ready_wall = time.monotonic()
        if not msg.data:
            self.stop()

    def on_path(self, msg):
        self.stop()
        self.path = None
        self.final_yaw = None
        self.final_aligning = False
        self.index = 1
        self.turning = True
        self.path_wall = time.monotonic()

        if len(msg.poses) < 2:
            self.state("Stopped: path cleared")
            return

        points = np.array([
            [p.pose.position.x, p.pose.position.y]
            for p in msg.poses
        ])
        if not np.all(np.isfinite(points)):
            self.state("Stopped: invalid path")
            return

        q = msg.poses[-1].pose.orientation
        values = [q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            self.state("Stopped: invalid final heading")
            return
        norm = math.sqrt(sum(v*v for v in values))
        if norm < 1e-9:
            self.state("Stopped: zero final quaternion")
            return
        x, y, z, w = [v/norm for v in values]
        if abs(x) > 1e-3 or abs(y) > 1e-3:
            self.state("Stopped: non-planar final heading")
            return
        self.final_yaw = math.atan2(2*w*z, 1-2*z*z)
        self.path = points
        self.path_frame = msg.header.frame_id
        self.state(f"New path: {len(points)} waypoints")

    def update(self):
        wall = time.monotonic()

        if self.path is None:
            self.stop()
            return

        if (
            self.checker is None
            or not self.ready or not self.valid
            or wall - self.map_wall > 0.6
            or wall - self.ready_wall > 0.6
            or wall - self.valid_wall > 0.6
            or self.valid_wall < self.path_wall
        ):
            self.stop("Stopped: waiting for fresh map/path status")
            return

        checker = self.checker
        map_age = (
            self.get_clock().now() - checker.stamp
        ).nanoseconds * 1e-9

        if (
            not -0.05 <= map_age < 0.6
            or self.path_frame != checker.frame
        ):
            self.stop("Stopped: stale map or frame mismatch")
            return

        try:
            tf = self.buffer.lookup_transform(
                checker.frame, "base_link", Time()
            )
        except TransformException:
            self.stop("Stopped: missing robot TF")
            return

        tf_age = (
            self.get_clock().now()
            - Time.from_msg(tf.header.stamp)
        ).nanoseconds * 1e-9
        if not -0.5 <= tf_age < 0.6:
            self.stop("Stopped: stale robot TF")
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        position = np.array([t.x, t.y])
        yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1-2*(q.y*q.y + q.z*q.z),
        )

        if not np.all(np.isfinite(position)) or not math.isfinite(yaw):
            self.stop("Stopped: invalid robot pose")
            return

        # Require a collision-free connection before changing waypoint.
        while (
            self.index < len(self.path) - 1
            and np.linalg.norm(
                self.path[self.index] - position
            ) <= 0.03
        ):
            if not checker.edge_free(
                checker.to_grid(position),
                checker.to_grid(self.path[self.index + 1]),
            ):
                self.stop("Stopped: next segment blocked")
                return
            self.index += 1
            self.turning = True

        remaining = [position] + list(self.path[self.index:])
        cells = [checker.to_grid(p) for p in remaining]

        if not all(
            checker.edge_free(a, b)
            for a, b in zip(cells[:-1], cells[1:])
        ):
            self.stop("Stopped: remaining path blocked")
            return

        delta = self.path[self.index] - position
        distance = float(np.linalg.norm(delta))

        at_last_waypoint = self.index == len(self.path) - 1
        if at_last_waypoint and distance <= 0.08:
            self.final_aligning = True
        elif self.final_aligning and distance > 0.12:
            self.final_aligning = False
            self.turning = True
            self.state("Final position drifted beyond 12 cm; correcting position")

        if at_last_waypoint and self.final_aligning:
            # Inflation covers the robot's circumscribed radius, so
            # checking the current centre also protects in-place turns.
            if not checker.point_free(checker.to_grid(position)):
                self.stop("Stopped: final rotation position blocked")
                return
            error = math.atan2(
                math.sin(self.final_yaw - yaw),
                math.cos(self.final_yaw - yaw),
            )
            if abs(error) <= self.yaw_tolerance:
                self.path = None
                self.stop("Goal reached: position and heading")
            else:
                cmd = Twist()
                cmd.angular.z = float(np.clip(1.5 * error, -0.30, 0.30))
                self.state("Aligning final heading")
                self.cmd_pub.publish(cmd)
            return

        target_yaw = math.atan2(delta[1], delta[0])
        error = math.atan2(
            math.sin(target_yaw - yaw),
            math.cos(target_yaw - yaw),
        )

        # Hysteresis avoids switching rapidly between turning/driving.
        if abs(error) > 0.15:
            self.turning = True
        elif abs(error) < 0.05:
            self.turning = False

        cmd = Twist()
        cmd.angular.z = float(np.clip(1.5 * error, -0.30, 0.30))

        if self.turning:
            self.state("Turning")
        else:
            cmd.linear.x = min(0.10, 0.6 * distance)

            # Check a short forward segment as well as the planned path.
            ahead = position + 0.15 * np.array([
                math.cos(yaw), math.sin(yaw)
            ])
            if not checker.edge_free(
                checker.to_grid(position), checker.to_grid(ahead)
            ):
                self.stop("Stopped: forward clearance blocked")
                return

            self.state(f"Driving to waypoint {self.index}")

        self.cmd_pub.publish(cmd)


def main(args=None):
    # Keep the ROS context alive long enough to send stop on Ctrl+C.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = Follower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(5):
                node.stop()
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()