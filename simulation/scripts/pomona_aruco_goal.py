import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException


class ArucoGoal(Node):
    def __init__(self):
        super().__init__("pomona_aruco_goal")
        self.declare_parameter("stop_distance", 0.80)
        self.stop_distance = float(
            self.get_parameter("stop_distance").value
        )
        if not math.isfinite(self.stop_distance) or self.stop_distance <= 0:
            raise ValueError("stop_distance must be positive")

        self.samples = deque(maxlen=5)
        self.marker_wall = 0.0
        self.costmap = None
        self.map_wall = 0.0
        self.ready = False
        self.ready_wall = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            PointStamped,
            "/pomona/aruco/position_map",
            self.on_marker,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            "/pomona/inflated_map",
            self.on_map,
            map_qos,
        )
        self.create_subscription(
            Bool, "/pomona/costmap_ready", self.on_ready, 10
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, "/goal_pose", 10
        )
        self.create_service(
            Trigger, "/pomona/aruco/send_goal", self.send_goal
        )

        self.get_logger().info(
            "Ready. Call /pomona/aruco/send_goal to send ONE ArUco goal."
        )

    def age(self, stamp):
        return (
            self.get_clock().now() - Time.from_msg(stamp)
        ).nanoseconds * 1e-9

    def on_marker(self, msg):
        p = np.array([msg.point.x, msg.point.y], dtype=float)
        if (
            msg.header.frame_id == "map"
            and np.all(np.isfinite(p))
            and -0.05 <= self.age(msg.header.stamp) <= 1.0
        ):
            self.samples.append((msg.header.stamp, p))
            self.marker_wall = time.monotonic()

    def on_map(self, msg):
        self.costmap = msg
        self.map_wall = time.monotonic()

    def on_ready(self, msg):
        self.ready = msg.data
        self.ready_wall = time.monotonic()

    def free_cell(self, x, y):
        msg = self.costmap
        info = msg.info
        q = info.origin.orientation
        values = [
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            q.x, q.y, q.z, q.w,
        ]

        if (
            msg.header.frame_id != "map"
            or not all(math.isfinite(v) for v in values)
            or info.resolution <= 0
            or info.width <= 0
            or info.height <= 0
            or len(msg.data) != info.width * info.height
        ):
            raise ValueError("Invalid costmap geometry")

        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-9:
            raise ValueError("Invalid costmap orientation")

        qx, qy, qz, qw = (
            q.x/norm, q.y/norm, q.z/norm, q.w/norm
        )
        if abs(qx) > 1e-3 or abs(qy) > 1e-3:
            raise ValueError("Costmap origin must be planar")

        yaw = math.atan2(
            2 * (qw*qz + qx*qy),
            1 - 2 * (qy*qy + qz*qz),
        )
        dx = x - info.origin.position.x
        dy = y - info.origin.position.y
        c, s = math.cos(yaw), math.sin(yaw)

        gx = math.floor((c*dx + s*dy) / info.resolution)
        gy = math.floor((-s*dx + c*dy) / info.resolution)

        return (
            0 <= gx < info.width
            and 0 <= gy < info.height
            and msg.data[gy * info.width + gx] == 0
        )

    def send_goal(self, request, response):
        def reject(message):
            response.success = False
            response.message = message
            self.get_logger().warning(message)
            return response

        wall = time.monotonic()
        if (
            not self.ready
            or wall - self.ready_wall > 1.0
            or self.costmap is None
            or wall - self.map_wall > 1.0
            or not -0.05 <= self.age(
                self.costmap.header.stamp
            ) <= 1.0
        ):
            return reject("Costmap is not fresh/ready; no goal sent.")

        if len(self.samples) < 5 or wall - self.marker_wall > 1.0:
            return reject("Waiting for five fresh marker observations.")

        if any(
            not -0.05 <= self.age(stamp) <= 1.0
            for stamp, _ in self.samples
        ):
            return reject("Marker observations are stale; no goal sent.")

        points = np.array([p for _, p in self.samples])
        marker = np.median(points, axis=0)
        if np.max(np.linalg.norm(points - marker, axis=1)) > 0.03:
            return reject("Marker position is unstable; try again.")

        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", Time()
            )
        except TransformException as error:
            return reject(f"Robot TF unavailable: {error}")

        if not -0.05 <= self.age(tf.header.stamp) <= 0.6:
            return reject("Robot TF is stale; no goal sent.")

        robot = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
        ])
        if not np.all(np.isfinite(robot)):
            return reject("Robot position is invalid.")

        direction = marker - robot
        distance = float(np.linalg.norm(direction))
        if distance <= self.stop_distance + 0.05:
            return reject("Robot is already near the stand-off distance.")

        direction /= distance
        target = marker - self.stop_distance * direction

        try:
            if not self.free_cell(*robot):
                return reject("Robot start is blocked on the costmap.")
            if not self.free_cell(*target):
                return reject("ArUco stand-off goal is blocked/outside map.")
        except ValueError as error:
            return reject(str(error))

        if self.goal_pub.get_subscription_count() == 0:
            return reject("No goal subscriber; start pomona_rrt.py first.")

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(target[0])
        goal.pose.position.y = float(target[1])

        yaw = math.atan2(direction[1], direction[0])
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)

        self.goal_pub.publish(goal)
        response.success = True
        response.message = (
            f"Goal sent: x={target[0]:.3f}, y={target[1]:.3f}; "
            f"stand-off={self.stop_distance:.2f} m"
        )
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = ArucoGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()