import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException


class Planner(Node):
    def __init__(self):
        super().__init__("pomona_rrt")

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.path_pub = self.create_publisher(
            Path, "/pomona/rrt_path", map_qos
        )
        self.valid_pub = self.create_publisher(
            Bool, "/pomona/path_valid", 1
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/pomona/inflated_map",
            self.on_map,
            map_qos,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self.on_goal, 1
        )
        self.ready_sub = self.create_subscription(
            Bool, "/pomona/costmap_ready", self.on_ready, 1
        )

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.frame = "map"
        self.grid = None
        self.geometry = None
        self.goal = None
        self.path = None

        self.ready = False
        self.ready_wall = 0.0
        self.map_wall = 0.0
        self.map_stamp = None

        self.next_attempt = 0.0
        self.last_message = None
        self.rng = np.random.default_rng()

        self.goal_yaw = None
        self.yaw_tolerance = 0.05
        self.final_aligning = False
        self.final_exit_tolerance = 0.12
        self.goal_tolerance = 0.08
        self.waypoint_tolerance = 0.05
        self.max_deviation = 0.20

        self.timer = self.create_timer(0.2, self.update)

        self.get_logger().info(
            "RRT final-heading hysteresis version ready; "
            "waiting for costmap and /goal_pose"
        )

    def report(self, message):
        if message != self.last_message:
            self.get_logger().info(message)
            self.last_message = message

    def clear_path(self):
        self.path = None
        self.final_aligning = False

        msg = Path()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()

        self.path_pub.publish(msg)
        self.valid_pub.publish(Bool(data=False))

    def on_ready(self, msg):
        self.ready = msg.data
        self.ready_wall = time.monotonic()

        if not self.ready and self.path is not None:
            self.clear_path()

    def on_map(self, msg):
        info = msg.info
        q = info.origin.orientation
        p = info.origin.position

        numbers = [
            info.resolution,
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
        ]

        if (
            not all(math.isfinite(v) for v in numbers)
            or info.width == 0
            or info.height == 0
            or info.resolution <= 0
            or len(msg.data) != info.width * info.height
            or not msg.header.frame_id
        ):
            self.grid = None
            self.clear_path()
            self.report("Invalid costmap")
            return

        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-9:
            self.grid = None
            self.clear_path()
            self.report("Invalid map origin quaternion")
            return

        qx, qy, qz, qw = (
            q.x / norm, q.y / norm, q.z / norm, q.w / norm
        )

        if abs(qx) > 1e-6 or abs(qy) > 1e-6:
            self.grid = None
            self.clear_path()
            self.report("Map origin must be planar")
            return

        yaw = math.atan2(
            2.0 * qw * qz,
            1.0 - 2.0 * qz * qz,
        )

        geometry = (
            msg.header.frame_id,
            info.width,
            info.height,
            info.resolution,
            p.x, p.y, yaw,
        )

        if geometry != self.geometry:
            self.clear_path()
            self.geometry = geometry
            self.next_attempt = 0.0

        self.frame = msg.header.frame_id
        self.res = info.resolution
        self.origin = np.array([p.x, p.y])

        c, s = math.cos(yaw), math.sin(yaw)
        self.rotation = np.array([[c, -s], [s, c]])

        self.grid = np.asarray(
            msg.data, dtype=np.int16
        ).reshape(info.height, info.width)

        # Only known-free cells are traversable.
        self.blocked = np.argwhere(self.grid != 0)[:, ::-1]
        self.free_cells = np.argwhere(self.grid == 0)[:, ::-1]

        self.map_stamp = Time.from_msg(msg.header.stamp)
        self.map_wall = time.monotonic()

    def on_goal(self, msg):
        self.clear_path()
        self.goal = None

        if not (
            math.isfinite(msg.pose.position.x)
            and math.isfinite(msg.pose.position.y)
        ):
            self.report("Rejected non-finite goal")
            return

        q = msg.pose.orientation
        values = [q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            self.report("Rejected non-finite goal orientation")
            return
        norm = math.sqrt(sum(v*v for v in values))
        if norm < 1e-9:
            self.report("Rejected zero goal quaternion")
            return
        x, y, z, w = [v/norm for v in values]
        if abs(x) > 1e-3 or abs(y) > 1e-3:
            self.report("Goal orientation must be planar")
            return
        self.goal_yaw = math.atan2(2*w*z, 1-2*z*z)
        self.goal = msg
        self.next_attempt = 0.0
        self.report("New goal received")

    def to_grid(self, xy):
        return (
            (np.asarray(xy) - self.origin)
            @ self.rotation
            / self.res
        )

    def to_world(self, xy):
        return (
            (np.asarray(xy) * self.res)
            @ self.rotation.T
            + self.origin
        )

    def point_free(self, point):
        if not np.all(np.isfinite(point)):
            return False

        x, y = np.floor(point).astype(int)
        height, width = self.grid.shape

        return (
            0 <= x < width
            and 0 <= y < height
            and self.grid[y, x] == 0
        )

    def edge_free(self, a, b):
        # Test the entire segment against blocked cell rectangles.
        # Contact with the boundary of a blocked cell is collision.
        a = np.asarray(a)
        b = np.asarray(b)

        if not self.point_free(a) or not self.point_free(b):
            return False

        lower = np.minimum(a, b)
        upper = np.maximum(a, b)

        cells = self.blocked
        mask = np.all(
            (cells <= upper + 1e-9)
            & (cells + 1 >= lower - 1e-9),
            axis=1,
        )
        cells = cells[mask]

        if len(cells) == 0:
            return True

        enter = np.zeros(len(cells))
        leave = np.ones(len(cells))
        possible = np.ones(len(cells), dtype=bool)
        delta = b - a

        for axis in (0, 1):
            low = cells[:, axis] - 1e-9
            high = cells[:, axis] + 1 + 1e-9

            if abs(delta[axis]) < 1e-12:
                possible &= (
                    (a[axis] >= low)
                    & (a[axis] <= high)
                )
            else:
                t1 = (low - a[axis]) / delta[axis]
                t2 = (high - a[axis]) / delta[axis]

                enter = np.maximum(enter, np.minimum(t1, t2))
                leave = np.minimum(leave, np.maximum(t1, t2))

        return not np.any(possible & (enter <= leave))

    def plan(self, start, goal):
        if self.edge_free(start, goal):
            return [start.copy(), goal.copy()]

        if len(self.free_cells) == 0:
            return None

        deadline = time.monotonic() + 0.25
        step = 0.4 / self.res

        nodes = [start.copy()]
        parents = [-1]

        for _ in range(5000):
            if time.monotonic() > deadline:
                break

            if self.rng.random() < 0.15:
                sample = goal
            else:
                cell = self.free_cells[
                    self.rng.integers(len(self.free_cells))
                ]
                sample = cell + self.rng.uniform(0.05, 0.95, 2)

            points = np.asarray(nodes)
            nearest = int(np.argmin(
                np.sum((points - sample) ** 2, axis=1)
            ))

            direction = sample - nodes[nearest]
            length = np.linalg.norm(direction)

            if length < 1e-9:
                continue

            new = (
                nodes[nearest]
                + direction * min(1.0, step / length)
            )

            if not self.edge_free(nodes[nearest], new):
                continue

            nodes.append(new)
            parents.append(nearest)
            index = len(nodes) - 1

            if (
                np.linalg.norm(new - goal) <= 2 * step
                and self.edge_free(new, goal)
            ):
                route = [goal.copy()]

                while index != -1:
                    route.append(nodes[index])
                    index = parents[index]

                route.reverse()

                # Shortcut only through collision-free segments.
                result = [route[0]]
                i = 0

                while i < len(route) - 1:
                    j = len(route) - 1

                    while (
                        j > i + 1
                        and not self.edge_free(route[i], route[j])
                    ):
                        j -= 1

                    result.append(route[j])
                    i = j

                return result

        return None

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()

        world = [self.to_world(point) for point in self.path]

        for i, xy in enumerate(world):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(xy[0])
            pose.pose.position.y = float(xy[1])

            if i + 1 < len(world):
                direction = world[i + 1] - xy
                yaw = math.atan2(direction[1], direction[0])
            else:
                # Preserve requested heading through every replan.
                yaw = self.goal_yaw

            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def remaining_path_valid(self, start):
        # Advance only when near the next waypoint and the onward
        # connection is free. Do not skip directly across corners.
        while (
            len(self.path) > 2
            and np.linalg.norm(start - self.path[1]) * self.res
            <= self.waypoint_tolerance
            and self.edge_free(start, self.path[2])
        ):
            self.path = self.path[1:]

        a = self.path[0]
        b = self.path[1]
        segment = b - a
        length_squared = float(np.dot(segment, segment))

        if length_squared > 1e-12:
            fraction = float(np.clip(
                np.dot(start - a, segment) / length_squared,
                0.0,
                1.0,
            ))
        else:
            fraction = 0.0

        projection = a + fraction * segment
        deviation = np.linalg.norm(start - projection) * self.res

        if deviation > self.max_deviation:
            return False

        remaining = [start] + self.path[1:]

        return all(
            self.edge_free(a, b)
            for a, b in zip(remaining[:-1], remaining[1:])
        )

    def update(self):
        if self.grid is None or self.goal is None:
            self.valid_pub.publish(Bool(data=False))
            return

        wall = time.monotonic()
        age = (
            self.get_clock().now() - self.map_stamp
        ).nanoseconds * 1e-9

        fresh = (
            self.ready
            and wall - self.ready_wall < 1.0
            and wall - self.map_wall < 1.0
            and -0.05 <= age < 1.0
        )

        if not fresh:
            self.clear_path()
            self.report("Waiting for fresh costmap")
            return

        if self.goal.header.frame_id != self.frame:
            self.clear_path()
            self.report(
                "Goal frame differs from map; "
                "set RViz Fixed Frame to map"
            )
            return

        try:
            tf = self.buffer.lookup_transform(
                self.frame, "base_link", Time()
            )
        except TransformException:
            self.clear_path()
            self.report("Waiting for map -> base_link TF")
            return

        tf_age = (
            self.get_clock().now()
            - Time.from_msg(tf.header.stamp)
        ).nanoseconds * 1e-9

        if not -0.5 <= tf_age < 1.0:
            self.clear_path()
            self.report("Robot TF is stale")
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        if not all(math.isfinite(v) for v in [t.x, t.y, q.x, q.y, q.z, q.w]):
            self.clear_path()
            self.report("Invalid robot TF")
            return
        qnorm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if qnorm < 1e-9:
            self.clear_path()
            self.report("Invalid robot TF quaternion")
            return
        qx, qy, qz, qw = q.x/qnorm, q.y/qnorm, q.z/qnorm, q.w/qnorm
        robot_yaw = math.atan2(
            2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz)
        )
        start = self.to_grid([t.x, t.y])
        goal = self.to_grid([
            self.goal.pose.position.x,
            self.goal.pose.position.y,
        ])

        if not self.point_free(start) or not self.point_free(goal):
            self.clear_path()
            self.report(
                "Start or goal is blocked/outside map; "
                "choose a free goal"
            )
            return

        distance_to_goal = float(np.linalg.norm(start - goal) * self.res)
        # Enter at 8 cm; retain final rotation through small pose drift.
        # Return to positional tracking only beyond the 12 cm outer bound.
        if distance_to_goal <= self.goal_tolerance:
            self.final_aligning = True
        elif distance_to_goal > self.final_exit_tolerance:
            self.final_aligning = False

        if self.final_aligning:
            error = math.atan2(
                math.sin(self.goal_yaw - robot_yaw),
                math.cos(self.goal_yaw - robot_yaw),
            )
            if abs(error) <= self.yaw_tolerance:
                self.clear_path()
                self.goal = None
                self.report("Goal reached: position and heading")
                return

            # Hand off final rotation even after a path invalidation.
            # The connection must still be collision-free.
            if not self.edge_free(start, goal):
                self.clear_path()
                self.report("Final approach blocked; waiting for clearance")
                return
            if self.path is None:
                self.path = [start.copy(), goal.copy()]
                self.publish_path()
            self.valid_pub.publish(Bool(data=True))
            self.report("Position reached; waiting for final heading")
            return

        if self.path is not None:
            if not self.remaining_path_valid(start):
                self.clear_path()
                self.next_attempt = 0.0
                self.report(
                    "Remaining path blocked or robot off path "
                    "-> replanning"
                )

        if self.path is None and wall >= self.next_attempt:
            self.next_attempt = wall + 1.0
            self.path = self.plan(start, goal)

            if self.path is None:
                self.report("No path found this attempt; retrying")
            else:
                self.publish_path()
                self.report(
                    f"Published path: {len(self.path)} waypoints"
                )

        self.valid_pub.publish(Bool(data=self.path is not None))


def main(args=None):
    rclpy.init(args=args)
    node = Planner()

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