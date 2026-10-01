import copy
import math
import time

import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException

from pomona_costmap import PomonaCostmap


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-9:
        raise ValueError("Invalid quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


class LiveCostmap(PomonaCostmap):
    def __init__(self):
        super().__init__()
        self.map_msg = None
        self.map_key = None
        self.live = None
        self.pending_scan = None
        self.last_success_wall = None
        self.last_success_stamp = None
        self.last_warning = 0.0
        self.first_scan = True

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scout_mini/scan",
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.ready_pub = self.create_publisher(
            Bool, "/pomona/costmap_ready", 1
        )
        self.timer = self.create_timer(0.2, self.update)

    def on_map(self, msg):
        # Store the static map. Inflation is performed after merging scans.
        if (
            msg.info.width == 0
            or msg.info.height == 0
            or msg.info.resolution <= 0
            or len(msg.data) != msg.info.width * msg.info.height
        ):
            return

        p = msg.info.origin.position
        q = msg.info.origin.orientation
        key = (
            msg.header.frame_id,
            msg.info.width, msg.info.height, msg.info.resolution,
            p.x, p.y, p.z, q.x, q.y, q.z, q.w,
        )

        if key != self.map_key:
            self.live = np.zeros(
                (msg.info.height, msg.info.width), dtype=bool
            )
            self.map_key = key
            self.last_success_wall = None
            self.last_success_stamp = None

        self.map_msg = msg

    def on_scan(self, msg):
        self.pending_scan = msg

    def cells(self, xyz):
        # Convert map-frame points to grid indices, including origin rotation.
        origin = self.map_msg.info.origin
        offset = np.array([
            origin.position.x,
            origin.position.y,
            origin.position.z,
        ])
        local = (xyz - offset) @ rotation_matrix(origin.orientation)
        ij = np.floor(
            local[:, :2] / self.map_msg.info.resolution
        ).astype(np.int64)

        valid = (
            (ij[:, 0] >= 0)
            & (ij[:, 0] < self.map_msg.info.width)
            & (ij[:, 1] >= 0)
            & (ij[:, 1] < self.map_msg.info.height)
        )
        return ij[valid]

    def process_scan(self, scan, transform):
        rotation = rotation_matrix(transform.rotation)
        translation = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ])
        step = self.map_msg.info.resolution / 3.0
        hit_points = []
        valid_beams = 0

        for index, value in enumerate(scan.ranges):
            # In this Gazebo lidar, +inf represents no return within range.
            if math.isnan(value) or value == -math.inf:
                continue
            if value < scan.range_min:
                continue

            hit = math.isfinite(value) and value < scan.range_max
            distance = min(value, scan.range_max)

            if not math.isfinite(distance) or distance <= 0:
                continue

            valid_beams += 1
            angle = scan.angle_min + index * scan.angle_increment
            direction = rotation @ np.array([
                math.cos(angle), math.sin(angle), 0.0
            ])

            # Clear only the portion observed before the obstacle.
            clear_end = max(
                scan.range_min,
                distance - step if hit else distance,
            )
            samples = np.arange(scan.range_min, clear_end, step)
            if samples.size:
                points = (
                    translation[None, :]
                    + samples[:, None] * direction[None, :]
                )
                ij = self.cells(points)
                self.live[ij[:, 1], ij[:, 0]] = False

            if hit:
                hit_points.append(translation + distance * direction)

        # Mark all endpoints after clearing, so adjacent rays do not
        # erase endpoints belonging to the same scan.
        if hit_points:
            ij = self.cells(np.asarray(hit_points))
            self.live[ij[:, 1], ij[:, 0]] = True

        return valid_beams > 0

    def update(self):
        scan = self.pending_scan
        if scan is not None and self.map_msg is not None:
            stamp = Time.from_msg(scan.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9

            if age < -0.05 or age > 0.5:
                self.pending_scan = None
            else:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.map_msg.header.frame_id,
                        scan.header.frame_id,
                        stamp,
                    )

                    if self.process_scan(scan, tf.transform):
                        merged = copy.deepcopy(self.map_msg)
                        values = np.asarray(
                            merged.data, dtype=np.int16
                        ).reshape(self.live.shape).copy()
                        values[self.live] = 100
                        merged.data = values.ravel().tolist()
                        merged.header.stamp = scan.header.stamp

                        # Reuse inflation from the existing static node.
                        super().on_map(merged)
                        self.last_success_wall = time.monotonic()
                        self.last_success_stamp = stamp

                        if self.first_scan:
                            self.get_logger().info(
                                "Live lidar layer active; "
                                "publishing merged inflated map"
                            )
                            self.first_scan = False

                    self.pending_scan = None

                except TransformException as exc:
                    # Retry this scan on the next timer tick.
                    if time.monotonic() - self.last_warning > 3.0:
                        self.get_logger().warning(
                            f"Waiting for scan-time TF: {exc}"
                        )
                        self.last_warning = time.monotonic()

        ready = False
        if self.last_success_wall is not None:
            sim_age = (
                self.get_clock().now() - self.last_success_stamp
            ).nanoseconds * 1e-9
            ready = (
                time.monotonic() - self.last_success_wall < 1.0
                and -0.05 <= sim_age < 1.0
            )

        self.ready_pub.publish(Bool(data=ready))


def main(args=None):
    rclpy.init(args=args)
    node = LiveCostmap()
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