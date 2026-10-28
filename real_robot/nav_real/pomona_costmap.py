import copy
import math

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from nav_msgs.msg import OccupancyGrid


class PomonaCostmap(Node):
    def __init__(self):
        super().__init__("pomona_costmap")
        cv2.setNumThreads(1)

        self.declare_parameter("robot_radius", 0.43)
        self.declare_parameter("safety_margin", 0.07)

        radius = float(self.get_parameter("robot_radius").value)
        margin = float(self.get_parameter("safety_margin").value)
        if not math.isfinite(radius + margin) or radius <= 0 or margin < 0:
            raise ValueError("Invalid robot_radius or safety_margin")
        self.inflation_radius = radius + margin

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(
            OccupancyGrid, "/pomona/inflated_map", qos
        )
        self.subscription = self.create_subscription(
            OccupancyGrid, "/map", self.on_map, qos
        )
        self.logged_first_map = False

        self.get_logger().info(
            f"Waiting for /map; inflation radius "
            f"{self.inflation_radius:.2f} m"
        )

    def on_map(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        if (
            width == 0
            or height == 0
            or not math.isfinite(resolution)
            or resolution <= 0
            or len(msg.data) != width * height
        ):
            self.get_logger().error("Invalid map dimensions or resolution")
            return

        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            height, width
        )

        # Only known-free cells are traversable.
        # Unknown (-1) and all nonzero cells are blocked.
        free = grid == 0

        # A blocked border represents the area outside the map.
        padded_free = np.pad(
            free, pad_width=1, mode="constant", constant_values=False
        )
        distance = cv2.distanceTransform(
            padded_free.astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE
        )[1:-1, 1:-1].astype(np.float64) * resolution

        # Account conservatively for the size of occupied grid cells.
        cell_allowance = resolution / math.sqrt(2.0)
        blocked = distance <= (
            self.inflation_radius + cell_allowance + 1e-6
        )

        output = OccupancyGrid()
        output.header = copy.deepcopy(msg.header)
        output.info = copy.deepcopy(msg.info)
        output.data = np.where(
            blocked, 100, 0
        ).astype(np.int8).ravel().tolist()

        self.publisher.publish(output)

        if not self.logged_first_map:
            self.get_logger().info(
                f"Published /pomona/inflated_map: "
                f"{width}x{height}, resolution={resolution:.3f} m, "
                f"free cells={np.count_nonzero(~blocked)}"
            )
            self.logged_first_map = True


def main(args=None):
    rclpy.init(args=args)
    node = PomonaCostmap()
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
