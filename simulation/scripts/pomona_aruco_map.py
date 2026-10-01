import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformException


class ArucoMap(Node):
    def __init__(self):
        super().__init__("pomona_aruco_map")

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pending = None
        self.last_log = 0.0

        self.publisher = self.create_publisher(
            PointStamped, "/pomona/aruco/position_map", 1
        )
        self.create_subscription(
            PointStamped,
            "/pomona/aruco/position_camera",
            self.on_point,
            1,
        )
        self.create_timer(0.05, self.update)

        self.get_logger().info("Waiting for marker position and TF")

    def on_point(self, msg):
        self.pending = msg

    def update(self):
        msg = self.pending
        if msg is None:
            return

        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9

        if age < -0.05 or age > 0.5:
            self.pending = None
            return

        try:
            transform = self.buffer.lookup_transform(
                "map", msg.header.frame_id, stamp
            )
        except TransformException:
            # Retry when the transform for this timestamp arrives.
            return

        q = transform.transform.rotation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-12:
            self.pending = None
            return

        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        px, py, pz = msg.point.x, msg.point.y, msg.point.z
        t = transform.transform.translation

        output = PointStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = "map"

        output.point.x = (
            (1 - 2*y*y - 2*z*z)*px
            + (2*x*y - 2*z*w)*py
            + (2*x*z + 2*y*w)*pz + t.x
        )
        output.point.y = (
            (2*x*y + 2*z*w)*px
            + (1 - 2*x*x - 2*z*z)*py
            + (2*y*z - 2*x*w)*pz + t.y
        )
        output.point.z = (
            (2*x*z - 2*y*w)*px
            + (2*y*z + 2*x*w)*py
            + (1 - 2*x*x - 2*y*y)*pz + t.z
        )

        if not all(math.isfinite(v) for v in (
            output.point.x, output.point.y, output.point.z
        )):
            self.pending = None
            return

        self.publisher.publish(output)
        self.pending = None

        if time.monotonic() - self.last_log >= 2.0:
            self.get_logger().info(
                f"Marker in map: x={output.point.x:.3f}, "
                f"y={output.point.y:.3f}, "
                f"z={output.point.z:.3f} m"
            )
            self.last_log = time.monotonic()


def main():
    rclpy.init()
    node = ArucoMap()
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