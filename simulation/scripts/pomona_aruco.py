import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("pomona_aruco")

        self.bridge = CvBridge()
        self.camera_info = None
        self.last_log = 0.0

        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        # Marker side = 0.30 m, including black border,
        # excluding the surrounding white margin.
        half = 0.30 / 2.0
        self.object_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)

        self.image_pub = self.create_publisher(
            Image, "/pomona/aruco/image", 1
        )
        self.point_pub = self.create_publisher(
            PointStamped, "/pomona/aruco/position_camera", 1
        )

        self.create_subscription(
            CameraInfo,
            "/scout_mini/rgb_cam/camera_info",
            self.on_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/scout_mini/rgb_cam/image_raw",
            self.on_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Waiting for camera; target: DICT_4X4_50, ID 0"
        )

    def on_info(self, msg):
        self.camera_info = msg

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as error:
            self.get_logger().error(str(error))
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        status = "Target ID 0 not detected"

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

            for index, marker_id in enumerate(ids.flatten()):
                if int(marker_id) != 0:
                    continue

                status = "ID 0 detected; waiting for CameraInfo"
                info = self.camera_info
                if info is None:
                    break

                if (
                    info.width != msg.width
                    or info.height != msg.height
                    or info.header.frame_id != msg.header.frame_id
                ):
                    status = "Image / CameraInfo mismatch"
                    break

                matrix = np.array(info.k, dtype=np.float64).reshape(3, 3)
                distortion = np.array(info.d, dtype=np.float64)
                if distortion.size == 0:
                    distortion = np.zeros(5, dtype=np.float64)

                if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
                    status = "Invalid camera calibration"
                    break

                image_points = corners[index].reshape(4, 2).astype(
                    np.float64
                )

                ok, rotation, translation = cv2.solvePnP(
                    self.object_points,
                    image_points,
                    matrix,
                    distortion,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )

                if not ok or not np.isfinite(translation).all():
                    status = "Pose estimation failed"
                    break

                x, y, z = translation.reshape(3)
                if z <= 0:
                    status = "Rejected pose behind camera"
                    break

                projected, _ = cv2.projectPoints(
                    self.object_points,
                    rotation,
                    translation,
                    matrix,
                    distortion,
                )
                error = np.sqrt(np.mean(np.sum(
                    (projected.reshape(4, 2) - image_points) ** 2,
                    axis=1,
                )))
                if error > 3.0:
                    status = f"Rejected pose: error {error:.1f} px"
                    break

                point = PointStamped()
                point.header = msg.header
                point.point.x = float(x)
                point.point.y = float(y)
                point.point.z = float(z)
                self.point_pub.publish(point)

                cv2.drawFrameAxes(
                    image, matrix, distortion,
                    rotation, translation, 0.10
                )
                status = (
                    f"ID 0: x={x:.2f}, y={y:.2f}, z={z:.2f} m"
                )
                break

        cv2.putText(
            image, status, (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (0, 255, 0), 1, cv2.LINE_AA,
        )

        output = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        output.header = msg.header
        self.image_pub.publish(output)

        if time.monotonic() - self.last_log >= 2.0:
            self.get_logger().info(status)
            self.last_log = time.monotonic()


def main():
    rclpy.init()
    node = ArucoDetector()
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