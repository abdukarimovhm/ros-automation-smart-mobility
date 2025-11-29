"""Custom feature: follow a QR code using computer vision."""

from __future__ import annotations

import cv2
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

from ..utils.logging import configure_logging

try:  # pragma: no cover - runtime dependency
    from cv_bridge import CvBridge
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("cv_bridge is required for QR code following") from exc


class QRFollowNode(Node):
    """Drive the robot toward a target QR code label."""

    def __init__(self) -> None:
        super().__init__("qr_follow_node")
        self.declare_parameter("target", "FOLLOW_ME")
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.12)
        self.declare_parameter("angular_gain", 0.0025)
        self.declare_parameter("distance_gain", 0.00025)
        self.declare_parameter("max_angular", 0.6)
        self.declare_parameter("max_linear", 0.18)

        self._logger = configure_logging(self.get_name())
        self._bridge = CvBridge()
        self._detector = cv2.QRCodeDetector()

        self._cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )
        self._image_sub = self.create_subscription(
            Image,
            self.get_parameter("camera_topic").value,
            self._on_image,
            10,
        )

        self._last_detection = None
        self._logger.info("QR follow node initialized. Target=%s", self.get_parameter("target").value)

    def _on_image(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        data, points, _ = self._detector.detectAndDecode(frame)
        twist = Twist()

        if data:
            self._last_detection = data
            target_label = self.get_parameter("target").value
            if target_label and data != target_label:
                self.get_logger().debug("Ignoring QR label %s", data)
                self._cmd_pub.publish(twist)
                return

            if points is not None and points.any():
                pts = points[0]
                cx = pts[:, 0].mean()
                cy = pts[:, 1].mean()
                width = frame.shape[1]
                height = frame.shape[0]
                error_x = (cx - width / 2.0)
                error_y = height - cy

                angular = -error_x * float(self.get_parameter("angular_gain").value)
                linear = min(
                    float(self.get_parameter("linear_speed").value),
                    error_y * float(self.get_parameter("distance_gain").value),
                )
                twist.angular.z = max(
                    -float(self.get_parameter("max_angular").value),
                    min(float(self.get_parameter("max_angular").value), angular),
                )
                twist.linear.x = max(
                    0.0,
                    min(float(self.get_parameter("max_linear").value), linear),
                )
                self._logger.debug(
                    "QR %s -> cmd linear=%.3f angular=%.3f", data, twist.linear.x, twist.angular.z
                )
        else:
            self._last_detection = None

        self._cmd_pub.publish(twist)


def main() -> None:
    rclpy.init()
    node = QRFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        node.get_logger().info("QR follow node interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
