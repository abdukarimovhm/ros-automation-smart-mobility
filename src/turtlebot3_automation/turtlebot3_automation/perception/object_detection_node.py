"""YOLOv8-powered object detection node publishing ROS 2 friendly outputs."""

from __future__ import annotations

import time
from pathlib import Path
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

from ..utils.logging import configure_logging

try:  # pragma: no cover - optional dependency
    from cv_bridge import CvBridge
except ImportError as exc:  # pragma: no cover - runtime guard
    raise RuntimeError("cv_bridge is required for object detection") from exc

try:  # pragma: no cover - optional dependency
    from ultralytics import YOLO
except ImportError as exc:  # pragma: no cover - runtime guard
    raise RuntimeError("ultralytics package is required; install via pip") from exc


class ObjectDetectionNode(Node):
    """Subscribe to camera frames, run YOLO, and broadcast detections."""

    def __init__(self) -> None:
        super().__init__("object_detection_node")

        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.4)
        self.declare_parameter("class_filter", [])
        self.declare_parameter("inference_period", 0.2)
        self.declare_parameter("marker_namespace", "yolo_detections")

        self._logger = configure_logging(self.get_name())
        self._bridge = CvBridge()
        self._last_inference_ts: float = 0.0

        camera_topic = self.get_parameter("camera_topic").value
        model_path = Path(self.get_parameter("model_path").value).expanduser()
        if not model_path.exists() and model_path.name == "yolov8n.pt":
            self._logger.info("Downloading default YOLOv8n model to %s", model_path)
        self._model = YOLO(str(model_path))
        self._class_names = self._model.model.names
        self._class_filter = set(self.get_parameter("class_filter").value or [])
        self._inference_period = float(self.get_parameter("inference_period").value)

        self._det_pub = self.create_publisher(Detection2DArray, "turtlebot3/perception/detections", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "turtlebot3/perception/markers", 10)
        self._label_pub = self.create_publisher(String, "turtlebot3/perception/labels", 10)

        self._image_sub = self.create_subscription(Image, camera_topic, self._on_image, 10)
        self._logger.info("Object detection node ready (model=%s)", model_path)

    def _on_image(self, msg: Image) -> None:
        now = time.time()
        if now - self._last_inference_ts < self._inference_period:
            return
        self._last_inference_ts = now

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self._model(frame, verbose=False)[0]

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header
        markers = MarkerArray()
        labels: List[str] = []

        for idx, box in enumerate(results.boxes):
            conf = float(box.conf.cpu().item())
            if conf < float(self.get_parameter("confidence_threshold").value):
                continue
            class_id = int(box.cls.cpu().item())
            label = self._class_names.get(class_id, str(class_id))
            if self._class_filter and label not in self._class_filter:
                continue

            detection = Detection2D()
            detection.header = msg.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = str(class_id)
            hypothesis.score = conf
            detection.results.append(hypothesis)

            xywh = box.xywh[0].cpu().numpy()
            bbox = BoundingBox2D()
            bbox.center.x = float(xywh[0])
            bbox.center.y = float(xywh[1])
            bbox.size_x = float(xywh[2])
            bbox.size_y = float(xywh[3])
            detection.bbox = bbox

            detections_msg.detections.append(detection)
            labels.append(f"{label}:{conf:.2f}")
            markers.markers.append(
                self._make_marker(
                    idx,
                    detection,
                    text=label,
                    confidence=conf,
                    color=self._color_for_class(class_id),
                )
            )

        if detections_msg.detections:
            self._det_pub.publish(detections_msg)
            self._marker_pub.publish(markers)
            label_msg = String()
            label_msg.data = ", ".join(labels)
            self._label_pub.publish(label_msg)

    def _make_marker(self, marker_id: int, detection: Detection2D, text: str, confidence: float, color: tuple) -> Marker:
        marker = Marker()
        marker.header = detection.header
        marker.ns = self.get_parameter("marker_namespace").value
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.9
        marker.pose.position.x = detection.bbox.center.x / 100.0
        marker.pose.position.y = detection.bbox.center.y / 100.0
        marker.pose.position.z = 1.0
        marker.text = f"{text} ({confidence:.2f})"
        return marker

    @staticmethod
    def _color_for_class(class_id: int) -> tuple:
        golden_ratio_conjugate = 0.61803398875
        hue = (class_id * golden_ratio_conjugate) % 1.0
        return ObjectDetectionNode._hsv_to_rgb(hue, 0.7, 0.95)

    @staticmethod
    def _hsv_to_rgb(h: float, s: float, v: float) -> tuple:
        i = int(h * 6.0)
        f = h * 6.0 - i
        p = v * (1.0 - s)
        q = v * (1.0 - f * s)
        t = v * (1.0 - (1.0 - f) * s)
        i = i % 6
        if i == 0:
            return v, t, p
        if i == 1:
            return q, v, p
        if i == 2:
            return p, v, t
        if i == 3:
            return p, q, v
        if i == 4:
            return t, p, v
        return v, p, q


def main() -> None:
    rclpy.init()
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        node.get_logger().info("Object detection node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
