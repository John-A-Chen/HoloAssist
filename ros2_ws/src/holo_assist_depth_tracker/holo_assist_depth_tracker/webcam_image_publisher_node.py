#!/usr/bin/env python3

from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class WebcamImagePublisherNode(Node):
    """Publish laptop webcam frames as sensor_msgs/Image for RGB fallback demos."""

    def __init__(self) -> None:
        super().__init__("holo_assist_webcam_image_publisher")

        self.declare_parameter("device_index", 0)
        self.declare_parameter("image_topic", "/holo_assist/webcam/image_raw")
        self.declare_parameter("frame_id", "webcam_frame")
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)

        self.device_index = int(self.get_parameter("device_index").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.fps = float(self.get_parameter("fps").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)

        if self.fps <= 0.0:
            self.get_logger().warn("fps must be > 0. Using 15.0.")
            self.fps = 15.0
        if self.width <= 0:
            self.width = 640
        if self.height <= 0:
            self.height = 480

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.frame_count = 0
        self.capture: Optional[cv2.VideoCapture] = cv2.VideoCapture(self.device_index)

        if self.capture is None or not self.capture.isOpened():
            raise RuntimeError(
                f"Failed to open webcam device_index={self.device_index}. "
                "Check camera permissions and device availability."
            )

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        self.capture.set(cv2.CAP_PROP_FPS, float(self.fps))

        period_s = 1.0 / self.fps
        self.timer = self.create_timer(period_s, self._publish_frame_cb)
        self.status_timer = self.create_timer(2.0, self._status_cb)

        self.get_logger().info(
            f"Webcam publisher started. topic={self.image_topic}, device_index={self.device_index}, "
            f"target={self.width}x{self.height}@{self.fps:.1f}Hz"
        )

    def _publish_frame_cb(self) -> None:
        if self.capture is None:
            return

        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.get_logger().warn("Failed to read webcam frame.")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)
        self.frame_count += 1

    def _status_cb(self) -> None:
        self.get_logger().info(
            f"status: webcam_frames_published={self.frame_count}, topic={self.image_topic}"
        )

    def destroy_node(self) -> bool:
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebcamImagePublisherNode()
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
