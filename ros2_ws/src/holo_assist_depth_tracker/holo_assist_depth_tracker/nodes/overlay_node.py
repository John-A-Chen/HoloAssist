#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import cv2
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image


class OverlayNode(Node):
    """Render merged AprilTag detections onto RGB image stream."""

    def __init__(self) -> None:
        super().__init__("holoassist_overlay")

        self.declare_parameter("input_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/detections_all")
        self.declare_parameter("output_image_topic", "/holoassist/perception/apriltag_overlay")
        self.declare_parameter("detections_timeout_s", 1.0)
        self.declare_parameter("line_thickness_px", 2)
        self.declare_parameter("font_scale", 0.55)

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.output_image_topic = str(self.get_parameter("output_image_topic").value)
        self.detections_timeout_s = max(0.05, float(self.get_parameter("detections_timeout_s").value))
        self.line_thickness_px = max(1, int(self.get_parameter("line_thickness_px").value))
        self.font_scale = max(0.2, float(self.get_parameter("font_scale").value))

        self._bridge = CvBridge()
        self._latest_detections: list = []
        self._latest_detections_stamp: Optional[rclpy.time.Time] = None

        self._image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._det_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._on_detections,
            10,
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._overlay_pub = self.create_publisher(Image, self.output_image_topic, reliable_qos)

        self.get_logger().info(
            "overlay node started image=%s detections=%s out=%s"
            % (self.input_image_topic, self.detections_topic, self.output_image_topic)
        )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self._latest_detections = list(msg.detections)
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        self._latest_detections_stamp = stamp

    def _on_image(self, msg: Image) -> None:
        try:
            image_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"RGB conversion failed: {exc}")
            return

        detections, age_s = self._fresh_detections_with_age()
        if detections is not None:
            self._draw_detections(image_bgr, detections)
            summary = f"tags={len(detections)} age={age_s:.2f}s"
        else:
            summary = "tags=0 age=inf"

        cv2.putText(
            image_bgr,
            summary,
            (12, image_bgr.shape[0] - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            self.font_scale,
            (0, 255, 255),
            self.line_thickness_px,
            cv2.LINE_AA,
        )

        out_msg = self._bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
        out_msg.header = msg.header
        self._overlay_pub.publish(out_msg)

    def _fresh_detections_with_age(self) -> tuple[Optional[list], float]:
        if self._latest_detections_stamp is None:
            return None, float("inf")

        age_s = (self.get_clock().now() - self._latest_detections_stamp).nanoseconds / 1e9
        if age_s > self.detections_timeout_s:
            return None, age_s
        return self._latest_detections, age_s

    def _draw_detections(self, image_bgr, detections: list) -> None:
        line_color = (0, 255, 255)
        arrow_color = (255, 255, 0)
        center_color = (40, 220, 40)

        for det in detections:
            corners = [(int(round(float(p.x))), int(round(float(p.y)))) for p in det.corners]
            if len(corners) == 4:
                for i in range(4):
                    cv2.line(
                        image_bgr,
                        corners[i],
                        corners[(i + 1) % 4],
                        line_color,
                        self.line_thickness_px,
                        cv2.LINE_AA,
                    )
                cv2.arrowedLine(
                    image_bgr,
                    corners[0],
                    corners[1],
                    arrow_color,
                    self.line_thickness_px,
                    cv2.LINE_AA,
                    tipLength=0.22,
                )

            cx = int(round(float(det.centre.x)))
            cy = int(round(float(det.centre.y)))
            cv2.circle(image_bgr, (cx, cy), 4, center_color, -1, cv2.LINE_AA)
            cv2.putText(
                image_bgr,
                f"id={int(det.id)}",
                (cx + 8, max(16, cy - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.font_scale,
                line_color,
                self.line_thickness_px,
                cv2.LINE_AA,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OverlayNode()
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
