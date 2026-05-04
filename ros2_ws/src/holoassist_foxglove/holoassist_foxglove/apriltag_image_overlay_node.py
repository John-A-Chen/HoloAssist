#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image


class AprilTagImageOverlayNode(Node):
    """Render AprilTag detections as an RGB overlay image for quick visual debug."""

    def __init__(self) -> None:
        super().__init__("apriltag_image_overlay")

        self.declare_parameter("input_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("output_image_topic", "/holoassist/apriltag/debug_image")
        self.declare_parameter("detections_timeout_s", 1.0)
        self.declare_parameter("line_thickness_px", 2)
        self.declare_parameter("font_scale", 0.55)
        self.declare_parameter("workspace_outline_enabled", True)
        self.declare_parameter("workspace_left_tag_id", 0)
        self.declare_parameter("workspace_right_tag_id", 1)
        self.declare_parameter("workspace_width_m", 0.70)
        self.declare_parameter("workspace_depth_m", 0.50)
        self.declare_parameter("workspace_tag_size_m", 0.032)
        self.declare_parameter("workspace_left_corner_index_offset", 0)
        self.declare_parameter("workspace_right_corner_index_offset", 0)
        self.declare_parameter("workspace_left_corner_reverse", False)
        self.declare_parameter("workspace_right_corner_reverse", False)
        self.declare_parameter("workspace_outline_max_reproj_error_px", 24.0)
        self.declare_parameter("workspace_outline_thickness_px", 3)

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.output_image_topic = str(self.get_parameter("output_image_topic").value)
        self.detections_timeout_s = float(
            self.get_parameter("detections_timeout_s").value
        )
        self.line_thickness_px = int(self.get_parameter("line_thickness_px").value)
        self.font_scale = float(self.get_parameter("font_scale").value)
        self.workspace_outline_enabled = bool(
            self.get_parameter("workspace_outline_enabled").value
        )
        self.workspace_left_tag_id = int(
            self.get_parameter("workspace_left_tag_id").value
        )
        self.workspace_right_tag_id = int(
            self.get_parameter("workspace_right_tag_id").value
        )
        self.workspace_width_m = float(self.get_parameter("workspace_width_m").value)
        self.workspace_depth_m = float(self.get_parameter("workspace_depth_m").value)
        self.workspace_tag_size_m = float(
            self.get_parameter("workspace_tag_size_m").value
        )
        self.workspace_left_corner_index_offset = int(
            self.get_parameter("workspace_left_corner_index_offset").value
        )
        self.workspace_right_corner_index_offset = int(
            self.get_parameter("workspace_right_corner_index_offset").value
        )
        self.workspace_left_corner_reverse = bool(
            self.get_parameter("workspace_left_corner_reverse").value
        )
        self.workspace_right_corner_reverse = bool(
            self.get_parameter("workspace_right_corner_reverse").value
        )
        self.workspace_outline_max_reproj_error_px = float(
            self.get_parameter("workspace_outline_max_reproj_error_px").value
        )
        self.workspace_outline_thickness_px = int(
            self.get_parameter("workspace_outline_thickness_px").value
        )

        self.detections_timeout_s = max(0.05, self.detections_timeout_s)
        self.line_thickness_px = max(1, self.line_thickness_px)
        self.font_scale = max(0.2, self.font_scale)
        self.workspace_width_m = max(0.10, self.workspace_width_m)
        self.workspace_depth_m = max(0.10, self.workspace_depth_m)
        self.workspace_tag_size_m = max(0.01, self.workspace_tag_size_m)
        self._warn_expected_apriltag_size("workspace_tag_size_m", self.workspace_tag_size_m)
        self.workspace_outline_max_reproj_error_px = max(
            1.0, self.workspace_outline_max_reproj_error_px
        )
        self.workspace_outline_thickness_px = max(1, self.workspace_outline_thickness_px)

        self.bridge = CvBridge()
        self.latest_detections: list = []
        self.latest_detections_stamp: Optional[rclpy.time.Time] = None

        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._on_detections,
            10,
        )

        reliable_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.overlay_pub = self.create_publisher(
            Image,
            self.output_image_topic,
            reliable_pub_qos,
        )

        self.get_logger().info(
            "apriltag overlay started image=%s detections=%s out=%s workspace_outline=%s ids=(%d,%d)"
            % (
                self.input_image_topic,
                self.detections_topic,
                self.output_image_topic,
                self.workspace_outline_enabled,
                self.workspace_left_tag_id,
                self.workspace_right_tag_id,
            )
        )

    def _warn_expected_apriltag_size(self, name: str, value_m: float) -> None:
        expected = 0.032
        if abs(value_m - expected) > 1e-6:
            self.get_logger().warn(
                "%s=%.6f m but all current AprilTags are expected to be %.3f m."
                % (name, value_m, expected)
            )
        if value_m > 0.05:
            self.get_logger().warn(
                "%s=%.6f m is unusually large; overlay board mapping may be wrong."
                % (name, value_m)
            )
        if value_m >= 1.0:
            self.get_logger().warn(
                "%s=%.6f looks like millimeters passed as meters (e.g. 32/40)."
                % (name, value_m)
            )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self.latest_detections = list(msg.detections)
        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds == 0:
                stamp = self.get_clock().now()
        except Exception:
            stamp = self.get_clock().now()
        self.latest_detections_stamp = stamp

    def _on_image(self, msg: Image) -> None:
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"RGB conversion failed: {exc}")
            return

        detections = self._fresh_detections()
        if detections is not None:
            self._draw_detections(image_bgr, detections)
        else:
            cv2.putText(
                image_bgr,
                "AprilTag: no recent detections",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.font_scale,
                (0, 180, 255),
                self.line_thickness_px,
                cv2.LINE_AA,
            )

        out_msg = self.bridge.cv2_to_imgmsg(image_bgr, encoding="bgr8")
        out_msg.header = msg.header
        self.overlay_pub.publish(out_msg)

    def _fresh_detections(self) -> Optional[list]:
        if self.latest_detections_stamp is None:
            return None
        age_s = (self.get_clock().now() - self.latest_detections_stamp).nanoseconds / 1e9
        if age_s > self.detections_timeout_s:
            return None
        return self.latest_detections

    def _draw_detections(self, image_bgr, detections: list) -> None:
        line_color = (0, 255, 255)
        edge_arrow_color = (255, 255, 0)
        center_color = (40, 220, 40)

        for det in detections:
            corners = []
            for p in det.corners:
                corners.append((int(round(float(p.x))), int(round(float(p.y)))))

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
                    edge_arrow_color,
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

        self._draw_workspace_outline(image_bgr, detections)

        cv2.putText(
            image_bgr,
            f"tags={len(detections)}",
            (12, image_bgr.shape[0] - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            self.font_scale,
            line_color,
            self.line_thickness_px,
            cv2.LINE_AA,
        )

    def _find_detection(self, detections: list, tag_id: int):
        for det in detections:
            if int(det.id) == int(tag_id):
                return det
        return None

    def _corners_as_array(self, det) -> Optional[np.ndarray]:
        if len(det.corners) != 4:
            return None
        corners = np.array(
            [[float(p.x), float(p.y)] for p in det.corners], dtype=np.float32
        )
        if not np.isfinite(corners).all():
            return None
        return corners

    def _mapped_corners(
        self, corners: np.ndarray, index_offset: int, reverse: bool
    ) -> np.ndarray:
        mapped = corners[::-1].copy() if reverse else corners.copy()
        shift = int(index_offset) % 4
        if shift:
            mapped = np.roll(mapped, -shift, axis=0)
        return mapped

    def _workspace_homography(
        self, left_det, right_det
    ) -> Tuple[Optional[np.ndarray], float]:
        w = float(self.workspace_width_m)
        s = float(self.workspace_tag_size_m)
        right_x0 = w - s

        board_points = []
        image_points = []
        if left_det is not None:
            left_raw = self._corners_as_array(left_det)
            if left_raw is not None:
                left = self._mapped_corners(
                    left_raw,
                    self.workspace_left_corner_index_offset,
                    self.workspace_left_corner_reverse,
                )
                board_points.extend(
                    [[0.0, 0.0], [s, 0.0], [s, s], [0.0, s]]
                )
                image_points.extend(left.tolist())

        if right_det is not None:
            right_raw = self._corners_as_array(right_det)
            if right_raw is not None:
                right = self._mapped_corners(
                    right_raw,
                    self.workspace_right_corner_index_offset,
                    self.workspace_right_corner_reverse,
                )
                board_points.extend(
                    [[right_x0, 0.0], [w, 0.0], [w, s], [right_x0, s]]
                )
                image_points.extend(right.tolist())

        if len(board_points) < 4 or len(image_points) < 4:
            return None, float("inf")

        board_pts = np.asarray(board_points, dtype=np.float32)
        image_pts = np.asarray(image_points, dtype=np.float32)

        homography, _ = cv2.findHomography(board_pts, image_pts, 0)
        if homography is None:
            return None, float("inf")

        projected = cv2.perspectiveTransform(
            board_pts.reshape(1, -1, 2), homography
        ).reshape(-1, 2)
        reproj_err = float(np.mean(np.linalg.norm(projected - image_pts, axis=1)))
        return homography, reproj_err

    def _draw_workspace_outline(self, image_bgr, detections: list) -> None:
        if not self.workspace_outline_enabled:
            return

        left_det = self._find_detection(detections, self.workspace_left_tag_id)
        right_det = self._find_detection(detections, self.workspace_right_tag_id)
        if left_det is None and right_det is None:
            return

        homography, reproj_err = self._workspace_homography(left_det, right_det)
        if homography is None or reproj_err > self.workspace_outline_max_reproj_error_px:
            return

        w = float(self.workspace_width_m)
        d = float(self.workspace_depth_m)
        board_outline = np.array(
            [[[0.0, 0.0], [w, 0.0], [w, d], [0.0, d]]], dtype=np.float32
        )
        front_edge = np.array([[[0.0, 0.0], [w, 0.0]]], dtype=np.float32)

        outline_px = cv2.perspectiveTransform(board_outline, homography).reshape(-1, 2)
        front_px = cv2.perspectiveTransform(front_edge, homography).reshape(-1, 2)
        if not np.isfinite(outline_px).all() or not np.isfinite(front_px).all():
            return

        outline_int = np.round(outline_px).astype(np.int32).reshape((-1, 1, 2))
        front_int = np.round(front_px).astype(np.int32)

        cv2.polylines(
            image_bgr,
            [outline_int],
            isClosed=True,
            color=(255, 0, 255),
            thickness=self.workspace_outline_thickness_px,
            lineType=cv2.LINE_AA,
        )
        cv2.line(
            image_bgr,
            (int(front_int[0][0]), int(front_int[0][1])),
            (int(front_int[1][0]), int(front_int[1][1])),
            (0, 140, 255),
            max(self.workspace_outline_thickness_px + 1, self.line_thickness_px),
            cv2.LINE_AA,
        )

        corner_labels = ("FL", "FR", "BR", "BL")
        for idx, (x_f, y_f) in enumerate(outline_px):
            x = int(round(float(x_f)))
            y = int(round(float(y_f)))
            cv2.circle(image_bgr, (x, y), 4, (255, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(
                image_bgr,
                corner_labels[idx],
                (x + 6, max(16, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.font_scale * 0.9,
                (255, 0, 255),
                max(1, self.line_thickness_px),
                cv2.LINE_AA,
            )

        cv2.putText(
            image_bgr,
            f"ws={self.workspace_width_m:.3f}x{self.workspace_depth_m:.3f}m err={reproj_err:.1f}px",
            (12, max(24, image_bgr.shape[0] - 42)),
            cv2.FONT_HERSHEY_SIMPLEX,
            self.font_scale * 0.9,
            (255, 0, 255),
            max(1, self.line_thickness_px),
            cv2.LINE_AA,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagImageOverlayNode()
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
