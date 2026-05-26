#!/usr/bin/env python3

from collections import deque
import re
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from holo_assist_depth_tracker.utils.math3d import rotation_matrix_from_quaternion

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    HAVE_APRILTAG_MSGS = True
except Exception:
    AprilTagDetectionArray = None
    HAVE_APRILTAG_MSGS = False

_CUBE_COLORS = [
    (80,  80,  255),
    (50,  200,  50),
    (255, 140,  40),
    (0,   210, 210),
]
_COLOR_CALIB   = (100, 255, 120)
_COLOR_UNKNOWN = (140, 140, 140)
_COLOR_BIN     = (198, 121, 255)
_COLOR_NOT_IN_BIN = (60, 60, 255)

class DepthTrackerNode(Node):
    """RGB debug viewer: AprilTag outlines + oriented cube wireframes from cube_pose topics."""

    def __init__(self) -> None:
        super().__init__("holo_assist_depth_tracker")

        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("rgb_topic_fallback", "/camera/color/image_raw")
        self.declare_parameter("apriltag_topic", "/detections_all")
        self.declare_parameter("apriltag_topic_fallback", "/detections")
        self.declare_parameter("apriltag_timeout_s", 1.0)
        self.declare_parameter("calib_tag_id", 1)
        self.declare_parameter("cube_pose_topic_prefix", "/holoassist/perception")
        self.declare_parameter("cube_size_m", 0.040)
        self.declare_parameter("april_cube_1_tag_ids", list(range(10, 16)))
        self.declare_parameter("april_cube_2_tag_ids", list(range(16, 22)))
        self.declare_parameter("april_cube_3_tag_ids", list(range(22, 28)))
        self.declare_parameter("april_cube_4_tag_ids", list(range(28, 34)))

        self.rgb_topic               = str(self.get_parameter("rgb_topic").value)
        self.rgb_topic_fallback      = str(self.get_parameter("rgb_topic_fallback").value)
        self.apriltag_topic          = str(self.get_parameter("apriltag_topic").value)
        self.apriltag_topic_fallback = str(self.get_parameter("apriltag_topic_fallback").value)
        self.apriltag_timeout_s      = float(self.get_parameter("apriltag_timeout_s").value)
        self.calib_tag_id            = int(self.get_parameter("calib_tag_id").value)
        self.cube_pose_topic_prefix  = str(self.get_parameter("cube_pose_topic_prefix").value).rstrip("/")
        self.cube_size_m             = max(0.001, float(self.get_parameter("cube_size_m").value))

        cube_groups = [
            [int(v) for v in self.get_parameter("april_cube_1_tag_ids").value],
            [int(v) for v in self.get_parameter("april_cube_2_tag_ids").value],
            [int(v) for v in self.get_parameter("april_cube_3_tag_ids").value],
            [int(v) for v in self.get_parameter("april_cube_4_tag_ids").value],
        ]
        self._cube_groups = cube_groups
        self._tag_to_cube: dict[int, int] = {
            tag_id: cube_idx
            for cube_idx, group in enumerate(cube_groups)
            for tag_id in group
        }

        self._bridge = CvBridge()
        self._latest_detections: list = []
        self._latest_det_stamp: Optional[rclpy.time.Time] = None
        self._frame_times: deque = deque(maxlen=30)

        # TF buffer — apriltag_ros publishes tag36h11:N frames under camera_color_optical_frame
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._camera_info: Optional[CameraInfo] = None

        self._cube_pose: dict[int, Optional[PoseStamped]] = {idx: None for idx in range(4)}
        self._cube_bin: dict[int, str] = {idx: "" for idx in range(4)}

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._rgb_sub = self.create_subscription(
            Image, self.rgb_topic, self._on_rgb, qos_profile_sensor_data
        )
        if self.rgb_topic_fallback and self.rgb_topic_fallback != self.rgb_topic:
            self.create_subscription(
                Image, self.rgb_topic_fallback, self._on_rgb, qos_profile_sensor_data
            )

        if HAVE_APRILTAG_MSGS and AprilTagDetectionArray is not None:
            self.create_subscription(
                AprilTagDetectionArray, self.apriltag_topic, self._on_detections, 10
            )
            if self.apriltag_topic_fallback and self.apriltag_topic_fallback != self.apriltag_topic:
                self.create_subscription(
                    AprilTagDetectionArray, self.apriltag_topic_fallback, self._on_detections, 10
                )
        else:
            self.get_logger().warn("apriltag_msgs unavailable — tag overlay disabled")

        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self._on_camera_info,
            1,
        )
        for idx in range(4):
            topic = f"{self.cube_pose_topic_prefix}/april_cube_{idx + 1}_pose"
            self.create_subscription(
                PoseStamped,
                topic,
                self._make_cube_pose_cb(idx),
                10,
            )
            bin_topic = f"{self.cube_pose_topic_prefix}/april_cube_{idx + 1}_bin_check"
            self.create_subscription(
                String,
                bin_topic,
                self._make_cube_bin_cb(idx),
                10,
            )

        self._debug_pub = self.create_publisher(
            Image, "/holo_assist_depth_tracker/debug_image", reliable_qos
        )

        self.get_logger().info(
            f"depth_tracker started rgb={self.rgb_topic} "
            f"detections={self.apriltag_topic} "
            f"calib_id={self.calib_tag_id} cubes={cube_groups} "
            f"cube_pose_prefix={self.cube_pose_topic_prefix} cube_size_m={self.cube_size_m:.3f} "
            f"apriltag_timeout_s={self.apriltag_timeout_s:.2f}"
        )

    # ── callbacks ────────────────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_detections(self, msg) -> None:
        self._latest_detections = list(msg.detections)
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        self._latest_det_stamp = stamp

    def _make_cube_pose_cb(self, cube_idx: int):
        def _cb(msg: PoseStamped) -> None:
            self._cube_pose[cube_idx] = msg

        return _cb

    def _make_cube_bin_cb(self, cube_idx: int):
        def _cb(msg: String) -> None:
            raw = str(msg.data)
            sorted_match = re.search(r"\bsorted=(true|false)", raw, re.IGNORECASE)
            bin_match = re.search(r"\bbin=([^\s]+)", raw)
            if sorted_match and sorted_match.group(1).lower() == "true" and bin_match:
                self._cube_bin[cube_idx] = bin_match.group(1).replace("_", " ")
            elif sorted_match and sorted_match.group(1).lower() == "false":
                self._cube_bin[cube_idx] = "NOT IN BIN"
            else:
                self._cube_bin[cube_idx] = ""

        return _cb

    def _on_rgb(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(f"RGB conversion failed: {exc}")
            return

        now = self.get_clock().now()
        self._frame_times.append(now.nanoseconds)
        canvas = bgr.copy()

        det_age_s = float("inf")
        if self._latest_det_stamp is not None:
            det_age_s = (now - self._latest_det_stamp).nanoseconds / 1e9

        if det_age_s <= self.apriltag_timeout_s:
            self._draw_tags(canvas, self._latest_detections)
        cam_frame = str(msg.header.frame_id).strip()
        if not cam_frame and self._camera_info is not None:
            cam_frame = str(self._camera_info.header.frame_id).strip()
        if not cam_frame:
            cam_frame = "camera_color_optical_frame"
        self._draw_cube_wireframes(canvas, cam_frame)
        self._draw_info_panel(canvas, det_age_s, now)

        out = self._bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
        out.header = msg.header
        self._debug_pub.publish(out)

    # ── drawing ───────────────────────────────────────────────────────────────

    def _tag_color(self, tag_id: int) -> tuple:
        if tag_id == self.calib_tag_id:
            return _COLOR_CALIB
        cube_idx = self._tag_to_cube.get(tag_id)
        return _CUBE_COLORS[cube_idx] if cube_idx is not None else _COLOR_UNKNOWN

    def _draw_tags(self, canvas: np.ndarray, detections: list) -> None:
        for det in detections:
            tag_id = int(det.id)
            color = self._tag_color(tag_id)
            corners = [(int(round(float(p.x))), int(round(float(p.y)))) for p in det.corners]
            if len(corners) == 4:
                for i in range(4):
                    cv2.line(canvas, corners[i], corners[(i + 1) % 4], color, 2, cv2.LINE_AA)
                cv2.arrowedLine(canvas, corners[0], corners[1], (255, 255, 0), 2,
                                cv2.LINE_AA, tipLength=0.22)
            cx = int(round(float(det.centre.x)))
            cy = int(round(float(det.centre.y)))
            cv2.circle(canvas, (cx, cy), 4, color, -1, cv2.LINE_AA)
            cv2.putText(canvas, f"id={tag_id}", (cx + 8, max(16, cy - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 2, cv2.LINE_AA)

    def _draw_cube_wireframes(self, canvas: np.ndarray, camera_frame: str) -> None:
        if self._camera_info is None:
            return

        k = self._camera_info.k
        fx, fy, ppx, ppy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
        half = 0.5 * self.cube_size_m
        now_ns = self.get_clock().now().nanoseconds

        local_corners = np.asarray(
            [
                [-half, -half, -half],
                [half, -half, -half],
                [half, half, -half],
                [-half, half, -half],
                [-half, -half, half],
                [half, -half, half],
                [half, half, half],
                [-half, half, half],
            ],
            dtype=np.float64,
        )
        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )

        for cube_idx in range(4):
            pose_msg = self._cube_pose.get(cube_idx)
            if pose_msg is None:
                continue
            stamp_ns = int(pose_msg.header.stamp.sec) * 1_000_000_000 + int(pose_msg.header.stamp.nanosec)
            if stamp_ns > 0:
                age_s = (now_ns - stamp_ns) / 1e9
                if age_s > self.apriltag_timeout_s:
                    continue

            pose_cam = self._cube_pose_in_camera_frame(pose_msg, camera_frame)
            if pose_cam is None:
                continue
            center_cam, rot_cam = pose_cam

            points_cam = center_cam[None, :] + (rot_cam @ local_corners.T).T
            if np.any(points_cam[:, 2] <= 0.03):
                continue

            uv = np.zeros((8, 2), dtype=np.int32)
            for i in range(8):
                x_c, y_c, z_c = points_cam[i]
                u = int(round(fx * x_c / z_c + ppx))
                v = int(round(fy * y_c / z_c + ppy))
                uv[i, 0] = u
                uv[i, 1] = v

            color = _CUBE_COLORS[cube_idx]
            for a, b in edges:
                cv2.line(
                    canvas,
                    (int(uv[a, 0]), int(uv[a, 1])),
                    (int(uv[b, 0]), int(uv[b, 1])),
                    color,
                    2,
                    cv2.LINE_AA,
                )

            label_pos = uv[6]
            cv2.putText(
                canvas,
                f"Cube {cube_idx + 1}",
                (int(label_pos[0]) + 4, int(label_pos[1]) - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

    def _cube_pose_in_camera_frame(
        self,
        pose_msg: PoseStamped,
        camera_frame: str,
    ) -> Optional[tuple[np.ndarray, np.ndarray]]:
        src_frame = str(pose_msg.header.frame_id).strip()
        if not src_frame:
            return None

        p_s = np.array(
            [
                float(pose_msg.pose.position.x),
                float(pose_msg.pose.position.y),
                float(pose_msg.pose.position.z),
            ],
            dtype=np.float64,
        )
        q_s = pose_msg.pose.orientation
        r_s_cube = rotation_matrix_from_quaternion(
            float(q_s.x), float(q_s.y), float(q_s.z), float(q_s.w)
        )

        if src_frame == camera_frame:
            return p_s, r_s_cube

        try:
            tf = self._tf_buffer.lookup_transform(
                camera_frame,
                src_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.01),
            )
        except TransformException:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        r_c_s = rotation_matrix_from_quaternion(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )
        t_c_s = np.array([float(t.x), float(t.y), float(t.z)], dtype=np.float64)

        p_c = r_c_s @ p_s + t_c_s
        r_c_cube = r_c_s @ r_s_cube
        return p_c, r_c_cube

    def _draw_info_panel(self, canvas: np.ndarray, det_age_s: float,
                         now: rclpy.time.Time) -> None:
        if len(self._frame_times) >= 2:
            span_ns = self._frame_times[-1] - self._frame_times[0]
            fps = (len(self._frame_times) - 1) / (span_ns / 1e9) if span_ns > 0 else 0.0
        else:
            fps = 0.0

        visible_ids: set[int] = set()
        if det_age_s <= self.apriltag_timeout_s:
            visible_ids = {int(det.id) for det in self._latest_detections}

        calib_vis = self.calib_tag_id in visible_ids
        grey      = (200, 200, 200)
        dim       = (100, 100, 100)
        age_color = grey if det_age_s < 0.5 else (60, 180, 255) if det_age_s < 1.0 else (60, 60, 255)

        lines: list[tuple[str, tuple]] = [
            (f"FPS: {fps:.1f}", grey),
            (f"Det age: {det_age_s:.2f}s" if det_age_s < 1e6 else "Det age: --", age_color),
            (f"Tags visible: {len(visible_ids)}", grey),
            ("", grey),
            (f"Calib id={self.calib_tag_id}: {'VISIBLE' if calib_vis else '--'}",
             _COLOR_CALIB if calib_vis else dim),
        ]
        cube_line_start = len(lines)
        for cube_idx, group in enumerate(self._cube_groups):
            seen = sorted(t for t in group if t in visible_ids)
            color = _CUBE_COLORS[cube_idx] if seen else dim
            lines.append((f"Cube {cube_idx + 1}: {seen if seen else '--'}", color))

        font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.46, 1
        pad, line_h, panel_w = 7, 17, 315
        panel_h = pad * 2 + len(lines) * line_h
        x0, y0 = 8, 8

        roi  = canvas[y0: y0 + panel_h, x0: x0 + panel_w]
        canvas[y0: y0 + panel_h, x0: x0 + panel_w] = (roi * 0.25).astype(np.uint8)
        cv2.rectangle(canvas, (x0, y0), (x0 + panel_w, y0 + panel_h), (70, 70, 70), 1)

        for i, (text, color) in enumerate(lines):
            if not text:
                continue
            baseline = y0 + pad + (i + 1) * line_h - 2
            text_origin = (x0 + pad, baseline)
            cv2.putText(canvas, text, text_origin,
                        font, scale, color, thick, cv2.LINE_AA)
            cube_idx = i - cube_line_start
            if 0 <= cube_idx < len(self._cube_groups) and self._cube_bin[cube_idx]:
                text_width = cv2.getTextSize(text, font, scale, thick)[0][0]
                bin_color = (
                    _COLOR_NOT_IN_BIN
                    if self._cube_bin[cube_idx] == "NOT IN BIN"
                    else _COLOR_BIN
                )
                cv2.putText(
                    canvas,
                    f"  {self._cube_bin[cube_idx]}",
                    (text_origin[0] + text_width, baseline),
                    font,
                    scale,
                    bin_color,
                    thick,
                    cv2.LINE_AA,
                )


def main(args=None):
    rclpy.init(args=args)
    node = DepthTrackerNode()
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
