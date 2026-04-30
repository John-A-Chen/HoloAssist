#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker


def _normalize(vec: np.ndarray) -> Optional[np.ndarray]:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-9:
        return None
    return vec / norm


def _rotation_matrix_from_quaternion(
    x: float, y: float, z: float, w: float
) -> np.ndarray:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _quat_from_rotation_matrix(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    m00 = float(rotation[0, 0])
    m01 = float(rotation[0, 1])
    m02 = float(rotation[0, 2])
    m10 = float(rotation[1, 0])
    m11 = float(rotation[1, 1])
    m12 = float(rotation[1, 2])
    m20 = float(rotation[2, 0])
    m21 = float(rotation[2, 1])
    m22 = float(rotation[2, 2])

    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def _orthogonal_unit(vec: np.ndarray) -> np.ndarray:
    x_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    y_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    ref = x_axis if abs(float(np.dot(vec, x_axis))) < 0.9 else y_axis
    ortho = ref - float(np.dot(ref, vec)) * vec
    out = _normalize(ortho)
    if out is None:
        return np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return out


class AprilTagCubePoseNode(Node):
    """Estimate AprilCube pose from per-tag TFs emitted by apriltag_ros."""

    def __init__(self) -> None:
        super().__init__("apriltag_cube_pose")

        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("tag_ids", [10, 11, 12, 13, 14, 15])
        self.declare_parameter("tag_frame_names", [])
        self.declare_parameter("detections_timeout_s", 0.5)
        self.declare_parameter("tf_lookup_timeout_s", 0.05)
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter(
            "obstacle_marker_topic", "/holo_assist_depth_tracker/obstacle_marker"
        )
        self.declare_parameter("use_obstacle_marker_center_fusion", False)
        self.declare_parameter("publish_obstacle_only_pose", False)
        self.declare_parameter("obstacle_timeout_s", 0.75)
        self.declare_parameter("obstacle_center_weight_min", 0.20)
        self.declare_parameter("obstacle_center_weight_max", 0.65)
        self.declare_parameter("obstacle_max_center_delta_m", 0.15)

        self.declare_parameter("cube_size_m", 0.075)
        self.declare_parameter("normal_axis", "z")
        self.declare_parameter("normal_sign_preferred", 1.0)
        self.declare_parameter("cube_pose_z_offset_m", 0.0)

        self.declare_parameter("tag_id_pos_x", 10)
        self.declare_parameter("tag_id_neg_x", 11)
        self.declare_parameter("tag_id_pos_y", 12)
        self.declare_parameter("tag_id_neg_y", 13)
        self.declare_parameter("tag_id_pos_z", 14)
        self.declare_parameter("tag_id_neg_z", 15)

        self.declare_parameter(
            "cube_pose_topic", "/holoassist/perception/april_cube_pose"
        )
        self.declare_parameter(
            "cube_marker_topic", "/holoassist/perception/april_cube_marker"
        )
        self.declare_parameter(
            "cube_status_topic", "/holoassist/perception/april_cube_status"
        )
        self.declare_parameter("publish_cube_tf", True)
        self.declare_parameter("cube_tf_child_frame", "apriltag_cube")
        self.declare_parameter("publish_tag_tfs", True)
        self.declare_parameter("tag_tf_child_prefix", "apriltag_cube_tag_")
        self.declare_parameter("publish_cube_marker", True)
        self.declare_parameter("marker_alpha", 0.80)
        self.declare_parameter("marker_lifetime_s", 0.4)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value).strip()
        self.tag_family = str(self.get_parameter("tag_family").value).strip()
        self.tag_ids = [int(v) for v in self.get_parameter("tag_ids").value]
        self.tag_frame_names = [str(v) for v in self.get_parameter("tag_frame_names").value]
        self.detections_timeout_s = float(
            self.get_parameter("detections_timeout_s").value
        )
        self.tf_lookup_timeout_s = float(self.get_parameter("tf_lookup_timeout_s").value)
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        self.obstacle_marker_topic = str(
            self.get_parameter("obstacle_marker_topic").value
        )
        self.use_obstacle_marker_center_fusion = bool(
            self.get_parameter("use_obstacle_marker_center_fusion").value
        )
        self.publish_obstacle_only_pose = bool(
            self.get_parameter("publish_obstacle_only_pose").value
        )
        self.obstacle_timeout_s = float(self.get_parameter("obstacle_timeout_s").value)
        self.obstacle_center_weight_min = float(
            self.get_parameter("obstacle_center_weight_min").value
        )
        self.obstacle_center_weight_max = float(
            self.get_parameter("obstacle_center_weight_max").value
        )
        self.obstacle_max_center_delta_m = float(
            self.get_parameter("obstacle_max_center_delta_m").value
        )

        self.cube_size_m = float(self.get_parameter("cube_size_m").value)
        self.normal_axis = str(self.get_parameter("normal_axis").value).strip().lower()
        self.normal_sign_preferred = float(
            self.get_parameter("normal_sign_preferred").value
        )
        self.cube_pose_z_offset_m = float(
            self.get_parameter("cube_pose_z_offset_m").value
        )

        self.tag_id_pos_x = int(self.get_parameter("tag_id_pos_x").value)
        self.tag_id_neg_x = int(self.get_parameter("tag_id_neg_x").value)
        self.tag_id_pos_y = int(self.get_parameter("tag_id_pos_y").value)
        self.tag_id_neg_y = int(self.get_parameter("tag_id_neg_y").value)
        self.tag_id_pos_z = int(self.get_parameter("tag_id_pos_z").value)
        self.tag_id_neg_z = int(self.get_parameter("tag_id_neg_z").value)

        self.cube_pose_topic = str(self.get_parameter("cube_pose_topic").value)
        self.cube_marker_topic = str(self.get_parameter("cube_marker_topic").value)
        self.cube_status_topic = str(self.get_parameter("cube_status_topic").value)
        self.publish_cube_tf = bool(self.get_parameter("publish_cube_tf").value)
        self.cube_tf_child_frame = str(
            self.get_parameter("cube_tf_child_frame").value
        ).strip()
        self.publish_tag_tfs = bool(self.get_parameter("publish_tag_tfs").value)
        self.tag_tf_child_prefix = str(
            self.get_parameter("tag_tf_child_prefix").value
        ).strip()
        self.publish_cube_marker = bool(self.get_parameter("publish_cube_marker").value)
        self.marker_alpha = float(self.get_parameter("marker_alpha").value)
        self.marker_lifetime_s = float(self.get_parameter("marker_lifetime_s").value)

        self.detections_timeout_s = max(0.05, self.detections_timeout_s)
        self.tf_lookup_timeout_s = max(0.0, self.tf_lookup_timeout_s)
        self.update_rate_hz = max(1.0, self.update_rate_hz)
        self.obstacle_timeout_s = max(0.05, self.obstacle_timeout_s)
        self.obstacle_center_weight_min = float(
            np.clip(self.obstacle_center_weight_min, 0.0, 1.0)
        )
        self.obstacle_center_weight_max = float(
            np.clip(self.obstacle_center_weight_max, 0.0, 1.0)
        )
        if self.obstacle_center_weight_max < self.obstacle_center_weight_min:
            self.obstacle_center_weight_max = self.obstacle_center_weight_min
        self.obstacle_max_center_delta_m = max(0.0, self.obstacle_max_center_delta_m)
        self.cube_size_m = max(0.01, self.cube_size_m)
        self.marker_alpha = max(0.05, min(1.0, self.marker_alpha))
        self.marker_lifetime_s = max(0.0, self.marker_lifetime_s)
        if self.normal_axis not in ("x", "y", "z"):
            self.normal_axis = "z"
        if self.normal_sign_preferred == 0.0:
            self.normal_sign_preferred = 1.0
        self.normal_sign_preferred = (
            1.0 if self.normal_sign_preferred >= 0.0 else -1.0
        )
        if not self.cube_tf_child_frame:
            self.cube_tf_child_frame = "apriltag_cube"
        if self.publish_tag_tfs and not self.tag_tf_child_prefix:
            self.tag_tf_child_prefix = "apriltag_cube_tag_"

        self._axis_index = {"x": 0, "y": 1, "z": 2}[self.normal_axis]
        self._last_normal_sign = self.normal_sign_preferred

        self._id_to_cube_axis: Dict[int, np.ndarray] = {
            self.tag_id_pos_x: np.array([1.0, 0.0, 0.0], dtype=np.float64),
            self.tag_id_neg_x: np.array([-1.0, 0.0, 0.0], dtype=np.float64),
            self.tag_id_pos_y: np.array([0.0, 1.0, 0.0], dtype=np.float64),
            self.tag_id_neg_y: np.array([0.0, -1.0, 0.0], dtype=np.float64),
            self.tag_id_pos_z: np.array([0.0, 0.0, 1.0], dtype=np.float64),
            self.tag_id_neg_z: np.array([0.0, 0.0, -1.0], dtype=np.float64),
        }

        if not self.tag_ids:
            self.tag_ids = sorted(self._id_to_cube_axis.keys())
        else:
            self.tag_ids = sorted(set(self.tag_ids))

        self._tag_frame_by_id: Dict[int, str] = {}
        if len(self.tag_frame_names) == len(self.tag_ids):
            for tag_id, frame_name in zip(self.tag_ids, self.tag_frame_names):
                self._tag_frame_by_id[int(tag_id)] = frame_name.strip()
        else:
            for tag_id in self.tag_ids:
                self._tag_frame_by_id[int(tag_id)] = f"tag{self.tag_family}:{int(tag_id)}"

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pose_pub = self.create_publisher(PoseStamped, self.cube_pose_topic, reliable_qos)
        self.marker_pub = self.create_publisher(Marker, self.cube_marker_topic, reliable_qos)
        self.status_pub = self.create_publisher(String, self.cube_status_topic, reliable_qos)

        self.detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._on_detections,
            10,
        )
        self.obstacle_sub = self.create_subscription(
            Marker,
            self.obstacle_marker_topic,
            self._on_obstacle_marker,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self._on_timer)

        self._latest_detection_stamp: Optional[rclpy.time.Time] = None
        self._latest_frame_id: str = self.camera_frame
        self._latest_visible_ids: List[int] = []
        self._latest_obstacle_stamp: Optional[rclpy.time.Time] = None
        self._latest_obstacle_frame_id: str = ""
        self._latest_obstacle_center: Optional[np.ndarray] = None
        self._last_pose_frame_id: str = ""
        self._last_pose_quat: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
        self._marker_active = False
        self._last_status = ""

        self.get_logger().info(
            "apriltag cube pose started: detections=%s frame=%s cube_size=%.4fm tags=%s tf_child=%s tag_tf_prefix=%s obstacle_topic=%s obstacle_fusion=%s obstacle_only=%s"
            % (
                self.detections_topic,
                self.camera_frame if self.camera_frame else "<from_detections>",
                self.cube_size_m,
                self.tag_ids,
                self.cube_tf_child_frame if self.publish_cube_tf else "<disabled>",
                self.tag_tf_child_prefix if self.publish_tag_tfs else "<disabled>",
                self.obstacle_marker_topic,
                self.use_obstacle_marker_center_fusion,
                self.publish_obstacle_only_pose,
            )
        )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        visible_ids: List[int] = []
        allowed = set(self.tag_ids)
        for det in msg.detections:
            det_id = int(det.id)
            if det_id in allowed and det_id in self._id_to_cube_axis:
                visible_ids.append(det_id)

        self._latest_visible_ids = sorted(set(visible_ids))
        self._latest_frame_id = self.camera_frame or str(msg.header.frame_id).strip()

        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds == 0:
                stamp = self.get_clock().now()
        except Exception:
            stamp = self.get_clock().now()
        self._latest_detection_stamp = stamp

    def _on_obstacle_marker(self, msg: Marker) -> None:
        if msg.action in (Marker.DELETE, Marker.DELETEALL):
            self._latest_obstacle_stamp = None
            self._latest_obstacle_frame_id = ""
            self._latest_obstacle_center = None
            return

        frame_id = str(msg.header.frame_id).strip()
        if not frame_id:
            return

        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds == 0:
                stamp = self.get_clock().now()
        except Exception:
            stamp = self.get_clock().now()

        self._latest_obstacle_stamp = stamp
        self._latest_obstacle_frame_id = frame_id
        self._latest_obstacle_center = np.array(
            [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ],
            dtype=np.float64,
        )

    def _obstacle_center_in_frame(self, target_frame: str) -> Optional[np.ndarray]:
        if (
            self._latest_obstacle_stamp is None
            or self._latest_obstacle_center is None
            or not self._latest_obstacle_frame_id
            or not target_frame
        ):
            return None

        age_s = (
            self.get_clock().now() - self._latest_obstacle_stamp
        ).nanoseconds / 1e9
        if age_s > self.obstacle_timeout_s:
            return None

        if self._latest_obstacle_frame_id == target_frame:
            return np.array(self._latest_obstacle_center, dtype=np.float64)

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                self._latest_obstacle_frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_s),
            )
        except TransformException:
            return None

        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        rot = _rotation_matrix_from_quaternion(
            float(r.x), float(r.y), float(r.z), float(r.w)
        )
        trans = np.array([float(t.x), float(t.y), float(t.z)], dtype=np.float64)
        return rot @ self._latest_obstacle_center + trans

    def _obstacle_weight_for_tag_count(self, tag_count: int) -> float:
        # More visible tags -> trust tag geometry more; fewer tags -> trust obstacle center more.
        if tag_count <= 1:
            return self.obstacle_center_weight_max
        if tag_count >= 3:
            return self.obstacle_center_weight_min
        u = float(np.clip((3.0 - float(tag_count)) / 2.0, 0.0, 1.0))
        return float(
            self.obstacle_center_weight_min
            + (self.obstacle_center_weight_max - self.obstacle_center_weight_min) * u
        )

    def _candidate_tag_frames(self, tag_id: int) -> List[str]:
        explicit = self._tag_frame_by_id.get(tag_id, "").strip()
        candidates = []
        if explicit:
            candidates.append(explicit)
        candidates.extend(
            [
                f"tag{self.tag_family}:{tag_id}",
                f"tag{tag_id}",
                f"tag_{tag_id}",
                f"{self.tag_family}:{tag_id}",
                f"apriltag_{tag_id}",
            ]
        )
        deduped: List[str] = []
        seen = set()
        for frame in candidates:
            if not frame or frame in seen:
                continue
            seen.add(frame)
            deduped.append(frame)
        return deduped

    def _lookup_tag_transform(
        self, target_frame: str, tag_id: int
    ) -> Optional[Tuple[TransformStamped, str]]:
        for tag_frame in self._candidate_tag_frames(tag_id):
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    target_frame,
                    tag_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_lookup_timeout_s),
                )
                return tf_msg, tag_frame
            except TransformException:
                continue
        return None

    def _publish_observed_tag_tfs(
        self,
        frame_id: str,
        tag_transforms: List[Tuple[int, TransformStamped]],
        stamp_msg,
    ) -> None:
        if not self.publish_tag_tfs or not tag_transforms:
            return
        out = []
        for tag_id, tf_in in tag_transforms:
            msg = TransformStamped()
            msg.header.frame_id = frame_id
            msg.header.stamp = stamp_msg
            msg.child_frame_id = f"{self.tag_tf_child_prefix}{int(tag_id)}"
            msg.transform = tf_in.transform
            out.append(msg)
        self.tf_broadcaster.sendTransform(out)

    def _publish_cube_outputs(
        self,
        frame_id: str,
        center: np.ndarray,
        quat_xyzw: Tuple[float, float, float, float],
    ) -> None:
        qx, qy, qz, qw = quat_xyzw
        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = stamp
        pose_msg.pose.position.x = float(center[0])
        pose_msg.pose.position.y = float(center[1])
        pose_msg.pose.position.z = float(center[2])
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        self.pose_pub.publish(pose_msg)

        if self.publish_cube_tf:
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = frame_id
            tf_msg.header.stamp = stamp
            tf_msg.child_frame_id = self.cube_tf_child_frame
            tf_msg.transform.translation.x = float(center[0])
            tf_msg.transform.translation.y = float(center[1])
            tf_msg.transform.translation.z = float(center[2])
            tf_msg.transform.rotation.x = float(qx)
            tf_msg.transform.rotation.y = float(qy)
            tf_msg.transform.rotation.z = float(qz)
            tf_msg.transform.rotation.w = float(qw)
            self.tf_broadcaster.sendTransform(tf_msg)

        if self.publish_cube_marker:
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = "holoassist_april_cube"
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose_msg.pose
            marker.scale.x = float(self.cube_size_m)
            marker.scale.y = float(self.cube_size_m)
            marker.scale.z = float(self.cube_size_m)
            marker.color.r = 0.10
            marker.color.g = 0.85
            marker.color.b = 0.35
            marker.color.a = float(self.marker_alpha)
            self._apply_marker_lifetime(marker)
            self.marker_pub.publish(marker)
            self._marker_active = True

        self._last_pose_frame_id = frame_id
        self._last_pose_quat = (float(qx), float(qy), float(qz), float(qw))

    def _on_timer(self) -> None:
        frame_id = self._latest_frame_id.strip()
        if not frame_id:
            self._publish_status("waiting for detections/frame_id")
            self._delete_marker_if_needed()
            return

        if self._latest_detection_stamp is None:
            self._publish_status("waiting for detections")
            self._delete_marker_if_needed()
            return

        age_s = (self.get_clock().now() - self._latest_detection_stamp).nanoseconds / 1e9
        if age_s > self.detections_timeout_s:
            if self.publish_obstacle_only_pose:
                fallback_frame = frame_id or self._latest_obstacle_frame_id
                obstacle_center = self._obstacle_center_in_frame(fallback_frame)
                if obstacle_center is not None and fallback_frame:
                    quat = (
                        self._last_pose_quat
                        if self._last_pose_frame_id == fallback_frame
                        else (0.0, 0.0, 0.0, 1.0)
                    )
                    self._publish_cube_outputs(fallback_frame, obstacle_center, quat)
                    self._publish_status(
                        "obstacle-only pose frame=%s (stale detections age=%.2fs > %.2fs)"
                        % (fallback_frame, age_s, self.detections_timeout_s)
                    )
                    return
            self._publish_status(
                "stale detections age=%.2fs (>%.2fs)" % (age_s, self.detections_timeout_s)
            )
            self._delete_marker_if_needed(frame_id=frame_id)
            return

        observations: List[Dict[str, object]] = []
        observed_tag_tfs: List[Tuple[int, TransformStamped]] = []
        for tag_id in self._latest_visible_ids:
            lookup = self._lookup_tag_transform(frame_id, tag_id)
            if lookup is None:
                continue
            tf_msg, _ = lookup
            observed_tag_tfs.append((int(tag_id), tf_msg))

            t = tf_msg.transform.translation
            r = tf_msg.transform.rotation
            point = np.array([float(t.x), float(t.y), float(t.z)], dtype=np.float64)
            rot = _rotation_matrix_from_quaternion(
                float(r.x), float(r.y), float(r.z), float(r.w)
            )

            raw_normal = _normalize(rot[:, self._axis_index].astype(np.float64))
            if raw_normal is None:
                continue

            tangent = _normalize(rot[:, 0].astype(np.float64))
            if tangent is None:
                tangent = _orthogonal_unit(raw_normal)

            cube_axis = self._id_to_cube_axis.get(tag_id)
            if cube_axis is None:
                continue

            observations.append(
                {
                    "id": int(tag_id),
                    "point": point,
                    "raw_normal": raw_normal,
                    "tangent": tangent,
                    "cube_axis": cube_axis,
                }
            )

        if not observations:
            if self.publish_obstacle_only_pose:
                obstacle_center = self._obstacle_center_in_frame(frame_id)
                if obstacle_center is not None:
                    quat = (
                        self._last_pose_quat
                        if self._last_pose_frame_id == frame_id
                        else (0.0, 0.0, 0.0, 1.0)
                    )
                    self._publish_cube_outputs(frame_id, obstacle_center, quat)
                    self._publish_status(
                        "obstacle-only pose frame=%s (no tag TFs)" % frame_id
                    )
                    return
            self._publish_status("no tag TFs available in frame=%s" % frame_id)
            self._delete_marker_if_needed(frame_id=frame_id)
            return

        self._publish_observed_tag_tfs(frame_id, observed_tag_tfs, self.get_clock().now().to_msg())

        solved = self._solve_pose(observations)
        if solved is None:
            self._publish_status("failed to solve cube pose")
            self._delete_marker_if_needed(frame_id=frame_id)
            return

        center, rot_c_cube, normal_sign, pos_resid_m, ori_resid_rad, used_ids = solved
        self._last_normal_sign = normal_sign

        if self.cube_pose_z_offset_m != 0.0:
            center = center + rot_c_cube[:, 2] * float(self.cube_pose_z_offset_m)

        fusion_note = ""
        if self.use_obstacle_marker_center_fusion:
            obstacle_center = self._obstacle_center_in_frame(frame_id)
            if obstacle_center is not None:
                delta = obstacle_center - center
                delta_norm = float(np.linalg.norm(delta))
                if (
                    self.obstacle_max_center_delta_m <= 0.0
                    or delta_norm <= self.obstacle_max_center_delta_m
                ):
                    w_obs = self._obstacle_weight_for_tag_count(len(observations))
                    center = (1.0 - w_obs) * center + w_obs * obstacle_center
                    fusion_note = " fuse_obs[w=%.2f d=%.0fmm]" % (
                        w_obs,
                        1000.0 * delta_norm,
                    )
                else:
                    fusion_note = " obs_reject[d=%.0fmm]" % (1000.0 * delta_norm)

        qx, qy, qz, qw = _quat_from_rotation_matrix(rot_c_cube)
        self._publish_cube_outputs(frame_id, center, (qx, qy, qz, qw))

        self._publish_status(
            "frame=%s tags=%s sign=%+d pos_resid=%.1fmm ori_resid=%.1fdeg%s"
            % (
                frame_id,
                used_ids,
                int(normal_sign),
                1000.0 * pos_resid_m,
                math.degrees(ori_resid_rad),
                fusion_note,
            )
        )

    def _solve_pose(
        self, observations: List[Dict[str, object]]
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float, float, float, List[int]]]:
        if not observations:
            return None

        half = 0.5 * float(self.cube_size_m)
        preferred = 1.0 if self._last_normal_sign >= 0.0 else -1.0
        sign_candidates = [preferred, -preferred]

        best = None
        for sign in sign_candidates:
            normals = []
            for obs in observations:
                raw = np.asarray(obs["raw_normal"], dtype=np.float64)
                n = _normalize(sign * raw)
                if n is None:
                    n = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                normals.append(n)

            centers = []
            for obs, normal in zip(observations, normals):
                point = np.asarray(obs["point"], dtype=np.float64)
                centers.append(point - half * normal)

            center = np.mean(np.asarray(centers, dtype=np.float64), axis=0)
            pos_resid = float(
                np.mean([np.linalg.norm(c - center) for c in centers])
            )

            rot_c_cube, ori_resid = self._estimate_rotation(observations, normals)
            score = pos_resid + 0.05 * ori_resid

            if best is None or score < best["score"]:
                best = {
                    "center": center,
                    "rotation": rot_c_cube,
                    "sign": sign,
                    "pos_resid": pos_resid,
                    "ori_resid": ori_resid,
                    "score": score,
                }

        if best is None:
            return None

        used_ids = sorted(int(obs["id"]) for obs in observations)
        return (
            np.asarray(best["center"], dtype=np.float64),
            np.asarray(best["rotation"], dtype=np.float64),
            float(best["sign"]),
            float(best["pos_resid"]),
            float(best["ori_resid"]),
            used_ids,
        )

    def _estimate_rotation(
        self, observations: List[Dict[str, object]], normals: List[np.ndarray]
    ) -> Tuple[np.ndarray, float]:
        if len(observations) >= 2:
            m = np.zeros((3, 3), dtype=np.float64)
            for obs, normal in zip(observations, normals):
                cube_axis = np.asarray(obs["cube_axis"], dtype=np.float64)
                m += np.outer(normal, cube_axis)

            u, _, vt = np.linalg.svd(m)
            rot = u @ vt
            if float(np.linalg.det(rot)) < 0.0:
                u[:, -1] *= -1.0
                rot = u @ vt

            ori_err = 0.0
            for obs, normal in zip(observations, normals):
                cube_axis = np.asarray(obs["cube_axis"], dtype=np.float64)
                projected = _normalize(rot @ cube_axis)
                if projected is None:
                    continue
                dot = float(np.clip(np.dot(projected, normal), -1.0, 1.0))
                ori_err += math.acos(dot)
            ori_err /= max(1, len(normals))
            return rot, ori_err

        obs = observations[0]
        normal = normals[0]
        cube_axis = _normalize(np.asarray(obs["cube_axis"], dtype=np.float64))
        if cube_axis is None:
            cube_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        tangent = np.asarray(obs["tangent"], dtype=np.float64)
        tangent = tangent - float(np.dot(tangent, normal)) * normal
        tangent = _normalize(tangent)
        if tangent is None:
            tangent = _orthogonal_unit(normal)

        bitangent = _normalize(np.cross(normal, tangent))
        if bitangent is None:
            bitangent = _orthogonal_unit(normal)
        tangent = _normalize(np.cross(bitangent, normal))
        if tangent is None:
            tangent = _orthogonal_unit(normal)

        cube_tangent = _orthogonal_unit(cube_axis)
        cube_bitangent = _normalize(np.cross(cube_axis, cube_tangent))
        if cube_bitangent is None:
            cube_bitangent = _orthogonal_unit(cube_axis)
        cube_tangent = _normalize(np.cross(cube_bitangent, cube_axis))
        if cube_tangent is None:
            cube_tangent = _orthogonal_unit(cube_axis)

        basis_cam = np.column_stack((tangent, bitangent, normal))
        basis_cube = np.column_stack((cube_tangent, cube_bitangent, cube_axis))
        rot = basis_cam @ basis_cube.T
        return rot, 0.0

    def _apply_marker_lifetime(self, marker: Marker) -> None:
        if self.marker_lifetime_s <= 0.0:
            return
        sec = int(self.marker_lifetime_s)
        nanosec = int((self.marker_lifetime_s - float(sec)) * 1e9)
        marker.lifetime.sec = max(0, sec)
        marker.lifetime.nanosec = max(0, min(999999999, nanosec))

    def _delete_marker_if_needed(self, frame_id: Optional[str] = None) -> None:
        if not self._marker_active or not self.publish_cube_marker:
            return
        marker = Marker()
        marker.header.frame_id = frame_id if frame_id else (self.camera_frame or "camera")
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "holoassist_april_cube"
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)
        self._marker_active = False

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

        if text != self._last_status:
            self.get_logger().info(text)
            self._last_status = text


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagCubePoseNode()
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
