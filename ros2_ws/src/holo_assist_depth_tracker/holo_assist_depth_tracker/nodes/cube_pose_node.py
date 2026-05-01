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

from holo_assist_depth_tracker.utils.apriltag import candidate_tag_frame_names
from holo_assist_depth_tracker.utils.math3d import (
    normalize,
    quaternion_from_rotation_matrix,
    rotation_matrix_from_quaternion,
)


class CubePoseNode(Node):
    """Estimate 4 independent cube poses from grouped AprilTag IDs."""

    EXPECTED_CUBE_EDGE_M = 0.040
    EXPECTED_CUBE_TAG_SIZE_M = 0.032
    EXPECTED_CUBE_FACE_OFFSET_M = 0.020
    WARNING_THROTTLE_S = 5.0
    DEBUG_THROTTLE_S = 1.0

    AXIS_SEQUENCE = [
        np.array([1.0, 0.0, 0.0], dtype=np.float64),
        np.array([-1.0, 0.0, 0.0], dtype=np.float64),
        np.array([0.0, 1.0, 0.0], dtype=np.float64),
        np.array([0.0, -1.0, 0.0], dtype=np.float64),
        np.array([0.0, 0.0, 1.0], dtype=np.float64),
        np.array([0.0, 0.0, -1.0], dtype=np.float64),
    ]

    CUBE_COLORS_RGBA = [
        (1.0, 0.0, 0.0, 0.75),  # cube 1 red
        (0.0, 1.0, 0.0, 0.75),  # cube 2 green
        (0.0, 0.3, 1.0, 0.75),  # cube 3 blue
        (1.0, 0.7, 0.0, 0.75),  # cube 4 yellow/orange
    ]

    def __init__(self) -> None:
        super().__init__("holoassist_cube_pose")
        self._last_log_time: Dict[str, float] = {}

        self.declare_parameter("detections_topic", "/detections_all")
        self.declare_parameter("workspace_frame", "workspace_frame")
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("cube_edge_size_m", 0.040)
        self.declare_parameter("cube_tag_size_m", 0.032)
        self.declare_parameter("cube_face_offset_m", 0.020)
        self.declare_parameter("cube_size_m", 0.040)
        self.declare_parameter("cube_min_center_z_m", -1.0)
        self.declare_parameter("detections_timeout_s", 1.0)
        self.declare_parameter("tf_lookup_timeout_s", 0.05)
        self.declare_parameter("timer_hz", 20.0)
        self.declare_parameter("publish_cube_tf", True)
        self.declare_parameter("publish_cube_markers", True)
        self.declare_parameter("marker_alpha", 1.0)
        self.declare_parameter("legacy_cube_pose_topic", "/holoassist/perception/april_cube_pose")

        self.declare_parameter("april_cube_1_tag_ids", [10, 11, 12, 13, 14, 15])
        self.declare_parameter("april_cube_2_tag_ids", [16, 17, 18, 19, 20, 21])
        self.declare_parameter("april_cube_3_tag_ids", [22, 23, 24, 25, 26, 27])
        self.declare_parameter("april_cube_4_tag_ids", [28, 29, 30, 31, 32, 33])
        # Backward-compatible aliases.
        self.declare_parameter("cube_1_ids", [10, 11, 12, 13, 14, 15])
        self.declare_parameter("cube_2_ids", [16, 17, 18, 19, 20, 21])
        self.declare_parameter("cube_3_ids", [22, 23, 24, 25, 26, 27])
        self.declare_parameter("cube_4_ids", [28, 29, 30, 31, 32, 33])

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.workspace_frame = str(self.get_parameter("workspace_frame").value)
        self.tag_family = str(self.get_parameter("tag_family").value)
        legacy_cube_size_m = float(self.get_parameter("cube_size_m").value)
        self.cube_edge_size_m = max(0.01, float(self.get_parameter("cube_edge_size_m").value))
        self.cube_tag_size_m = max(0.001, float(self.get_parameter("cube_tag_size_m").value))
        self.cube_face_offset_m = max(0.001, float(self.get_parameter("cube_face_offset_m").value))
        if abs(self.cube_edge_size_m - legacy_cube_size_m) > 1e-9:
            self._log_throttled(
                "warn",
                "legacy_cube_size_m_mismatch",
                self.WARNING_THROTTLE_S,
                "cube_size_m (legacy)=%.4f differs from cube_edge_size_m=%.4f; using cube_edge_size_m."
                % (legacy_cube_size_m, self.cube_edge_size_m),
            )
        self.cube_min_center_z_m = float(
            self.get_parameter("cube_min_center_z_m").value
        )
        self.detections_timeout_s = max(0.05, float(self.get_parameter("detections_timeout_s").value))
        self.tf_lookup_timeout_s = max(0.0, float(self.get_parameter("tf_lookup_timeout_s").value))
        self.timer_hz = max(1.0, float(self.get_parameter("timer_hz").value))
        self.publish_cube_tf = bool(self.get_parameter("publish_cube_tf").value)
        self.publish_cube_markers = bool(self.get_parameter("publish_cube_markers").value)
        self.marker_alpha = max(0.05, min(1.0, float(self.get_parameter("marker_alpha").value)))
        self.legacy_cube_pose_topic = str(self.get_parameter("legacy_cube_pose_topic").value)

        self.cube_groups: List[List[int]] = [
            self._read_tag_ids("april_cube_1_tag_ids", "cube_1_ids"),
            self._read_tag_ids("april_cube_2_tag_ids", "cube_2_ids"),
            self._read_tag_ids("april_cube_3_tag_ids", "cube_3_ids"),
            self._read_tag_ids("april_cube_4_tag_ids", "cube_4_ids"),
        ]
        self.cube_defs: List[Dict[str, object]] = []
        self._tag_to_cube: Dict[int, int] = {}
        self._tag_overlap_reported = False

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pose_pubs = []
        self.marker_pubs = []
        self.status_pubs = []
        self.cube_tf_frames = []
        for idx in range(4):
            n = idx + 1
            color = self.CUBE_COLORS_RGBA[idx]
            tag_ids = self.cube_groups[idx]
            pose_topic = f"/holoassist/perception/april_cube_{n}_pose"
            marker_topic = f"/holoassist/perception/april_cube_{n}_marker"
            status_topic = f"/holoassist/perception/april_cube_{n}_status"
            tf_child = f"apriltag_cube_{n}"
            self.pose_pubs.append(
                self.create_publisher(
                    PoseStamped,
                    pose_topic,
                    reliable_qos,
                )
            )
            self.marker_pubs.append(
                self.create_publisher(
                    Marker,
                    marker_topic,
                    reliable_qos,
                )
            )
            self.status_pubs.append(
                self.create_publisher(
                    String,
                    status_topic,
                    reliable_qos,
                )
            )
            self.cube_tf_frames.append(tf_child)
            self.cube_defs.append(
                {
                    "cube_id": n,
                    "tag_ids": tag_ids,
                    "pose_topic": pose_topic,
                    "marker_topic": marker_topic,
                    "status_topic": status_topic,
                    "tf_child_frame": tf_child,
                    "marker_color": color,
                }
            )
            for tag_id in tag_ids:
                if tag_id in self._tag_to_cube and self._tag_to_cube[tag_id] != n:
                    self._tag_overlap_reported = True
                    self._log_throttled(
                        "warn",
                        f"tag_overlap_{tag_id}",
                        self.WARNING_THROTTLE_S,
                        "tag id %d appears in more than one cube group: cube_%d and cube_%d"
                        % (tag_id, self._tag_to_cube[tag_id], n),
                    )
                self._tag_to_cube[tag_id] = n

        self.legacy_pose_pub = self.create_publisher(PoseStamped, self.legacy_cube_pose_topic, reliable_qos)

        self._detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._on_detections,
            reliable_qos,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._latest_visible_ids: set[int] = set()
        self._latest_detection_stamp: Optional[rclpy.time.Time] = None

        self._last_normal_sign: List[float] = [1.0, 1.0, 1.0, 1.0]
        self._marker_active: List[bool] = [False, False, False, False]
        self._last_centers: List[Optional[np.ndarray]] = [None, None, None, None]

        self._timer = self.create_timer(1.0 / self.timer_hz, self._on_timer)
        self._validate_configuration()

        self.get_logger().info(
            "cube pose node started detections=%s workspace_frame=%s cube_edge=%.3f cube_tag=%.3f cube_face_offset=%.3f groups=%s"
            % (
                self.detections_topic,
                self.workspace_frame,
                self.cube_edge_size_m,
                self.cube_tag_size_m,
                self.cube_face_offset_m,
                self.cube_groups,
            )
        )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self._latest_visible_ids = {int(det.id) for det in msg.detections}
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        self._latest_detection_stamp = stamp

    def _on_timer(self) -> None:
        if self._latest_detection_stamp is None:
            for idx in range(4):
                self._publish_status(
                    idx,
                    state="stale",
                    visible_ids=[],
                    selected_tag_id=-1,
                    age_s=float("inf"),
                    frame_id=self.workspace_frame,
                    center=self._last_centers[idx],
                    reason="waiting_for_detections",
                )
            return

        age_s = (self.get_clock().now() - self._latest_detection_stamp).nanoseconds / 1e9
        if age_s > self.detections_timeout_s:
            for idx in range(4):
                cube_id = int(self.cube_defs[idx]["cube_id"])
                visible_ids = sorted(
                    int(tag_id) for tag_id in self.cube_defs[idx]["tag_ids"] if int(tag_id) in self._latest_visible_ids
                )
                self._publish_status(
                    idx,
                    state="stale",
                    visible_ids=visible_ids,
                    selected_tag_id=-1,
                    age_s=age_s,
                    frame_id=self.workspace_frame,
                    center=self._last_centers[idx],
                    reason="detections_timeout",
                )
                self._delete_marker(idx)
                self._log_throttled(
                    "info",
                    f"cube_{cube_id}_stale",
                    self.DEBUG_THROTTLE_S,
                    "cube_id=%d state=stale visible_tag_ids=%s age_s=%.2f"
                    % (cube_id, visible_ids, age_s),
                )
            return

        first_cube_pose: Optional[PoseStamped] = None
        for idx, tag_ids in enumerate(self.cube_groups):
            cube_def = self.cube_defs[idx]
            cube_id = int(cube_def["cube_id"])
            detected_ids = sorted(int(tag_id) for tag_id in tag_ids if int(tag_id) in self._latest_visible_ids)
            observations = self._collect_observations(tag_ids)
            visible_ids = sorted(int(obs["id"]) for obs in observations)
            if not observations:
                self._publish_status(
                    idx,
                    state="stale",
                    visible_ids=detected_ids,
                    selected_tag_id=-1,
                    age_s=age_s,
                    frame_id=self.workspace_frame,
                    center=self._last_centers[idx],
                    reason="no_visible_cube_tags",
                )
                self._delete_marker(idx)
                continue

            solved = self._solve_pose(observations, idx)
            if solved is None:
                self._publish_status(
                    idx,
                    state="stale",
                    visible_ids=detected_ids,
                    selected_tag_id=-1,
                    age_s=age_s,
                    frame_id=self.workspace_frame,
                    center=self._last_centers[idx],
                    reason="failed_to_solve_cube_pose",
                )
                self._delete_marker(idx)
                continue

            center, rot_w_c, pos_resid_m, ori_resid_rad, used_ids, selected_tag_id = solved
            if self.cube_min_center_z_m >= 0.0:
                min_z = self.cube_min_center_z_m
            else:
                # Default safety floor: center cannot be below half-cube above workspace plane.
                min_z = 0.5 * float(self.cube_edge_size_m)
            if float(center[2]) < float(min_z):
                center = np.asarray(center, dtype=np.float64).copy()
                center[2] = float(min_z)
            qx, qy, qz, qw = quaternion_from_rotation_matrix(rot_w_c)
            pose = self._publish_cube_outputs(idx, center, (qx, qy, qz, qw))
            self._last_centers[idx] = np.asarray(center, dtype=np.float64)

            if idx == 0:
                first_cube_pose = pose

            self._publish_status(
                idx,
                state="visible",
                visible_ids=used_ids,
                selected_tag_id=selected_tag_id,
                age_s=age_s,
                frame_id=self.workspace_frame,
                center=center,
                reason="ok",
            )
            marker_color = cube_def["marker_color"]
            self._log_throttled(
                "info",
                f"cube_{cube_id}_debug",
                self.DEBUG_THROTTLE_S,
                "cube_id=%d visible_tag_ids=%s selected_tag_id=%d center=(%.3f,%.3f,%.3f) "
                "marker_frame=%s tf_parent=%s tf_child=%s marker_scale=(%.3f,%.3f,%.3f) "
                "cube_color_rgba=(%.2f,%.2f,%.2f,%.2f) pos_resid_mm=%.1f ori_resid_deg=%.1f"
                % (
                    cube_id,
                    used_ids,
                    selected_tag_id,
                    float(center[0]),
                    float(center[1]),
                    float(center[2]),
                    self.workspace_frame,
                    self.workspace_frame,
                    str(cube_def["tf_child_frame"]),
                    self.cube_edge_size_m,
                    self.cube_edge_size_m,
                    self.cube_edge_size_m,
                    float(marker_color[0]),
                    float(marker_color[1]),
                    float(marker_color[2]),
                    float(marker_color[3]),
                    1000.0 * pos_resid_m,
                    math.degrees(ori_resid_rad),
                ),
            )
            self._warn_if_pose_unreasonable(cube_id, center)

        if first_cube_pose is not None:
            self.legacy_pose_pub.publish(first_cube_pose)

    def _collect_observations(self, tag_ids: List[int]) -> List[Dict[str, object]]:
        observations: List[Dict[str, object]] = []
        visible_ordered = [tag_id for tag_id in tag_ids if tag_id in self._latest_visible_ids]

        for tag_id in visible_ordered:
            tf_msg = self._lookup_tag_transform(tag_id)
            if tf_msg is None:
                continue

            t = tf_msg.transform.translation
            r = tf_msg.transform.rotation
            point = np.array([float(t.x), float(t.y), float(t.z)], dtype=np.float64)
            rot_w_t = rotation_matrix_from_quaternion(float(r.x), float(r.y), float(r.z), float(r.w))

            raw_normal = normalize(rot_w_t[:, 2].astype(np.float64))
            if raw_normal is None:
                continue

            observations.append(
                {
                    "id": int(tag_id),
                    "point": point,
                    "raw_normal": raw_normal,
                    "cube_axis": self._cube_axis_for_id(tag_ids, tag_id),
                }
            )

        return observations

    def _lookup_tag_transform(self, tag_id: int) -> Optional[TransformStamped]:
        for candidate in candidate_tag_frame_names(self.tag_family, tag_id):
            try:
                return self._tf_buffer.lookup_transform(
                    self.workspace_frame,
                    candidate,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_lookup_timeout_s),
                )
            except TransformException:
                continue
        return None

    def _cube_axis_for_id(self, group_ids: List[int], tag_id: int) -> np.ndarray:
        try:
            idx = group_ids.index(tag_id)
        except ValueError:
            idx = 0
        idx = max(0, min(5, idx))
        return np.asarray(self.AXIS_SEQUENCE[idx], dtype=np.float64)

    def _solve_pose(
        self,
        observations: List[Dict[str, object]],
        cube_idx: int,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float, float, List[int], int]]:
        if not observations:
            return None

        face_offset = float(self.cube_face_offset_m)
        preferred = 1.0 if self._last_normal_sign[cube_idx] >= 0.0 else -1.0
        sign_candidates = [preferred, -preferred]

        best = None
        for sign in sign_candidates:
            normals = []
            centers = []
            for obs in observations:
                raw = np.asarray(obs["raw_normal"], dtype=np.float64)
                normal = normalize(sign * raw)
                if normal is None:
                    normal = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                normals.append(normal)

                point = np.asarray(obs["point"], dtype=np.float64)
                centers.append(point - face_offset * normal)

            center = np.mean(np.asarray(centers, dtype=np.float64), axis=0)
            pos_resid = float(np.mean([np.linalg.norm(c - center) for c in centers]))
            rot_w_c, ori_resid = self._estimate_rotation(observations, normals)
            score = pos_resid + 0.05 * ori_resid
            per_tag_residuals = [
                (int(obs["id"]), float(np.linalg.norm(c - center)))
                for obs, c in zip(observations, centers)
            ]
            selected_tag_id = min(per_tag_residuals, key=lambda item: item[1])[0]

            if best is None or score < best["score"]:
                best = {
                    "center": center,
                    "rotation": rot_w_c,
                    "pos_resid": pos_resid,
                    "ori_resid": ori_resid,
                    "score": score,
                    "sign": sign,
                    "selected_tag_id": selected_tag_id,
                }

        if best is None:
            return None

        self._last_normal_sign[cube_idx] = float(best["sign"])
        return (
            np.asarray(best["center"], dtype=np.float64),
            np.asarray(best["rotation"], dtype=np.float64),
            float(best["pos_resid"]),
            float(best["ori_resid"]),
            sorted(int(obs["id"]) for obs in observations),
            int(best["selected_tag_id"]),
        )

    def _estimate_rotation(
        self,
        observations: List[Dict[str, object]],
        normals: List[np.ndarray],
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
                projected = normalize(rot @ cube_axis)
                if projected is None:
                    continue
                dot = float(np.clip(np.dot(projected, normal), -1.0, 1.0))
                ori_err += math.acos(dot)
            ori_err /= max(1, len(normals))
            return rot, ori_err

        obs = observations[0]
        normal = normals[0]
        cube_axis = normalize(np.asarray(obs["cube_axis"], dtype=np.float64))
        if cube_axis is None:
            cube_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        cross = np.cross(cube_axis, normal)
        cross_norm = float(np.linalg.norm(cross))
        dot = float(np.clip(np.dot(cube_axis, normal), -1.0, 1.0))
        if cross_norm < 1e-9:
            if dot >= 0.0:
                rot_align = np.eye(3, dtype=np.float64)
            else:
                axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
                if abs(float(cube_axis[0])) > 0.9:
                    axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
                axis = normalize(axis - float(np.dot(axis, cube_axis)) * cube_axis)
                if axis is None:
                    axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                rot_align = self._axis_angle(axis, math.pi)
        else:
            axis = cross / cross_norm
            angle = math.atan2(cross_norm, dot)
            rot_align = self._axis_angle(axis, angle)

        helper_ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(np.dot(helper_ref, normal))) > 0.9:
            helper_ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        helper_tangent = normalize(helper_ref - float(np.dot(helper_ref, normal)) * normal)
        if helper_tangent is None:
            helper_tangent = np.array([0.0, 1.0, 0.0], dtype=np.float64)

        tangent_proj = normalize(rot_align @ np.array([1.0, 0.0, 0.0], dtype=np.float64))
        if tangent_proj is None:
            tangent_proj = helper_tangent

        dot_t = float(np.clip(np.dot(tangent_proj, helper_tangent), -1.0, 1.0))
        sign = float(np.dot(np.cross(tangent_proj, helper_tangent), normal))
        yaw = math.atan2(sign, dot_t)
        rot_yaw = self._axis_angle(normal, yaw)
        rot = rot_yaw @ rot_align
        return rot, 0.0

    def _axis_angle(self, axis: np.ndarray, angle: float) -> np.ndarray:
        x, y, z = axis
        c = math.cos(angle)
        s = math.sin(angle)
        t = 1.0 - c
        return np.array(
            [
                [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
            ],
            dtype=np.float64,
        )

    def _publish_cube_outputs(
        self,
        cube_idx: int,
        center: np.ndarray,
        quat_xyzw: Tuple[float, float, float, float],
    ) -> PoseStamped:
        qx, qy, qz, qw = quat_xyzw
        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.workspace_frame
        pose_msg.header.stamp = stamp
        pose_msg.pose.position.x = float(center[0])
        pose_msg.pose.position.y = float(center[1])
        pose_msg.pose.position.z = float(center[2])
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        self.pose_pubs[cube_idx].publish(pose_msg)

        if self.publish_cube_tf:
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = self.workspace_frame
            tf_msg.header.stamp = stamp
            tf_msg.child_frame_id = self.cube_tf_frames[cube_idx]
            tf_msg.transform.translation.x = float(center[0])
            tf_msg.transform.translation.y = float(center[1])
            tf_msg.transform.translation.z = float(center[2])
            tf_msg.transform.rotation.x = float(qx)
            tf_msg.transform.rotation.y = float(qy)
            tf_msg.transform.rotation.z = float(qz)
            tf_msg.transform.rotation.w = float(qw)
            self._tf_broadcaster.sendTransform(tf_msg)

        if self.publish_cube_markers:
            marker = Marker()
            marker.header.frame_id = self.workspace_frame
            marker.header.stamp = stamp
            marker.ns = f"holoassist_april_cube_{cube_idx + 1}"
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose_msg.pose
            marker.scale.x = float(self.cube_edge_size_m)
            marker.scale.y = float(self.cube_edge_size_m)
            marker.scale.z = float(self.cube_edge_size_m)
            color = self.cube_defs[cube_idx]["marker_color"]
            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])
            marker.color.a = max(0.05, min(1.0, float(color[3]) * float(self.marker_alpha)))
            self.marker_pubs[cube_idx].publish(marker)
            self._marker_active[cube_idx] = True

        return pose_msg

    def _publish_status(
        self,
        cube_idx: int,
        state: str,
        visible_ids: List[int],
        selected_tag_id: int,
        age_s: float,
        frame_id: str,
        center: Optional[np.ndarray],
        reason: str,
    ) -> None:
        cube_id = int(self.cube_defs[cube_idx]["cube_id"])
        if center is None:
            px, py, pz = float("nan"), float("nan"), float("nan")
        else:
            c = np.asarray(center, dtype=np.float64)
            px, py, pz = float(c[0]), float(c[1]), float(c[2])
        msg = String()
        msg.data = (
            "cube_id=%d state=%s reason=%s visible_tag_ids=%s selected_tag_id=%d "
            "age_s=%s frame_id=%s position_m=(%.3f,%.3f,%.3f)"
            % (
                cube_id,
                state,
                reason,
                visible_ids,
                selected_tag_id,
                "inf" if not math.isfinite(age_s) else f"{age_s:.3f}",
                frame_id,
                px,
                py,
                pz,
            )
        )
        self.status_pubs[cube_idx].publish(msg)

    def _delete_marker(self, cube_idx: int) -> None:
        if not self._marker_active[cube_idx]:
            return
        marker = Marker()
        marker.header.frame_id = self.workspace_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"holoassist_april_cube_{cube_idx + 1}"
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pubs[cube_idx].publish(marker)
        self._marker_active[cube_idx] = False

    def _read_tag_ids(self, primary_name: str, fallback_name: str) -> List[int]:
        primary = [int(v) for v in self.get_parameter(primary_name).value]
        fallback = [int(v) for v in self.get_parameter(fallback_name).value]
        if primary != fallback:
            self._log_throttled(
                "info",
                f"tag_ids_alias_{primary_name}",
                self.WARNING_THROTTLE_S,
                "using %s=%s (legacy %s=%s)"
                % (primary_name, primary, fallback_name, fallback),
            )
        return primary

    def _validate_configuration(self) -> None:
        if abs(self.cube_edge_size_m - self.EXPECTED_CUBE_EDGE_M) > 1e-6:
            self._log_throttled(
                "warn",
                "cube_edge_size_m_unexpected",
                self.WARNING_THROTTLE_S,
                "marker scale parameter cube_edge_size_m is %.6f m, expected %.3f m."
                % (self.cube_edge_size_m, self.EXPECTED_CUBE_EDGE_M),
            )
        if abs(self.cube_tag_size_m - self.EXPECTED_CUBE_TAG_SIZE_M) > 1e-6:
            self._log_throttled(
                "warn",
                "cube_tag_size_m_unexpected",
                self.WARNING_THROTTLE_S,
                "cube tag size parameter cube_tag_size_m is %.6f m, expected %.3f m."
                % (self.cube_tag_size_m, self.EXPECTED_CUBE_TAG_SIZE_M),
            )
        if abs(self.cube_face_offset_m - self.EXPECTED_CUBE_FACE_OFFSET_M) > 1e-6:
            self._log_throttled(
                "warn",
                "cube_face_offset_m_unexpected",
                self.WARNING_THROTTLE_S,
                "cube face offset parameter cube_face_offset_m is %.6f m, expected %.3f m."
                % (self.cube_face_offset_m, self.EXPECTED_CUBE_FACE_OFFSET_M),
            )
        expected_from_edge = 0.5 * float(self.cube_edge_size_m)
        if abs(expected_from_edge - self.cube_face_offset_m) > 1e-6:
            self._log_throttled(
                "warn",
                "cube_offset_vs_edge_mismatch",
                self.WARNING_THROTTLE_S,
                "cube_face_offset_m %.6f m is not half of cube_edge_size_m %.6f m."
                % (self.cube_face_offset_m, self.cube_edge_size_m),
            )
        if self._tag_overlap_reported:
            self._log_throttled(
                "warn",
                "cube_tag_overlap_summary",
                self.WARNING_THROTTLE_S,
                "one or more tag IDs appear in multiple cube groups; this can cause ambiguous tracking.",
            )

    def _warn_if_pose_unreasonable(self, cube_id: int, center: np.ndarray) -> None:
        c = np.asarray(center, dtype=np.float64)
        if abs(float(c[0])) > 1.0 or abs(float(c[1])) > 1.0 or float(c[2]) < -0.20:
            self._log_throttled(
                "warn",
                f"cube_{cube_id}_pose_outside_workspace",
                self.WARNING_THROTTLE_S,
                "cube_id=%d pose looks unreasonable for workspace: position=(%.3f,%.3f,%.3f)"
                % (cube_id, float(c[0]), float(c[1]), float(c[2])),
            )

    def _log_throttled(self, level: str, key: str, period_s: float, msg: str) -> None:
        now_s = float(self.get_clock().now().nanoseconds) / 1e9
        last_s = self._last_log_time.get(key, -1e12)
        if now_s - last_s < max(0.01, period_s):
            return
        self._last_log_time[key] = now_s
        if level == "warn":
            self.get_logger().warning(msg)
        elif level == "debug":
            self.get_logger().debug(msg)
        else:
            self.get_logger().info(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CubePoseNode()
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
