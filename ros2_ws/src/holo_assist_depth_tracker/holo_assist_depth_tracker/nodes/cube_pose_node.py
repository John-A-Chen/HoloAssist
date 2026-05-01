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
from holo_assist_depth_tracker.utils.math3d import quaternion_from_rotation_matrix, rotation_matrix_from_quaternion


class CubePoseNode(Node):
    """Estimate 4 physical cube centre poses from grouped AprilTag face IDs."""

    EXPECTED_CUBE_EDGE_M = 0.040
    EXPECTED_CUBE_TAG_SIZE_M = 0.032
    EXPECTED_CUBE_FACE_OFFSET_M = 0.020
    EXPECTED_CANDIDATE_SPREAD_THRESHOLD_M = 0.050
    WARNING_THROTTLE_S = 5.0
    DEBUG_THROTTLE_S = 1.0

    DEFAULT_FACE_ORDER = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
    FACE_LABEL_TO_AXIS = {
        "+X": np.array([1.0, 0.0, 0.0], dtype=np.float64),
        "-X": np.array([-1.0, 0.0, 0.0], dtype=np.float64),
        "+Y": np.array([0.0, 1.0, 0.0], dtype=np.float64),
        "-Y": np.array([0.0, -1.0, 0.0], dtype=np.float64),
        "+Z": np.array([0.0, 0.0, 1.0], dtype=np.float64),
        "-Z": np.array([0.0, 0.0, -1.0], dtype=np.float64),
    }

    CUBE_COLORS_RGBA = [
        (1.0, 0.0, 0.0, 0.75),
        (0.0, 1.0, 0.0, 0.75),
        (0.0, 0.3, 1.0, 0.75),
        (1.0, 0.7, 0.0, 0.75),
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
        self.declare_parameter("candidate_consensus_threshold_m", 0.05)

        self.declare_parameter("april_cube_1_tag_ids", [10, 11, 12, 13, 14, 15])
        self.declare_parameter("april_cube_2_tag_ids", [16, 17, 18, 19, 20, 21])
        self.declare_parameter("april_cube_3_tag_ids", [22, 23, 24, 25, 26, 27])
        self.declare_parameter("april_cube_4_tag_ids", [28, 29, 30, 31, 32, 33])
        self.declare_parameter("april_cube_face_order", self.DEFAULT_FACE_ORDER)
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
        self.candidate_consensus_threshold_m = max(
            0.001,
            float(self.get_parameter("candidate_consensus_threshold_m").value),
        )
        if abs(self.cube_edge_size_m - legacy_cube_size_m) > 1e-9:
            self._log_throttled(
                "warn",
                "legacy_cube_size_m_mismatch",
                self.WARNING_THROTTLE_S,
                "cube_size_m (legacy)=%.4f differs from cube_edge_size_m=%.4f; using cube_edge_size_m."
                % (legacy_cube_size_m, self.cube_edge_size_m),
            )
        self.cube_min_center_z_m = float(self.get_parameter("cube_min_center_z_m").value)
        self.detections_timeout_s = max(0.05, float(self.get_parameter("detections_timeout_s").value))
        self.tf_lookup_timeout_s = max(0.0, float(self.get_parameter("tf_lookup_timeout_s").value))
        self.timer_hz = max(1.0, float(self.get_parameter("timer_hz").value))
        self.publish_cube_tf = bool(self.get_parameter("publish_cube_tf").value)
        self.publish_cube_markers = bool(self.get_parameter("publish_cube_markers").value)
        self.marker_alpha = max(0.05, min(1.0, float(self.get_parameter("marker_alpha").value)))
        self.legacy_cube_pose_topic = str(self.get_parameter("legacy_cube_pose_topic").value)

        self.face_order = self._read_face_order()

        self.cube_groups: List[List[int]] = [
            self._read_tag_ids("april_cube_1_tag_ids", "cube_1_ids"),
            self._read_tag_ids("april_cube_2_tag_ids", "cube_2_ids"),
            self._read_tag_ids("april_cube_3_tag_ids", "cube_3_ids"),
            self._read_tag_ids("april_cube_4_tag_ids", "cube_4_ids"),
        ]
        self.cube_defs: List[Dict[str, object]] = []
        self._tag_to_cube: Dict[int, int] = {}
        self._tag_to_face: Dict[int, str] = {}
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
            tag_to_face = self._build_face_map_for_group(tag_ids)
            pose_topic = f"/holoassist/perception/april_cube_{n}_pose"
            marker_topic = f"/holoassist/perception/april_cube_{n}_marker"
            status_topic = f"/holoassist/perception/april_cube_{n}_status"
            tf_child = f"apriltag_cube_{n}"
            self.pose_pubs.append(self.create_publisher(PoseStamped, pose_topic, reliable_qos))
            self.marker_pubs.append(self.create_publisher(Marker, marker_topic, reliable_qos))
            self.status_pubs.append(self.create_publisher(String, status_topic, reliable_qos))
            self.cube_tf_frames.append(tf_child)
            self.cube_defs.append(
                {
                    "cube_id": n,
                    "tag_ids": tag_ids,
                    "tag_to_face": tag_to_face,
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
                if tag_id in self._tag_to_face and self._tag_to_face[tag_id] != tag_to_face[tag_id]:
                    self._tag_overlap_reported = True
                    self._log_throttled(
                        "warn",
                        f"face_overlap_{tag_id}",
                        self.WARNING_THROTTLE_S,
                        "tag id %d mapped to conflicting face labels %s and %s"
                        % (tag_id, self._tag_to_face[tag_id], tag_to_face[tag_id]),
                    )
                self._tag_to_face[tag_id] = tag_to_face[tag_id]

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

        self._marker_active: List[bool] = [False, False, False, False]
        self._last_centers: List[Optional[np.ndarray]] = [None, None, None, None]
        self._last_rotations: List[Optional[np.ndarray]] = [None, None, None, None]

        self._timer = self.create_timer(1.0 / self.timer_hz, self._on_timer)
        self._validate_configuration()

        self.get_logger().info(
            "cube pose node started detections=%s workspace_frame=%s cube_edge=%.3f cube_tag=%.3f "
            "cube_face_offset=%.3f face_order=%s groups=%s"
            % (
                self.detections_topic,
                self.workspace_frame,
                self.cube_edge_size_m,
                self.cube_tag_size_m,
                self.cube_face_offset_m,
                self.face_order,
                self.cube_groups,
            )
        )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self._latest_visible_ids = {int(det.id) for det in msg.detections}
        for tag_id in sorted(self._latest_visible_ids):
            if tag_id >= 10 and tag_id not in self._tag_to_cube:
                self._log_throttled(
                    "warn",
                    f"unassigned_tag_{tag_id}",
                    self.WARNING_THROTTLE_S,
                    "detected tag id %d is not assigned to any april_cube_N_tag_ids group" % tag_id,
                )

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        self._latest_detection_stamp = stamp

    def _on_timer(self) -> None:
        if self._latest_detection_stamp is None:
            for idx in range(4):
                self._publish_status(
                    cube_idx=idx,
                    state="stale",
                    visible_ids=[],
                    selected_tag_id=-1,
                    rejected_ids=[],
                    candidate_count=0,
                    candidate_spread_m=float("nan"),
                    method="waiting_for_detections",
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
                    cube_idx=idx,
                    state="stale",
                    visible_ids=visible_ids,
                    selected_tag_id=-1,
                    rejected_ids=[],
                    candidate_count=0,
                    candidate_spread_m=float("nan"),
                    method="detections_timeout",
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
                    "cube_id=%d state=stale visible_tag_ids=%s age_s=%.2f" % (cube_id, visible_ids, age_s),
                )
            return

        visible_pose_records: List[Tuple[int, PoseStamped, np.ndarray]] = []

        for idx, tag_ids in enumerate(self.cube_groups):
            cube_def = self.cube_defs[idx]
            cube_id = int(cube_def["cube_id"])
            detected_ids = sorted(int(tag_id) for tag_id in tag_ids if int(tag_id) in self._latest_visible_ids)
            observations = self._collect_observations(idx, tag_ids)
            visible_ids = sorted(int(obs["id"]) for obs in observations)
            if not observations:
                self._publish_status(
                    cube_idx=idx,
                    state="stale",
                    visible_ids=detected_ids,
                    selected_tag_id=-1,
                    rejected_ids=[],
                    candidate_count=0,
                    candidate_spread_m=float("nan"),
                    method="no_visible_cube_tags",
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
                    cube_idx=idx,
                    state="stale",
                    visible_ids=visible_ids,
                    selected_tag_id=-1,
                    rejected_ids=[],
                    candidate_count=0,
                    candidate_spread_m=float("nan"),
                    method="failed_to_solve_cube_pose",
                    age_s=age_s,
                    frame_id=self.workspace_frame,
                    center=self._last_centers[idx],
                    reason="failed_to_solve_cube_pose",
                )
                self._delete_marker(idx)
                continue

            center = np.asarray(solved["center"], dtype=np.float64)
            rot_w_c = np.asarray(solved["rotation"], dtype=np.float64)
            used_ids = sorted(int(v) for v in solved["used_ids"])
            rejected_ids = sorted(int(v) for v in solved["rejected_ids"])
            selected_tag_id = int(solved["selected_tag_id"])
            candidate_spread_m = float(solved["candidate_spread_m"])
            method = str(solved["method"])
            pos_resid_m = float(solved["pos_resid_m"])

            if self.cube_min_center_z_m >= 0.0:
                min_z = self.cube_min_center_z_m
            else:
                min_z = 0.5 * float(self.cube_edge_size_m)
            if float(center[2]) < float(min_z):
                center = center.copy()
                center[2] = float(min_z)

            qx, qy, qz, qw = quaternion_from_rotation_matrix(rot_w_c)
            pose = self._publish_cube_outputs(idx, center, (qx, qy, qz, qw))
            self._last_centers[idx] = np.asarray(center, dtype=np.float64)
            self._last_rotations[idx] = np.asarray(rot_w_c, dtype=np.float64)
            visible_pose_records.append((idx, pose, center))

            self._publish_status(
                cube_idx=idx,
                state="visible",
                visible_ids=used_ids,
                selected_tag_id=selected_tag_id,
                rejected_ids=rejected_ids,
                candidate_count=len(used_ids),
                candidate_spread_m=candidate_spread_m,
                method=method,
                age_s=age_s,
                frame_id=self.workspace_frame,
                center=center,
                reason="ok",
            )

            marker_color = cube_def["marker_color"]
            if candidate_spread_m > self.candidate_consensus_threshold_m:
                self._log_throttled(
                    "warn",
                    f"cube_{cube_id}_spread_warning",
                    self.WARNING_THROTTLE_S,
                    "cube_id=%d candidate spread %.4f m exceeds %.4f m, visible=%s rejected=%s selected_tag_id=%d"
                    % (
                        cube_id,
                        candidate_spread_m,
                        self.candidate_consensus_threshold_m,
                        used_ids,
                        rejected_ids,
                        selected_tag_id,
                    ),
                )
            self._log_throttled(
                "info",
                f"cube_{cube_id}_debug",
                self.DEBUG_THROTTLE_S,
                "cube_id=%d visible_tag_ids=%s selected_tag_id=%d rejected_tag_ids=%s center=(%.3f,%.3f,%.3f) "
                "candidate_spread_m=%.4f pos_resid_mm=%.1f method=%s tf_child=%s marker_topic=%s "
                "marker_scale=(%.3f,%.3f,%.3f) cube_color_rgba=(%.2f,%.2f,%.2f,%.2f)"
                % (
                    cube_id,
                    used_ids,
                    selected_tag_id,
                    rejected_ids,
                    float(center[0]),
                    float(center[1]),
                    float(center[2]),
                    candidate_spread_m,
                    1000.0 * pos_resid_m,
                    method,
                    str(cube_def["tf_child_frame"]),
                    str(cube_def["marker_topic"]),
                    self.cube_edge_size_m,
                    self.cube_edge_size_m,
                    self.cube_edge_size_m,
                    float(marker_color[0]),
                    float(marker_color[1]),
                    float(marker_color[2]),
                    float(marker_color[3]),
                ),
            )
            self._warn_if_pose_unreasonable(cube_id, center)

        if visible_pose_records:
            cube1_pose = next((pose for idx, pose, _ in visible_pose_records if idx == 0), None)
            if cube1_pose is not None:
                self.legacy_pose_pub.publish(cube1_pose)
            else:
                nearest = min(visible_pose_records, key=lambda item: float(np.linalg.norm(item[2])))
                self.legacy_pose_pub.publish(nearest[1])

    def _collect_observations(self, cube_idx: int, tag_ids: List[int]) -> List[Dict[str, object]]:
        observations: List[Dict[str, object]] = []
        visible_ordered = [tag_id for tag_id in tag_ids if tag_id in self._latest_visible_ids]
        cube_def = self.cube_defs[cube_idx]
        cube_id = int(cube_def["cube_id"])
        tag_to_face: Dict[int, str] = cube_def["tag_to_face"]  # type: ignore[assignment]

        for tag_id in visible_ordered:
            tf_msg = self._lookup_tag_transform(tag_id)
            if tf_msg is None:
                continue

            t = tf_msg.transform.translation
            r = tf_msg.transform.rotation
            point = np.array([float(t.x), float(t.y), float(t.z)], dtype=np.float64)
            rot_w_t = rotation_matrix_from_quaternion(float(r.x), float(r.y), float(r.z), float(r.w))
            tag_normal = rot_w_t[:, 2].astype(np.float64)
            face_label = tag_to_face.get(int(tag_id), self.face_order[0])
            local_offset = self._local_offset_for_face(face_label)
            candidate_center = point + (rot_w_t @ local_offset)

            observations.append(
                {
                    "id": int(tag_id),
                    "point": point,
                    "rot_w_t": rot_w_t,
                    "tag_normal": tag_normal,
                    "face_label": face_label,
                    "face_axis": self.FACE_LABEL_TO_AXIS[face_label],
                    "local_offset": local_offset,
                    "candidate_center": candidate_center,
                }
            )

            self._log_throttled(
                "info",
                f"cube_{cube_id}_tag_{tag_id}",
                self.DEBUG_THROTTLE_S,
                "detected_tag_id=%d mapped_cube_id=%d mapped_face_label=%s tag_center=(%.3f,%.3f,%.3f) "
                "tag_normal=(%.3f,%.3f,%.3f) local_offset_used=(%.3f,%.3f,%.3f) "
                "candidate_cube_center=(%.3f,%.3f,%.3f)"
                % (
                    int(tag_id),
                    cube_id,
                    face_label,
                    float(point[0]),
                    float(point[1]),
                    float(point[2]),
                    float(tag_normal[0]),
                    float(tag_normal[1]),
                    float(tag_normal[2]),
                    float(local_offset[0]),
                    float(local_offset[1]),
                    float(local_offset[2]),
                    float(candidate_center[0]),
                    float(candidate_center[1]),
                    float(candidate_center[2]),
                ),
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

    def _solve_pose(self, observations: List[Dict[str, object]], cube_idx: int) -> Optional[Dict[str, object]]:
        if not observations:
            return None

        candidate_centers = [np.asarray(obs["candidate_center"], dtype=np.float64) for obs in observations]
        consensus_idx, rejected_idx, spread_m = self._select_consensus_indices(
            centers=candidate_centers,
            threshold_m=self.candidate_consensus_threshold_m,
            last_center=self._last_centers[cube_idx],
        )
        if not consensus_idx:
            return None

        used_observations = [observations[i] for i in consensus_idx]
        used_centers = [candidate_centers[i] for i in consensus_idx]
        used_ids = [int(obs["id"]) for obs in used_observations]
        rejected_ids = [int(observations[i]["id"]) for i in rejected_idx]

        center = np.mean(np.asarray(used_centers, dtype=np.float64), axis=0)
        pos_resid_m = float(np.mean([np.linalg.norm(c - center) for c in used_centers]))

        selected_local_idx = self._select_reliable_candidate_idx(used_centers, center, self._last_centers[cube_idx])
        selected_obs = used_observations[selected_local_idx]
        selected_tag_id = int(selected_obs["id"])

        if len(used_observations) == 1:
            rot_w_c = np.asarray(selected_obs["rot_w_t"], dtype=np.float64)
            method = "single_tag_candidate"
        elif self._last_rotations[cube_idx] is not None:
            rot_w_c = np.asarray(self._last_rotations[cube_idx], dtype=np.float64)
            method = "consensus_center_keep_previous_orientation"
        else:
            rot_w_c = np.asarray(selected_obs["rot_w_t"], dtype=np.float64)
            method = "consensus_center_selected_tag_orientation"

        if rejected_ids:
            self._log_throttled(
                "warn",
                f"cube_{cube_idx + 1}_rejected_tags",
                self.WARNING_THROTTLE_S,
                "cube_id=%d rejected tag IDs due to centre disagreement: %s (used=%s spread_m=%.4f threshold_m=%.4f)"
                % (
                    cube_idx + 1,
                    rejected_ids,
                    used_ids,
                    spread_m,
                    self.candidate_consensus_threshold_m,
                ),
            )

        return {
            "center": center,
            "rotation": rot_w_c,
            "used_ids": used_ids,
            "rejected_ids": rejected_ids,
            "selected_tag_id": selected_tag_id,
            "candidate_spread_m": spread_m,
            "method": method,
            "pos_resid_m": pos_resid_m,
        }

    def _select_consensus_indices(
        self,
        centers: List[np.ndarray],
        threshold_m: float,
        last_center: Optional[np.ndarray],
    ) -> Tuple[List[int], List[int], float]:
        n = len(centers)
        if n == 0:
            return [], [], float("nan")
        if n == 1:
            return [0], [], 0.0

        dist = np.zeros((n, n), dtype=np.float64)
        for i in range(n):
            for j in range(i + 1, n):
                d = float(np.linalg.norm(centers[i] - centers[j]))
                dist[i, j] = d
                dist[j, i] = d

        best_inliers: List[int] = []
        best_score = (-1, -1e9, -1e9)

        for i in range(n):
            inliers = [j for j in range(n) if float(dist[i, j]) <= threshold_m]
            inlier_arr = np.asarray([centers[j] for j in inliers], dtype=np.float64)
            mean_center = np.mean(inlier_arr, axis=0)
            spread = float(np.max([np.linalg.norm(centers[j] - mean_center) for j in inliers]))
            if last_center is not None:
                stability = -float(np.linalg.norm(mean_center - np.asarray(last_center, dtype=np.float64)))
            else:
                stability = -float(np.linalg.norm(mean_center))
            score = (len(inliers), -spread, stability)
            if score > best_score:
                best_score = score
                best_inliers = inliers

        if not best_inliers:
            best_inliers = [0]

        inlier_set = set(best_inliers)
        rejected = [idx for idx in range(n) if idx not in inlier_set]
        used_arr = np.asarray([centers[idx] for idx in best_inliers], dtype=np.float64)
        used_center = np.mean(used_arr, axis=0)
        spread = float(np.max([np.linalg.norm(centers[idx] - used_center) for idx in best_inliers]))
        if len(best_inliers) == 1 and n > 1:
            spread = float(np.max([np.linalg.norm(c - used_center) for c in centers]))

        return sorted(best_inliers), rejected, spread

    def _select_reliable_candidate_idx(
        self,
        centers: List[np.ndarray],
        mean_center: np.ndarray,
        last_center: Optional[np.ndarray],
    ) -> int:
        if not centers:
            return 0
        if last_center is not None:
            dists = [float(np.linalg.norm(c - np.asarray(last_center, dtype=np.float64))) for c in centers]
            return int(min(range(len(centers)), key=lambda idx: dists[idx]))
        dists = [float(np.linalg.norm(c - mean_center)) for c in centers]
        return int(min(range(len(centers)), key=lambda idx: dists[idx]))

    def _build_face_map_for_group(self, tag_ids: List[int]) -> Dict[int, str]:
        mapping: Dict[int, str] = {}
        for idx, tag_id in enumerate(tag_ids):
            face_label = self.face_order[min(idx, len(self.face_order) - 1)]
            mapping[int(tag_id)] = face_label
        return mapping

    def _read_face_order(self) -> List[str]:
        raw = [str(v).strip().upper() for v in self.get_parameter("april_cube_face_order").value]
        if len(raw) != 6:
            self._log_throttled(
                "warn",
                "face_order_length",
                self.WARNING_THROTTLE_S,
                "april_cube_face_order must have 6 labels; got %d. Falling back to default %s"
                % (len(raw), self.DEFAULT_FACE_ORDER),
            )
            return list(self.DEFAULT_FACE_ORDER)

        output: List[str] = []
        for idx, label in enumerate(raw):
            if label not in self.FACE_LABEL_TO_AXIS:
                self._log_throttled(
                    "warn",
                    f"face_order_invalid_{idx}",
                    self.WARNING_THROTTLE_S,
                    "april_cube_face_order[%d]=%s invalid. Using default %s"
                    % (idx, label, self.DEFAULT_FACE_ORDER[idx]),
                )
                output.append(self.DEFAULT_FACE_ORDER[idx])
            else:
                output.append(label)
        return output

    def _local_offset_for_face(self, face_label: str) -> np.ndarray:
        f = float(self.cube_face_offset_m)
        if face_label == "+X":
            return np.array([-f, 0.0, 0.0], dtype=np.float64)
        if face_label == "-X":
            return np.array([f, 0.0, 0.0], dtype=np.float64)
        if face_label == "+Y":
            return np.array([0.0, -f, 0.0], dtype=np.float64)
        if face_label == "-Y":
            return np.array([0.0, f, 0.0], dtype=np.float64)
        if face_label == "+Z":
            return np.array([0.0, 0.0, -f], dtype=np.float64)
        if face_label == "-Z":
            return np.array([0.0, 0.0, f], dtype=np.float64)
        return np.array([0.0, 0.0, -f], dtype=np.float64)

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
        rejected_ids: List[int],
        candidate_count: int,
        candidate_spread_m: float,
        method: str,
        age_s: float,
        frame_id: str,
        center: Optional[np.ndarray],
        reason: str,
    ) -> None:
        cube_def = self.cube_defs[cube_idx]
        cube_id = int(cube_def["cube_id"])
        color = cube_def["marker_color"]
        if center is None:
            px, py, pz = float("nan"), float("nan"), float("nan")
        else:
            c = np.asarray(center, dtype=np.float64)
            px, py, pz = float(c[0]), float(c[1]), float(c[2])

        spread_text = "nan" if not math.isfinite(candidate_spread_m) else f"{candidate_spread_m:.4f}"
        msg = String()
        msg.data = (
            "cube_id=%d state=%s reason=%s visible_tag_ids=%s selected_tag_id=%d rejected_tag_ids=%s "
            "candidate_count=%d candidate_spread_m=%s method=%s age_s=%s frame_id=%s "
            "position_m=(%.3f,%.3f,%.3f) marker_scale_m=(%.3f,%.3f,%.3f) color_rgba=(%.2f,%.2f,%.2f,%.2f)"
            % (
                cube_id,
                state,
                reason,
                visible_ids,
                selected_tag_id,
                rejected_ids,
                candidate_count,
                spread_text,
                method,
                "inf" if not math.isfinite(age_s) else f"{age_s:.3f}",
                frame_id,
                px,
                py,
                pz,
                self.cube_edge_size_m,
                self.cube_edge_size_m,
                self.cube_edge_size_m,
                float(color[0]),
                float(color[1]),
                float(color[2]),
                float(color[3]),
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
                "using %s=%s (legacy %s=%s)" % (primary_name, primary, fallback_name, fallback),
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
        if abs(self.candidate_consensus_threshold_m - self.EXPECTED_CANDIDATE_SPREAD_THRESHOLD_M) > 1e-6:
            self._log_throttled(
                "warn",
                "candidate_consensus_threshold_m_unexpected",
                self.WARNING_THROTTLE_S,
                "candidate_consensus_threshold_m is %.6f m, expected %.3f m."
                % (
                    self.candidate_consensus_threshold_m,
                    self.EXPECTED_CANDIDATE_SPREAD_THRESHOLD_M,
                ),
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

        for idx, tags in enumerate(self.cube_groups):
            if len(tags) != 6:
                self._log_throttled(
                    "warn",
                    f"cube_{idx + 1}_tag_count",
                    self.WARNING_THROTTLE_S,
                    "cube_%d has %d tags configured, expected 6 tags." % (idx + 1, len(tags)),
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
