#!/usr/bin/env python3

from __future__ import annotations

import math
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker


def _normalize(vec: np.ndarray) -> Optional[np.ndarray]:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-9:
        return None
    return vec / norm


def _quat_from_rotation_matrix(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    """Return quaternion (x,y,z,w) from 3x3 rotation matrix."""
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


class WorkspacePerceptionNode(Node):
    """Bench-plane workspace reasoning adapter from depth pointcloud to object pose topics."""

    def __init__(self) -> None:
        super().__init__("holoassist_workspace_perception")

        self._declare_parameters()
        self._load_parameters()

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pc_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self.workspace_mode_pub = self.create_publisher(
            String, self.workspace_mode_topic, reliable_qos
        )
        self.workspace_diag_pub = self.create_publisher(
            DiagnosticArray, self.workspace_diag_topic, reliable_qos
        )
        self.plane_coeff_pub = self.create_publisher(
            Float32MultiArray, self.plane_coeff_topic, reliable_qos
        )
        self.object_metrics_pub = self.create_publisher(
            Float32MultiArray, self.object_metrics_topic, reliable_qos
        )

        self.plane_marker_pub = self.create_publisher(
            Marker, self.plane_marker_topic, reliable_qos
        )
        self.tag_marker_pub = self.create_publisher(
            Marker, self.tag_marker_topic, reliable_qos
        )
        self.axes_marker_pub = self.create_publisher(
            Marker, self.axes_marker_topic, reliable_qos
        )
        self.object_marker_pub = self.create_publisher(
            Marker, self.object_marker_topic, reliable_qos
        )

        self.cropped_cloud_pub = self.create_publisher(
            PointCloud2, self.cropped_cloud_topic, reliable_qos
        )
        self.cropped_cloud_workspace_pub = self.create_publisher(
            PointCloud2, self.cropped_cloud_workspace_topic, reliable_qos
        )
        self.foreground_cloud_pub = self.create_publisher(
            PointCloud2, self.foreground_cloud_topic, reliable_qos
        )
        self.foreground_cloud_workspace_pub = self.create_publisher(
            PointCloud2, self.foreground_cloud_workspace_topic, reliable_qos
        )

        self.object_pose_pub = self.create_publisher(
            PoseStamped, self.object_pose_topic, reliable_qos
        )
        self.object_pose_workspace_pub = self.create_publisher(
            PoseStamped, self.object_pose_workspace_topic, reliable_qos
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.input_pointcloud_topic,
            self._on_pointcloud,
            qos_profile_sensor_data,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._rng = np.random.default_rng(self.random_seed)
        self._neighbor_offsets = [
            (dx, dy, dz)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
            if not (dx == 0 and dy == 0 and dz == 0)
        ]

        self._frame_count = 0
        self._last_mode = "invalid"
        self._last_origin_c: Optional[np.ndarray] = None
        self._last_basis_c: Optional[np.ndarray] = None  # columns=[x,y,z] in cloud frame
        self._last_plane: Optional[Tuple[float, float, float, float]] = None

        self._background_scores: Dict[Tuple[int, int, int], float] = {}
        self._last_object_center_w: Optional[np.ndarray] = None
        self._object_persistence_score: float = 0.0
        self._last_published_center_w: Optional[np.ndarray] = None
        self._last_published_center_c: Optional[np.ndarray] = None
        self._last_published_extent_w: Optional[np.ndarray] = None
        self._tracks: Dict[int, Dict[str, object]] = {}
        self._next_track_id: int = 1
        self._last_marker_ids: set[int] = set()
        self._last_primary_track_id: Optional[int] = None
        self._active_roi_x_min = self.roi_x_min
        self._active_roi_x_max = self.roi_x_max
        self._active_roi_y_min = self.roi_y_min
        self._active_roi_y_max = self.roi_y_max

        self.get_logger().info(
            "Workspace perception started. input=%s workspace_frame=%s tags=%s"
            % (
                self.input_pointcloud_topic,
                self.workspace_frame,
                self.tag_frames,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "input_pointcloud_topic", "/holo_assist_depth_tracker/pointcloud"
        )
        self.declare_parameter("workspace_frame", "workspace_frame")

        self.declare_parameter("use_tag_refinement", True)
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("tag_ids", [0, 1])
        self.declare_parameter("tag_frame_names", ["tag36h11:0", "tag36h11:1"])
        self.declare_parameter("tag_size_m", 0.035)
        self.declare_parameter("tag_timeout_s", 0.5)
        self.declare_parameter("tag_min_separation_m", 0.08)

        self.declare_parameter("plane_fit_max_points", 18000)
        self.declare_parameter("plane_ransac_iterations", 140)
        self.declare_parameter("plane_inlier_threshold_m", 0.015)
        self.declare_parameter("plane_min_inlier_ratio", 0.35)
        self.declare_parameter("plane_min_inliers", 600)
        self.declare_parameter("plane_smoothing_alpha", 0.25)

        self.declare_parameter("roi_x_min", -0.45)
        self.declare_parameter("roi_x_max", 0.45)
        self.declare_parameter("roi_y_min", -0.35)
        self.declare_parameter("roi_y_max", 0.35)
        self.declare_parameter("roi_z_min", 0.005)
        self.declare_parameter("roi_z_max", 0.35)
        self.declare_parameter("roi_from_tags_enabled", True)
        self.declare_parameter("roi_from_tags_margin_m", 0.03)
        self.declare_parameter("roi_from_tags_min_span_m", 0.12)
        self.declare_parameter("roi_from_tags_smoothing_alpha", 0.35)
        self.declare_parameter("enforce_above_plane_cull", True)
        self.declare_parameter("below_plane_tolerance_m", 0.0)

        self.declare_parameter("pedestal_exclusion_enabled", False)
        self.declare_parameter("pedestal_center_x", 0.0)
        self.declare_parameter("pedestal_center_y", 0.0)
        self.declare_parameter("pedestal_radius", 0.08)
        self.declare_parameter("pedestal_z_min", -0.05)
        self.declare_parameter("pedestal_z_max", 0.50)

        self.declare_parameter("max_points_per_frame", 30000)

        self.declare_parameter("background_voxel_size", 0.015)
        self.declare_parameter("background_increment", 0.04)
        self.declare_parameter("background_decay", 0.01)
        self.declare_parameter("background_threshold", 0.65)
        self.declare_parameter("background_warmup_frames", 20)
        self.declare_parameter("background_max_points", 20000)

        self.declare_parameter("cluster_voxel_size", 0.02)
        self.declare_parameter("min_foreground_points", 80)
        self.declare_parameter("min_cluster_points", 80)
        self.declare_parameter("max_cluster_points", 50000)
        self.declare_parameter("object_candidate_max_extent_x_m", 0.18)
        self.declare_parameter("object_candidate_max_extent_y_m", 0.18)
        self.declare_parameter("object_candidate_max_extent_z_m", 0.18)
        self.declare_parameter("object_candidate_max_volume_m3", 0.0035)
        self.declare_parameter("persistence_distance_m", 0.09)
        self.declare_parameter("persistence_gain", 0.25)
        self.declare_parameter("persistence_decay", 0.85)
        self.declare_parameter("centroid_update_closer_only", True)
        self.declare_parameter("centroid_min_closer_m", 0.03)
        self.declare_parameter("centroid_smoothing_alpha", 0.25)
        self.declare_parameter("max_tracked_objects", 6)
        self.declare_parameter("track_match_distance_m", 0.12)
        self.declare_parameter("track_spawn_deadzone_m", 0.08)
        self.declare_parameter("track_smoothing_alpha", 0.35)
        self.declare_parameter("track_max_missed_frames", 8)

        self.declare_parameter(
            "cropped_cloud_topic", "/holoassist/perception/cropped_pointcloud"
        )
        self.declare_parameter(
            "cropped_cloud_workspace_topic",
            "/holoassist/perception/cropped_pointcloud_workspace",
        )
        self.declare_parameter(
            "foreground_cloud_topic", "/holoassist/perception/foreground_pointcloud"
        )
        self.declare_parameter(
            "foreground_cloud_workspace_topic",
            "/holoassist/perception/foreground_pointcloud_workspace",
        )

        self.declare_parameter(
            "object_pose_topic", "/holoassist/perception/object_pose"
        )
        self.declare_parameter(
            "object_pose_workspace_topic", "/holoassist/perception/object_pose_workspace"
        )
        self.declare_parameter(
            "object_marker_topic", "/holoassist/perception/object_marker"
        )

        self.declare_parameter(
            "plane_marker_topic", "/holoassist/perception/bench_plane_marker"
        )
        self.declare_parameter(
            "tag_marker_topic", "/holoassist/perception/workspace_tag_markers"
        )
        self.declare_parameter(
            "axes_marker_topic", "/holoassist/perception/workspace_axes_marker"
        )
        self.declare_parameter(
            "plane_coeff_topic", "/holoassist/perception/bench_plane_coefficients"
        )
        self.declare_parameter(
            "workspace_mode_topic", "/holoassist/perception/workspace_mode"
        )
        self.declare_parameter(
            "workspace_diag_topic", "/holoassist/perception/workspace_diagnostics"
        )
        self.declare_parameter(
            "object_metrics_topic", "/holoassist/perception/object_metrics"
        )

        self.declare_parameter("publish_workspace_clouds", True)
        self.declare_parameter("publish_foreground_clouds", True)
        self.declare_parameter("publish_debug_markers", True)
        self.declare_parameter("publish_tf", True)

        self.declare_parameter("marker_alpha", 0.35)
        self.declare_parameter("plane_marker_thickness_m", 0.006)
        self.declare_parameter("axes_marker_length_m", 0.12)
        self.declare_parameter("axes_marker_radius_m", 0.01)
        self.declare_parameter("object_marker_min_extent_m", 0.02)
        self.declare_parameter("object_marker_max_extent_m", 0.18)

        self.declare_parameter("random_seed", 42)

    def _load_parameters(self) -> None:
        self.input_pointcloud_topic = str(
            self.get_parameter("input_pointcloud_topic").value
        )
        self.workspace_frame = str(self.get_parameter("workspace_frame").value)

        self.use_tag_refinement = bool(self.get_parameter("use_tag_refinement").value)
        self.tag_family = str(self.get_parameter("tag_family").value)
        self.tag_ids = [int(v) for v in self.get_parameter("tag_ids").value]
        self.tag_frame_names = [
            str(v).strip()
            for v in self.get_parameter("tag_frame_names").value
            if str(v).strip()
        ]
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        self.tag_timeout_s = float(self.get_parameter("tag_timeout_s").value)
        self.tag_min_separation_m = float(
            self.get_parameter("tag_min_separation_m").value
        )

        self.plane_fit_max_points = int(
            self.get_parameter("plane_fit_max_points").value
        )
        self.plane_ransac_iterations = int(
            self.get_parameter("plane_ransac_iterations").value
        )
        self.plane_inlier_threshold_m = float(
            self.get_parameter("plane_inlier_threshold_m").value
        )
        self.plane_min_inlier_ratio = float(
            self.get_parameter("plane_min_inlier_ratio").value
        )
        self.plane_min_inliers = int(self.get_parameter("plane_min_inliers").value)
        self.plane_smoothing_alpha = float(
            self.get_parameter("plane_smoothing_alpha").value
        )

        self.roi_x_min = float(self.get_parameter("roi_x_min").value)
        self.roi_x_max = float(self.get_parameter("roi_x_max").value)
        self.roi_y_min = float(self.get_parameter("roi_y_min").value)
        self.roi_y_max = float(self.get_parameter("roi_y_max").value)
        self.roi_z_min = float(self.get_parameter("roi_z_min").value)
        self.roi_z_max = float(self.get_parameter("roi_z_max").value)
        self.roi_from_tags_enabled = bool(
            self.get_parameter("roi_from_tags_enabled").value
        )
        self.roi_from_tags_margin_m = float(
            self.get_parameter("roi_from_tags_margin_m").value
        )
        self.roi_from_tags_min_span_m = float(
            self.get_parameter("roi_from_tags_min_span_m").value
        )
        self.roi_from_tags_smoothing_alpha = float(
            self.get_parameter("roi_from_tags_smoothing_alpha").value
        )
        self.enforce_above_plane_cull = bool(
            self.get_parameter("enforce_above_plane_cull").value
        )
        self.below_plane_tolerance_m = float(
            self.get_parameter("below_plane_tolerance_m").value
        )

        self.pedestal_exclusion_enabled = bool(
            self.get_parameter("pedestal_exclusion_enabled").value
        )
        self.pedestal_center_x = float(self.get_parameter("pedestal_center_x").value)
        self.pedestal_center_y = float(self.get_parameter("pedestal_center_y").value)
        self.pedestal_radius = float(self.get_parameter("pedestal_radius").value)
        self.pedestal_z_min = float(self.get_parameter("pedestal_z_min").value)
        self.pedestal_z_max = float(self.get_parameter("pedestal_z_max").value)

        self.max_points_per_frame = int(self.get_parameter("max_points_per_frame").value)

        self.background_voxel_size = float(
            self.get_parameter("background_voxel_size").value
        )
        self.background_increment = float(
            self.get_parameter("background_increment").value
        )
        self.background_decay = float(self.get_parameter("background_decay").value)
        self.background_threshold = float(
            self.get_parameter("background_threshold").value
        )
        self.background_warmup_frames = int(
            self.get_parameter("background_warmup_frames").value
        )
        self.background_max_points = int(
            self.get_parameter("background_max_points").value
        )

        self.cluster_voxel_size = float(self.get_parameter("cluster_voxel_size").value)
        self.min_foreground_points = int(
            self.get_parameter("min_foreground_points").value
        )
        self.min_cluster_points = int(self.get_parameter("min_cluster_points").value)
        self.max_cluster_points = int(self.get_parameter("max_cluster_points").value)
        self.object_candidate_max_extent_x_m = float(
            self.get_parameter("object_candidate_max_extent_x_m").value
        )
        self.object_candidate_max_extent_y_m = float(
            self.get_parameter("object_candidate_max_extent_y_m").value
        )
        self.object_candidate_max_extent_z_m = float(
            self.get_parameter("object_candidate_max_extent_z_m").value
        )
        self.object_candidate_max_volume_m3 = float(
            self.get_parameter("object_candidate_max_volume_m3").value
        )
        self.persistence_distance_m = float(
            self.get_parameter("persistence_distance_m").value
        )
        self.persistence_gain = float(self.get_parameter("persistence_gain").value)
        self.persistence_decay = float(self.get_parameter("persistence_decay").value)
        self.centroid_update_closer_only = bool(
            self.get_parameter("centroid_update_closer_only").value
        )
        self.centroid_min_closer_m = float(
            self.get_parameter("centroid_min_closer_m").value
        )
        self.centroid_smoothing_alpha = float(
            self.get_parameter("centroid_smoothing_alpha").value
        )
        self.max_tracked_objects = int(self.get_parameter("max_tracked_objects").value)
        self.track_match_distance_m = float(
            self.get_parameter("track_match_distance_m").value
        )
        self.track_spawn_deadzone_m = float(
            self.get_parameter("track_spawn_deadzone_m").value
        )
        self.track_smoothing_alpha = float(
            self.get_parameter("track_smoothing_alpha").value
        )
        self.track_max_missed_frames = int(
            self.get_parameter("track_max_missed_frames").value
        )

        self.cropped_cloud_topic = str(self.get_parameter("cropped_cloud_topic").value)
        self.cropped_cloud_workspace_topic = str(
            self.get_parameter("cropped_cloud_workspace_topic").value
        )
        self.foreground_cloud_topic = str(
            self.get_parameter("foreground_cloud_topic").value
        )
        self.foreground_cloud_workspace_topic = str(
            self.get_parameter("foreground_cloud_workspace_topic").value
        )

        self.object_pose_topic = str(self.get_parameter("object_pose_topic").value)
        self.object_pose_workspace_topic = str(
            self.get_parameter("object_pose_workspace_topic").value
        )
        self.object_marker_topic = str(self.get_parameter("object_marker_topic").value)
        self.plane_marker_topic = str(self.get_parameter("plane_marker_topic").value)
        self.tag_marker_topic = str(self.get_parameter("tag_marker_topic").value)
        self.axes_marker_topic = str(self.get_parameter("axes_marker_topic").value)
        self.plane_coeff_topic = str(self.get_parameter("plane_coeff_topic").value)
        self.workspace_mode_topic = str(self.get_parameter("workspace_mode_topic").value)
        self.workspace_diag_topic = str(self.get_parameter("workspace_diag_topic").value)
        self.object_metrics_topic = str(self.get_parameter("object_metrics_topic").value)

        self.publish_workspace_clouds = bool(
            self.get_parameter("publish_workspace_clouds").value
        )
        self.publish_foreground_clouds = bool(
            self.get_parameter("publish_foreground_clouds").value
        )
        self.publish_debug_markers = bool(
            self.get_parameter("publish_debug_markers").value
        )
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.marker_alpha = float(self.get_parameter("marker_alpha").value)
        self.plane_marker_thickness_m = float(
            self.get_parameter("plane_marker_thickness_m").value
        )
        self.axes_marker_length_m = float(
            self.get_parameter("axes_marker_length_m").value
        )
        self.axes_marker_radius_m = float(
            self.get_parameter("axes_marker_radius_m").value
        )
        self.object_marker_min_extent_m = float(
            self.get_parameter("object_marker_min_extent_m").value
        )
        self.object_marker_max_extent_m = float(
            self.get_parameter("object_marker_max_extent_m").value
        )

        self.random_seed = int(self.get_parameter("random_seed").value)

        if self.tag_frame_names:
            self.tag_frames = self.tag_frame_names[:2]
        else:
            self.tag_frames = [f"tag{self.tag_family}:{tag_id}" for tag_id in self.tag_ids[:2]]

        if self.roi_x_max <= self.roi_x_min:
            self.roi_x_min, self.roi_x_max = -0.45, 0.45
        if self.roi_y_max <= self.roi_y_min:
            self.roi_y_min, self.roi_y_max = -0.35, 0.35
        if self.roi_z_max <= self.roi_z_min:
            self.roi_z_min, self.roi_z_max = 0.005, 0.35
        self.roi_from_tags_margin_m = max(0.0, self.roi_from_tags_margin_m)
        self.roi_from_tags_min_span_m = max(0.01, self.roi_from_tags_min_span_m)
        self.roi_from_tags_smoothing_alpha = min(
            max(self.roi_from_tags_smoothing_alpha, 0.0), 1.0
        )
        self.below_plane_tolerance_m = max(0.0, self.below_plane_tolerance_m)

        self.plane_fit_max_points = max(200, self.plane_fit_max_points)
        self.plane_ransac_iterations = max(20, self.plane_ransac_iterations)
        self.plane_inlier_threshold_m = max(0.001, self.plane_inlier_threshold_m)
        self.plane_min_inlier_ratio = min(max(self.plane_min_inlier_ratio, 0.05), 0.99)
        self.plane_min_inliers = max(100, self.plane_min_inliers)
        self.plane_smoothing_alpha = min(max(self.plane_smoothing_alpha, 0.0), 1.0)

        self.background_voxel_size = max(0.005, self.background_voxel_size)
        self.background_increment = min(max(self.background_increment, 0.001), 1.0)
        self.background_decay = min(max(self.background_decay, 0.0), 1.0)
        self.background_threshold = min(max(self.background_threshold, 0.1), 0.95)
        self.background_warmup_frames = max(0, self.background_warmup_frames)
        self.background_max_points = max(200, self.background_max_points)

        self.cluster_voxel_size = max(0.005, self.cluster_voxel_size)
        self.min_foreground_points = max(1, self.min_foreground_points)
        self.min_cluster_points = max(1, self.min_cluster_points)
        self.max_cluster_points = max(self.min_cluster_points, self.max_cluster_points)
        self.object_candidate_max_extent_x_m = max(
            self.object_marker_min_extent_m, self.object_candidate_max_extent_x_m
        )
        self.object_candidate_max_extent_y_m = max(
            self.object_marker_min_extent_m, self.object_candidate_max_extent_y_m
        )
        self.object_candidate_max_extent_z_m = max(
            self.object_marker_min_extent_m, self.object_candidate_max_extent_z_m
        )
        self.object_candidate_max_volume_m3 = max(1e-5, self.object_candidate_max_volume_m3)
        self.object_marker_max_extent_m = max(
            self.object_marker_min_extent_m, self.object_marker_max_extent_m
        )
        self.centroid_min_closer_m = max(0.0, self.centroid_min_closer_m)
        self.centroid_smoothing_alpha = min(max(self.centroid_smoothing_alpha, 0.0), 1.0)
        self.max_tracked_objects = max(1, self.max_tracked_objects)
        self.track_match_distance_m = max(0.01, self.track_match_distance_m)
        self.track_spawn_deadzone_m = max(0.0, self.track_spawn_deadzone_m)
        self.track_smoothing_alpha = min(max(self.track_smoothing_alpha, 0.0), 1.0)
        self.track_max_missed_frames = max(0, self.track_max_missed_frames)

        self.max_points_per_frame = max(500, self.max_points_per_frame)
        self.tag_size_m = max(0.01, self.tag_size_m)

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        self._frame_count += 1

        xyz_all = self._extract_xyz(msg)
        if xyz_all is None or xyz_all.shape[0] < self.plane_min_inliers:
            self._tracks.clear()
            self._last_primary_track_id = None
            self._publish_workspace_state(
                mode="invalid",
                level=DiagnosticStatus.ERROR,
                message="pointcloud has too few points",
                extra={"points": 0 if xyz_all is None else int(xyz_all.shape[0])},
            )
            self._publish_object_delete(msg.header.frame_id, msg.header.stamp)
            return

        if xyz_all.shape[0] > self.max_points_per_frame:
            idx = self._rng.choice(xyz_all.shape[0], self.max_points_per_frame, replace=False)
            xyz = xyz_all[idx]
        else:
            xyz = xyz_all

        plane_fit = self._fit_bench_plane(xyz)
        if plane_fit is None:
            self._tracks.clear()
            self._last_primary_track_id = None
            self._publish_workspace_state(
                mode="invalid",
                level=DiagnosticStatus.ERROR,
                message="plane fit failed",
                extra={"points": int(xyz.shape[0])},
            )
            self._publish_object_delete(msg.header.frame_id, msg.header.stamp)
            return

        normal, d, inlier_mask, inlier_ratio, inlier_count, plane_origin = plane_fit

        tag_points_c = self._lookup_tag_points(msg.header.frame_id)
        basis = self._compute_workspace_basis(
            plane_normal=normal,
            plane_origin=plane_origin,
            tag_points_c=tag_points_c,
        )
        if basis is None:
            self._tracks.clear()
            self._last_primary_track_id = None
            self._publish_workspace_state(
                mode="invalid",
                level=DiagnosticStatus.ERROR,
                message="could not derive workspace basis",
                extra={"inliers": inlier_count, "inlier_ratio": round(inlier_ratio, 3)},
            )
            self._publish_object_delete(msg.header.frame_id, msg.header.stamp)
            return

        basis_c, mode = basis
        origin_c = plane_origin.copy()

        if self._last_origin_c is not None and self._last_basis_c is not None:
            alpha = self.plane_smoothing_alpha
            if alpha > 0.0:
                origin_c = (1.0 - alpha) * self._last_origin_c + alpha * origin_c
                z_axis = _normalize((1.0 - alpha) * self._last_basis_c[:, 2] + alpha * basis_c[:, 2])
                x_axis = _normalize((1.0 - alpha) * self._last_basis_c[:, 0] + alpha * basis_c[:, 0])
                if z_axis is not None and x_axis is not None:
                    y_axis = _normalize(np.cross(z_axis, x_axis))
                    if y_axis is not None:
                        x_axis = _normalize(np.cross(y_axis, z_axis))
                        if x_axis is not None:
                            basis_c = np.column_stack((x_axis, y_axis, z_axis))

        self._last_origin_c = origin_c
        self._last_basis_c = basis_c
        self._last_plane = (float(normal[0]), float(normal[1]), float(normal[2]), float(d))

        if self.publish_tf:
            self._broadcast_workspace_tf(
                parent_frame=msg.header.frame_id,
                stamp=msg.header.stamp,
                origin_c=origin_c,
                basis_c=basis_c,
            )

        self._publish_plane_coefficients(normal, d)
        roi_source = self._compute_active_roi_bounds(tag_points_c, origin_c, basis_c)
        if self.publish_debug_markers:
            self._publish_plane_marker(msg.header.stamp)
            self._publish_axes_markers(msg.header.stamp)
            self._publish_tag_markers(msg.header.stamp, tag_points_c, origin_c, basis_c)

        points_w = self._to_workspace(xyz, origin_c, basis_c)
        above_plane_mask = self._above_plane_mask(xyz, normal, d)
        roi_mask = self._workspace_roi_mask(points_w) & above_plane_mask

        cropped_points_c = xyz[roi_mask]
        cropped_points_w = points_w[roi_mask]
        culled_below_plane = int(np.count_nonzero(~above_plane_mask))

        self._publish_cloud(
            pub=self.cropped_cloud_pub,
            points=cropped_points_c,
            frame_id=msg.header.frame_id,
            stamp=msg.header.stamp,
        )
        if self.publish_workspace_clouds:
            self._publish_cloud(
                pub=self.cropped_cloud_workspace_pub,
                points=cropped_points_w,
                frame_id=self.workspace_frame,
                stamp=msg.header.stamp,
            )

        fg_points_c, fg_points_w = self._extract_foreground(cropped_points_c, cropped_points_w)

        if self.publish_foreground_clouds:
            self._publish_cloud(
                pub=self.foreground_cloud_pub,
                points=fg_points_c,
                frame_id=msg.header.frame_id,
                stamp=msg.header.stamp,
            )
            self._publish_cloud(
                pub=self.foreground_cloud_workspace_pub,
                points=fg_points_w,
                frame_id=self.workspace_frame,
                stamp=msg.header.stamp,
            )

        cluster_candidates = self._extract_cluster_candidates(fg_points_w)
        tracked_objects = self._update_tracks(cluster_candidates)

        if not tracked_objects:
            self._last_published_center_w = None
            self._last_published_center_c = None
            self._last_published_extent_w = None
            self._last_primary_track_id = None
            self._object_persistence_score *= self.persistence_decay
            self._publish_object_delete(msg.header.frame_id, msg.header.stamp)
            self._publish_workspace_state(
                mode=mode,
                level=(
                    DiagnosticStatus.OK
                    if mode == "plane_plus_tags"
                    else DiagnosticStatus.WARN
                ),
                message="workspace updated; no object cluster selected",
                extra={
                    "inliers": inlier_count,
                    "inlier_ratio": round(inlier_ratio, 3),
                    "cropped_points": int(cropped_points_c.shape[0]),
                    "foreground_points": int(fg_points_c.shape[0]),
                    "cluster_count": int(len(cluster_candidates)),
                    "roi_source": roi_source,
                    "roi_x_min": round(self._active_roi_x_min, 3),
                    "roi_x_max": round(self._active_roi_x_max, 3),
                    "roi_y_min": round(self._active_roi_y_min, 3),
                    "roi_y_max": round(self._active_roi_y_max, 3),
                    "culled_below_plane": culled_below_plane,
                    "tags_used": int(len(tag_points_c)),
                },
            )
            self._publish_object_metrics(
                foreground_points=int(fg_points_c.shape[0]),
                cluster_count=int(len(cluster_candidates)),
                selected_points=0,
                center_w=None,
                persistence=self._object_persistence_score,
            )
            return

        primary = tracked_objects[0]
        primary_track_id = int(primary["track_id"])
        center_w = np.asarray(primary["center_w"], dtype=np.float32)
        extent_w = np.asarray(primary["extent_w"], dtype=np.float32)
        selected_points = int(primary["points"])
        cluster_count = int(len(cluster_candidates))
        center_c = origin_c + basis_c @ center_w
        center_w, center_c, extent_w, centroid_updated, closer_delta = self._stabilize_published_object(
            center_w=center_w,
            center_c=center_c,
            extent_w=extent_w,
        )

        if self._last_primary_track_id is None:
            self._object_persistence_score = min(1.0, self.persistence_gain)
        elif self._last_primary_track_id == primary_track_id:
            self._object_persistence_score = min(
                1.0, self._object_persistence_score + self.persistence_gain
            )
        else:
            self._object_persistence_score *= self.persistence_decay
        self._last_primary_track_id = primary_track_id

        object_pose_c = PoseStamped()
        object_pose_c.header.frame_id = msg.header.frame_id
        object_pose_c.header.stamp = msg.header.stamp
        object_pose_c.pose.position.x = float(center_c[0])
        object_pose_c.pose.position.y = float(center_c[1])
        object_pose_c.pose.position.z = float(center_c[2])
        object_pose_c.pose.orientation.w = 1.0
        self.object_pose_pub.publish(object_pose_c)

        object_pose_w = PoseStamped()
        object_pose_w.header.frame_id = self.workspace_frame
        object_pose_w.header.stamp = msg.header.stamp
        object_pose_w.pose.position.x = float(center_w[0])
        object_pose_w.pose.position.y = float(center_w[1])
        object_pose_w.pose.position.z = float(center_w[2])
        object_pose_w.pose.orientation.w = 1.0
        self.object_pose_workspace_pub.publish(object_pose_w)

        self._publish_object_markers(
            frame_id=msg.header.frame_id,
            stamp=msg.header.stamp,
            origin_c=origin_c,
            basis_c=basis_c,
            tracked_objects=tracked_objects,
            primary_track_id=primary_track_id,
            primary_center_w=center_w,
            primary_extent_w=extent_w,
        )

        self._publish_workspace_state(
            mode=mode,
            level=(
                DiagnosticStatus.OK
                if mode == "plane_plus_tags"
                else DiagnosticStatus.WARN
            ),
            message="workspace and object updated",
            extra={
                "inliers": inlier_count,
                "inlier_ratio": round(inlier_ratio, 3),
                "cropped_points": int(cropped_points_c.shape[0]),
                "foreground_points": int(fg_points_c.shape[0]),
                "cluster_count": int(cluster_count),
                "selected_points": int(selected_points),
                "culled_below_plane": culled_below_plane,
                "roi_source": roi_source,
                "roi_x_min": round(self._active_roi_x_min, 3),
                "roi_x_max": round(self._active_roi_x_max, 3),
                "roi_y_min": round(self._active_roi_y_min, 3),
                "roi_y_max": round(self._active_roi_y_max, 3),
                "tracked_objects": int(len(tracked_objects)),
                "primary_track_id": int(primary_track_id),
                "centroid_updated": centroid_updated,
                "centroid_closer_delta_m": round(closer_delta, 4),
                "tags_used": int(len(tag_points_c)),
            },
        )
        self._publish_object_metrics(
            foreground_points=int(fg_points_c.shape[0]),
            cluster_count=int(cluster_count),
            selected_points=int(selected_points),
            center_w=center_w,
            persistence=self._object_persistence_score,
        )

    def _extract_xyz(self, msg: PointCloud2) -> Optional[np.ndarray]:
        if msg.point_step <= 0:
            return None

        field_by_name = {f.name: f for f in msg.fields}
        if "x" not in field_by_name or "y" not in field_by_name or "z" not in field_by_name:
            self.get_logger().warn("PointCloud2 is missing x/y/z fields.")
            return None

        fx = field_by_name["x"]
        fy = field_by_name["y"]
        fz = field_by_name["z"]
        if (
            fx.datatype != PointField.FLOAT32
            or fy.datatype != PointField.FLOAT32
            or fz.datatype != PointField.FLOAT32
        ):
            self.get_logger().warn("PointCloud2 x/y/z must be FLOAT32.")
            return None

        n_points = int(msg.width * msg.height)
        if n_points <= 0:
            return np.empty((0, 3), dtype=np.float32)

        endian = ">" if msg.is_bigendian else "<"
        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [f"{endian}f4", f"{endian}f4", f"{endian}f4"],
                "offsets": [int(fx.offset), int(fy.offset), int(fz.offset)],
                "itemsize": int(msg.point_step),
            }
        )

        cloud = np.frombuffer(msg.data, dtype=dtype, count=n_points)
        xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(
            np.float32, copy=False
        )
        finite = np.isfinite(xyz).all(axis=1)
        if not np.any(finite):
            return np.empty((0, 3), dtype=np.float32)
        return xyz[finite]

    def _fit_bench_plane(
        self, points: np.ndarray
    ) -> Optional[Tuple[np.ndarray, float, np.ndarray, float, int, np.ndarray]]:
        n_points = points.shape[0]
        if n_points < 3:
            return None

        fit_points = points
        if n_points > self.plane_fit_max_points:
            idx = self._rng.choice(n_points, self.plane_fit_max_points, replace=False)
            fit_points = points[idx]

        best_count = 0
        best_mask = None
        best_normal = None
        best_d = None

        for _ in range(self.plane_ransac_iterations):
            sample_idx = self._rng.choice(fit_points.shape[0], 3, replace=False)
            p0 = fit_points[sample_idx[0]]
            p1 = fit_points[sample_idx[1]]
            p2 = fit_points[sample_idx[2]]

            normal = np.cross(p1 - p0, p2 - p0)
            normal = _normalize(normal)
            if normal is None:
                continue

            d = -float(np.dot(normal, p0))
            dist = np.abs(fit_points @ normal + d)
            mask = dist <= self.plane_inlier_threshold_m
            count = int(np.count_nonzero(mask))
            if count > best_count:
                best_count = count
                best_mask = mask
                best_normal = normal
                best_d = d

        if best_mask is None or best_normal is None or best_d is None:
            return None

        inlier_ratio = best_count / float(fit_points.shape[0])
        if best_count < self.plane_min_inliers or inlier_ratio < self.plane_min_inlier_ratio:
            return None

        inlier_points = fit_points[best_mask]
        centroid = np.mean(inlier_points, axis=0)
        demeaned = inlier_points - centroid
        _, _, vh = np.linalg.svd(demeaned, full_matrices=False)
        normal = vh[-1]
        normal = _normalize(normal)
        if normal is None:
            return None

        # Orientation convention: +Z of workspace points from bench toward camera origin.
        sensor_vec = -centroid
        if float(np.dot(normal, sensor_vec)) < 0.0:
            normal = -normal

        d = -float(np.dot(normal, centroid))

        full_dist = np.abs(points @ normal + d)
        full_inlier_mask = full_dist <= self.plane_inlier_threshold_m
        full_inliers = int(np.count_nonzero(full_inlier_mask))
        full_ratio = full_inliers / max(1, points.shape[0])
        if full_inliers < self.plane_min_inliers or full_ratio < self.plane_min_inlier_ratio:
            return None

        full_inlier_points = points[full_inlier_mask]
        plane_origin = np.mean(full_inlier_points, axis=0)
        plane_origin = plane_origin - (np.dot(normal, plane_origin) + d) * normal

        return normal, d, full_inlier_mask, full_ratio, full_inliers, plane_origin

    def _lookup_tag_points(self, cloud_frame: str) -> Dict[str, np.ndarray]:
        tag_points: Dict[str, np.ndarray] = {}
        if not self.use_tag_refinement or len(self.tag_frames) < 2:
            return tag_points

        now = self.get_clock().now()
        for frame in self.tag_frames:
            try:
                tf_msg = self.tf_buffer.lookup_transform(cloud_frame, frame, rclpy.time.Time())
            except TransformException:
                continue

            stamp = rclpy.time.Time.from_msg(tf_msg.header.stamp)
            age_s = (now - stamp).nanoseconds / 1e9
            if age_s < 0.0:
                age_s = abs(age_s)
            if age_s > self.tag_timeout_s:
                continue

            translation = tf_msg.transform.translation
            tag_points[frame] = np.array(
                [translation.x, translation.y, translation.z], dtype=np.float32
            )

        return tag_points

    def _compute_workspace_basis(
        self,
        plane_normal: np.ndarray,
        plane_origin: np.ndarray,
        tag_points_c: Dict[str, np.ndarray],
    ) -> Optional[Tuple[np.ndarray, str]]:
        z_axis = _normalize(plane_normal)
        if z_axis is None:
            return None

        mode = "plane_only"
        x_axis = None

        if len(tag_points_c) >= 2:
            frame0, frame1 = self.tag_frames[0], self.tag_frames[1]
            if frame0 in tag_points_c and frame1 in tag_points_c:
                tag_vec = tag_points_c[frame1] - tag_points_c[frame0]
                if float(np.linalg.norm(tag_vec)) >= self.tag_min_separation_m:
                    tag_vec_proj = tag_vec - np.dot(tag_vec, z_axis) * z_axis
                    x_axis = _normalize(tag_vec_proj)
                    if x_axis is not None:
                        mode = "plane_plus_tags"

        if x_axis is None:
            camera_x = np.array([1.0, 0.0, 0.0], dtype=np.float32)
            x_proj = camera_x - np.dot(camera_x, z_axis) * z_axis
            x_axis = _normalize(x_proj)
            if x_axis is None:
                camera_y = np.array([0.0, 1.0, 0.0], dtype=np.float32)
                x_proj = camera_y - np.dot(camera_y, z_axis) * z_axis
                x_axis = _normalize(x_proj)

        if x_axis is None:
            return None

        y_axis = _normalize(np.cross(z_axis, x_axis))
        if y_axis is None:
            return None

        x_axis = _normalize(np.cross(y_axis, z_axis))
        if x_axis is None:
            return None

        basis_c = np.column_stack((x_axis, y_axis, z_axis))
        return basis_c, mode

    def _broadcast_workspace_tf(
        self,
        parent_frame: str,
        stamp,
        origin_c: np.ndarray,
        basis_c: np.ndarray,
    ) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = parent_frame
        tf_msg.header.stamp = stamp
        tf_msg.child_frame_id = self.workspace_frame
        tf_msg.transform.translation.x = float(origin_c[0])
        tf_msg.transform.translation.y = float(origin_c[1])
        tf_msg.transform.translation.z = float(origin_c[2])
        qx, qy, qz, qw = _quat_from_rotation_matrix(basis_c)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_plane_coefficients(self, normal: np.ndarray, d: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(normal[0]), float(normal[1]), float(normal[2]), float(d)]
        self.plane_coeff_pub.publish(msg)

    def _to_workspace(
        self, points_c: np.ndarray, origin_c: np.ndarray, basis_c: np.ndarray
    ) -> np.ndarray:
        return (points_c - origin_c) @ basis_c

    def _workspace_to_camera(
        self, points_w: np.ndarray, origin_c: np.ndarray, basis_c: np.ndarray
    ) -> np.ndarray:
        if points_w.ndim == 1:
            return origin_c + basis_c @ points_w
        return origin_c + points_w @ basis_c.T

    def _above_plane_mask(
        self, points_c: np.ndarray, normal: np.ndarray, d: float
    ) -> np.ndarray:
        if not self.enforce_above_plane_cull:
            return np.ones(points_c.shape[0], dtype=bool)
        signed_dist = points_c @ normal + float(d)
        return signed_dist >= -self.below_plane_tolerance_m

    def _compute_active_roi_bounds(
        self,
        tag_points_c: Dict[str, np.ndarray],
        origin_c: np.ndarray,
        basis_c: np.ndarray,
    ) -> str:
        target_x_min = self.roi_x_min
        target_x_max = self.roi_x_max
        target_y_min = self.roi_y_min
        target_y_max = self.roi_y_max
        roi_source = "configured"

        if self.roi_from_tags_enabled and len(tag_points_c) >= 2:
            tag_points_w: List[np.ndarray] = []
            for frame in self.tag_frames:
                if frame not in tag_points_c:
                    continue
                point_w = self._to_workspace(
                    tag_points_c[frame].reshape(1, 3), origin_c, basis_c
                )[0]
                tag_points_w.append(point_w)

            if len(tag_points_w) >= 2:
                tag_arr = np.asarray(tag_points_w, dtype=np.float32)
                tx_min = float(np.min(tag_arr[:, 0])) - self.roi_from_tags_margin_m
                tx_max = float(np.max(tag_arr[:, 0])) + self.roi_from_tags_margin_m
                ty_min = float(np.min(tag_arr[:, 1])) - self.roi_from_tags_margin_m
                ty_max = float(np.max(tag_arr[:, 1])) + self.roi_from_tags_margin_m

                cfg_span_x = max(0.02, self.roi_x_max - self.roi_x_min)
                cfg_span_y = max(0.02, self.roi_y_max - self.roi_y_min)
                span_x = tx_max - tx_min
                span_y = ty_max - ty_min

                cx = 0.5 * (tx_min + tx_max)
                cy = 0.5 * (ty_min + ty_max)
                if span_x < self.roi_from_tags_min_span_m:
                    half_x = 0.5 * cfg_span_x
                    tx_min = cx - half_x
                    tx_max = cx + half_x
                if span_y < self.roi_from_tags_min_span_m:
                    half_y = 0.5 * cfg_span_y
                    ty_min = cy - half_y
                    ty_max = cy + half_y

                target_x_min = tx_min
                target_x_max = tx_max
                target_y_min = ty_min
                target_y_max = ty_max
                roi_source = "tag_corners"

        alpha = self.roi_from_tags_smoothing_alpha if roi_source == "tag_corners" else 1.0
        if alpha > 0.0:
            self._active_roi_x_min = (1.0 - alpha) * self._active_roi_x_min + alpha * target_x_min
            self._active_roi_x_max = (1.0 - alpha) * self._active_roi_x_max + alpha * target_x_max
            self._active_roi_y_min = (1.0 - alpha) * self._active_roi_y_min + alpha * target_y_min
            self._active_roi_y_max = (1.0 - alpha) * self._active_roi_y_max + alpha * target_y_max

        if self._active_roi_x_max <= self._active_roi_x_min:
            self._active_roi_x_min = self.roi_x_min
            self._active_roi_x_max = self.roi_x_max
        if self._active_roi_y_max <= self._active_roi_y_min:
            self._active_roi_y_min = self.roi_y_min
            self._active_roi_y_max = self.roi_y_max

        return roi_source

    def _workspace_roi_mask(self, points_w: np.ndarray) -> np.ndarray:
        mask = (
            (points_w[:, 0] >= self._active_roi_x_min)
            & (points_w[:, 0] <= self._active_roi_x_max)
            & (points_w[:, 1] >= self._active_roi_y_min)
            & (points_w[:, 1] <= self._active_roi_y_max)
            & (points_w[:, 2] >= self.roi_z_min)
            & (points_w[:, 2] <= self.roi_z_max)
        )

        if self.pedestal_exclusion_enabled:
            dx = points_w[:, 0] - self.pedestal_center_x
            dy = points_w[:, 1] - self.pedestal_center_y
            radial = np.sqrt(dx * dx + dy * dy)
            pedestal_mask = (
                (radial <= self.pedestal_radius)
                & (points_w[:, 2] >= self.pedestal_z_min)
                & (points_w[:, 2] <= self.pedestal_z_max)
            )
            mask &= ~pedestal_mask

        return mask

    def _extract_foreground(
        self, cropped_points_c: np.ndarray, cropped_points_w: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        if cropped_points_w.shape[0] == 0:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
            )

        points_w = cropped_points_w
        points_c = cropped_points_c

        if points_w.shape[0] > self.background_max_points:
            idx = self._rng.choice(points_w.shape[0], self.background_max_points, replace=False)
            points_w_for_bg = points_w[idx]
        else:
            points_w_for_bg = points_w

        voxel_idx_bg = np.floor(points_w_for_bg / self.background_voxel_size).astype(np.int32)
        current_keys = set(tuple(row) for row in voxel_idx_bg.tolist())

        for key in current_keys:
            prev = self._background_scores.get(key, 0.0)
            self._background_scores[key] = min(1.0, prev + self.background_increment)

        if self.background_decay > 0.0 and self._background_scores:
            for key in list(self._background_scores.keys()):
                if key in current_keys:
                    continue
                decayed = self._background_scores[key] - self.background_decay
                if decayed <= 0.0:
                    del self._background_scores[key]
                else:
                    self._background_scores[key] = decayed

        if self._frame_count <= self.background_warmup_frames:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
            )

        voxel_idx_all = np.floor(points_w / self.background_voxel_size).astype(np.int32)
        point_keys = [tuple(row) for row in voxel_idx_all.tolist()]
        scores = np.array(
            [self._background_scores.get(key, 0.0) for key in point_keys],
            dtype=np.float32,
        )
        fg_mask = scores < self.background_threshold

        if int(np.count_nonzero(fg_mask)) < self.min_foreground_points:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
            )

        return points_c[fg_mask], points_w[fg_mask]

    def _extract_cluster_candidates(
        self, foreground_points_w: np.ndarray
    ) -> List[Dict[str, object]]:
        if foreground_points_w.shape[0] < self.min_foreground_points:
            return []

        voxel_idx = np.floor(foreground_points_w / self.cluster_voxel_size).astype(np.int32)
        voxel_to_indices: Dict[Tuple[int, int, int], List[int]] = defaultdict(list)
        for i, row in enumerate(voxel_idx.tolist()):
            voxel_to_indices[(row[0], row[1], row[2])].append(i)

        visited = set()
        clusters: List[List[int]] = []

        for voxel in voxel_to_indices.keys():
            if voxel in visited:
                continue
            stack = [voxel]
            visited.add(voxel)
            cluster_indices: List[int] = []

            while stack:
                current = stack.pop()
                cluster_indices.extend(voxel_to_indices[current])
                cx, cy, cz = current
                for dx, dy, dz in self._neighbor_offsets:
                    nxt = (cx + dx, cy + dy, cz + dz)
                    if nxt in visited:
                        continue
                    if nxt in voxel_to_indices:
                        visited.add(nxt)
                        stack.append(nxt)

            if len(cluster_indices) >= self.min_cluster_points:
                clusters.append(cluster_indices)

        if not clusters:
            return []

        candidates: List[Dict[str, object]] = []
        for cluster_indices in clusters:
            n_points = len(cluster_indices)
            if n_points > self.max_cluster_points:
                continue

            cluster_pts = foreground_points_w[cluster_indices]
            mins = np.min(cluster_pts, axis=0)
            maxs = np.max(cluster_pts, axis=0)
            center = 0.5 * (mins + maxs)
            extent = np.maximum(maxs - mins, self.object_marker_min_extent_m)
            volume = float(extent[0] * extent[1] * extent[2])

            # Reject human/background-scale clusters so we keep gripper-sized objects.
            if (
                extent[0] > self.object_candidate_max_extent_x_m
                or extent[1] > self.object_candidate_max_extent_y_m
                or extent[2] > self.object_candidate_max_extent_z_m
                or volume > self.object_candidate_max_volume_m3
            ):
                continue

            size_score = min(1.0, n_points / float(self.min_cluster_points * 4))
            height_score = np.clip(
                (center[2] - self.roi_z_min) / max(1e-6, (self.roi_z_max - self.roi_z_min)),
                0.0,
                1.0,
            )
            border_margin = min(
                center[0] - self._active_roi_x_min,
                self._active_roi_x_max - center[0],
                center[1] - self._active_roi_y_min,
                self._active_roi_y_max - center[1],
            )
            border_score = np.clip(border_margin / 0.12, 0.0, 1.0)
            score = 0.45 * size_score + 0.30 * height_score + 0.25 * border_score

            candidates.append(
                {
                    "center_w": center.astype(np.float32, copy=False),
                    "extent_w": extent.astype(np.float32, copy=False),
                    "points": int(n_points),
                    "score": float(score),
                }
            )

        candidates.sort(
            key=lambda c: (float(c["score"]), float(c["points"])),
            reverse=True,
        )
        max_candidates = max(self.max_tracked_objects, self.max_tracked_objects * 4)
        return candidates[:max_candidates]

    def _update_tracks(
        self, candidates: List[Dict[str, object]]
    ) -> List[Dict[str, object]]:
        matched_track_ids: set[int] = set()
        matched_candidate_ids: set[int] = set()

        distances: List[Tuple[float, int, int]] = []
        for track_id, track in self._tracks.items():
            track_center_w = np.asarray(track["center_w"], dtype=np.float32)
            for cand_idx, candidate in enumerate(candidates):
                cand_center_w = np.asarray(candidate["center_w"], dtype=np.float32)
                dist = float(np.linalg.norm(cand_center_w - track_center_w))
                if dist <= self.track_match_distance_m:
                    distances.append((dist, track_id, cand_idx))

        distances.sort(key=lambda item: item[0])
        for dist, track_id, cand_idx in distances:
            if track_id in matched_track_ids or cand_idx in matched_candidate_ids:
                continue
            matched_track_ids.add(track_id)
            matched_candidate_ids.add(cand_idx)
            track = self._tracks[track_id]
            candidate = candidates[cand_idx]

            prev_center_w = np.asarray(track["center_w"], dtype=np.float32)
            prev_extent_w = np.asarray(track["extent_w"], dtype=np.float32)
            cand_center_w = np.asarray(candidate["center_w"], dtype=np.float32)
            cand_extent_w = np.asarray(candidate["extent_w"], dtype=np.float32)

            alpha = self.track_smoothing_alpha
            if alpha > 0.0:
                new_center_w = (1.0 - alpha) * prev_center_w + alpha * cand_center_w
                new_extent_w = (1.0 - alpha) * prev_extent_w + alpha * cand_extent_w
            else:
                new_center_w = cand_center_w
                new_extent_w = cand_extent_w

            track["center_w"] = new_center_w.astype(np.float32, copy=False)
            track["extent_w"] = new_extent_w.astype(np.float32, copy=False)
            track["points"] = int(candidate["points"])
            track["score"] = float(candidate["score"])
            track["age"] = int(track.get("age", 0)) + 1
            track["missed"] = 0
            track["match_dist"] = float(dist)

        for track_id in list(self._tracks.keys()):
            if track_id in matched_track_ids:
                continue
            track = self._tracks[track_id]
            track["missed"] = int(track.get("missed", 0)) + 1
            track["score"] = float(track.get("score", 0.0)) * self.persistence_decay
            if int(track["missed"]) > self.track_max_missed_frames:
                del self._tracks[track_id]

        unmatched_candidates = [
            idx for idx in range(len(candidates)) if idx not in matched_candidate_ids
        ]
        unmatched_candidates.sort(
            key=lambda idx: float(candidates[idx]["score"]), reverse=True
        )
        for cand_idx in unmatched_candidates:
            if len(self._tracks) >= self.max_tracked_objects:
                break
            candidate = candidates[cand_idx]
            cand_center_w = np.asarray(candidate["center_w"], dtype=np.float32)

            # Spawn deadzone: do not create a new track if it would overlap an
            # existing one. This avoids duplicate markers on the same object.
            if self.track_spawn_deadzone_m > 0.0:
                suppress_spawn = False
                for existing_track in self._tracks.values():
                    existing_center_w = np.asarray(
                        existing_track["center_w"], dtype=np.float32
                    )
                    if (
                        float(np.linalg.norm(cand_center_w - existing_center_w))
                        < self.track_spawn_deadzone_m
                    ):
                        suppress_spawn = True
                        break
                if suppress_spawn:
                    continue

            track_id = self._next_track_id
            self._next_track_id += 1
            self._tracks[track_id] = {
                "track_id": int(track_id),
                "center_w": cand_center_w.copy(),
                "extent_w": np.asarray(candidate["extent_w"], dtype=np.float32).copy(),
                "points": int(candidate["points"]),
                "score": float(candidate["score"]),
                "age": 1,
                "missed": 0,
                "match_dist": 0.0,
            }

        active_tracks: List[Dict[str, object]] = []
        for track_id, track in self._tracks.items():
            if int(track.get("missed", 0)) > 0:
                continue
            active_tracks.append(
                {
                    "track_id": int(track_id),
                    "center_w": np.asarray(track["center_w"], dtype=np.float32),
                    "extent_w": np.asarray(track["extent_w"], dtype=np.float32),
                    "points": int(track.get("points", 0)),
                    "score": float(track.get("score", 0.0)),
                    "age": int(track.get("age", 0)),
                }
            )

        active_tracks.sort(
            key=lambda t: (float(t["score"]), float(t["points"])),
            reverse=True,
        )
        return active_tracks[: self.max_tracked_objects]

    def _stabilize_published_object(
        self,
        center_w: np.ndarray,
        center_c: np.ndarray,
        extent_w: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, bool, float]:
        if self._last_published_center_w is None or self._last_published_center_c is None:
            self._last_published_center_w = center_w.copy()
            self._last_published_center_c = center_c.copy()
            self._last_published_extent_w = extent_w.copy()
            return center_w, center_c, extent_w, True, 0.0

        last_center_c = self._last_published_center_c
        last_center_w = self._last_published_center_w
        last_extent_w = self._last_published_extent_w

        if last_extent_w is None:
            last_extent_w = extent_w.copy()

        prev_range_m = float(np.linalg.norm(last_center_c))
        new_range_m = float(np.linalg.norm(center_c))
        closer_delta = prev_range_m - new_range_m

        should_update = True
        if self.centroid_update_closer_only:
            should_update = closer_delta >= self.centroid_min_closer_m

        if should_update:
            alpha = self.centroid_smoothing_alpha
            if alpha > 0.0:
                center_w = (1.0 - alpha) * last_center_w + alpha * center_w
                center_c = (1.0 - alpha) * last_center_c + alpha * center_c
                extent_w = (1.0 - alpha) * last_extent_w + alpha * extent_w

            self._last_published_center_w = center_w.copy()
            self._last_published_center_c = center_c.copy()
            self._last_published_extent_w = extent_w.copy()
            return center_w, center_c, extent_w, True, closer_delta

        return (
            last_center_w.copy(),
            last_center_c.copy(),
            last_extent_w.copy(),
            False,
            closer_delta,
        )

    def _publish_cloud(self, pub, points: np.ndarray, frame_id: str, stamp) -> None:
        cloud = PointCloud2()
        cloud.header.frame_id = frame_id
        cloud.header.stamp = stamp
        cloud.height = 1
        cloud.width = int(points.shape[0])
        cloud.fields = self._pc_fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.width * cloud.point_step
        cloud.is_dense = False
        if points.shape[0] == 0:
            cloud.data = b""
        else:
            cloud.data = points.astype(np.float32, copy=False).tobytes()
        pub.publish(cloud)

    def _publish_plane_marker(self, stamp) -> None:
        marker = Marker()
        marker.header.frame_id = self.workspace_frame
        marker.header.stamp = stamp
        marker.ns = "holoassist_bench_plane"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float((self._active_roi_x_min + self._active_roi_x_max) * 0.5)
        marker.pose.position.y = float((self._active_roi_y_min + self._active_roi_y_max) * 0.5)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(max(0.02, self._active_roi_x_max - self._active_roi_x_min))
        marker.scale.y = float(max(0.02, self._active_roi_y_max - self._active_roi_y_min))
        marker.scale.z = float(max(0.001, self.plane_marker_thickness_m))
        marker.color.r = 0.2
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = float(np.clip(self.marker_alpha, 0.05, 1.0))
        self.plane_marker_pub.publish(marker)

    def _publish_tag_markers(
        self,
        stamp,
        tag_points_c: Dict[str, np.ndarray],
        origin_c: np.ndarray,
        basis_c: np.ndarray,
    ) -> None:
        marker = Marker()
        marker.header.frame_id = self.workspace_frame
        marker.header.stamp = stamp
        marker.ns = "holoassist_workspace_tags"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        nominal_tag_size = max(0.01, self.tag_size_m)
        marker.scale.x = nominal_tag_size
        marker.scale.y = nominal_tag_size
        marker.scale.z = nominal_tag_size
        marker.color.r = 1.0
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.9

        if tag_points_c:
            for frame in self.tag_frames:
                if frame not in tag_points_c:
                    continue
                point_w = self._to_workspace(tag_points_c[frame].reshape(1, 3), origin_c, basis_c)[0]
                pt = Point()
                pt.x = float(point_w[0])
                pt.y = float(point_w[1])
                pt.z = float(point_w[2])
                marker.points.append(pt)
        else:
            marker.action = Marker.DELETE

        self.tag_marker_pub.publish(marker)

    def _publish_axes_markers(self, stamp) -> None:
        colors = [
            (1.0, 0.1, 0.1),
            (0.1, 1.0, 0.1),
            (0.1, 0.4, 1.0),
        ]
        ends = [
            (self.axes_marker_length_m, 0.0, 0.0),
            (0.0, self.axes_marker_length_m, 0.0),
            (0.0, 0.0, self.axes_marker_length_m),
        ]

        for idx, (end, color) in enumerate(zip(ends, colors)):
            marker = Marker()
            marker.header.frame_id = self.workspace_frame
            marker.header.stamp = stamp
            marker.ns = "holoassist_workspace_axes"
            marker.id = idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.axes_marker_radius_m * 0.9
            marker.scale.y = self.axes_marker_radius_m * 1.4
            marker.scale.z = self.axes_marker_radius_m * 1.8
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.95
            p0 = Point()
            p0.x = 0.0
            p0.y = 0.0
            p0.z = 0.0
            p1 = Point()
            p1.x = float(end[0])
            p1.y = float(end[1])
            p1.z = float(end[2])
            marker.points = [p0, p1]
            self.axes_marker_pub.publish(marker)

    def _track_color(self, track_id: int) -> Tuple[float, float, float]:
        palette = [
            (1.0, 0.35, 0.2),
            (0.20, 0.75, 1.0),
            (0.25, 0.95, 0.35),
            (1.0, 0.90, 0.20),
            (0.90, 0.45, 1.0),
            (0.20, 1.0, 0.90),
        ]
        return palette[(max(1, int(track_id)) - 1) % len(palette)]

    def _publish_object_markers(
        self,
        frame_id: str,
        stamp,
        origin_c: np.ndarray,
        basis_c: np.ndarray,
        tracked_objects: List[Dict[str, object]],
        primary_track_id: int,
        primary_center_w: np.ndarray,
        primary_extent_w: np.ndarray,
    ) -> None:
        qx, qy, qz, qw = _quat_from_rotation_matrix(basis_c)
        current_marker_ids: set[int] = set()

        for tracked in tracked_objects:
            track_id = int(tracked["track_id"])
            if track_id == primary_track_id:
                center_w = primary_center_w
                extent_w = primary_extent_w
            else:
                center_w = np.asarray(tracked["center_w"], dtype=np.float32)
                extent_w = np.asarray(tracked["extent_w"], dtype=np.float32)

            center_c = origin_c + basis_c @ center_w
            color = self._track_color(track_id)

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = "holoassist_object"
            marker.id = int(track_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(center_c[0])
            marker.pose.position.y = float(center_c[1])
            marker.pose.position.z = float(center_c[2])
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = float(
                min(
                    self.object_marker_max_extent_m,
                    max(self.object_marker_min_extent_m, extent_w[0]),
                )
            )
            marker.scale.y = float(
                min(
                    self.object_marker_max_extent_m,
                    max(self.object_marker_min_extent_m, extent_w[1]),
                )
            )
            marker.scale.z = float(
                min(
                    self.object_marker_max_extent_m,
                    max(self.object_marker_min_extent_m, extent_w[2]),
                )
            )
            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])
            marker.color.a = 0.72 if track_id == primary_track_id else 0.50
            self.object_marker_pub.publish(marker)
            current_marker_ids.add(marker.id)

        stale_ids = self._last_marker_ids - current_marker_ids
        for marker_id in stale_ids:
            delete_marker = Marker()
            delete_marker.header.frame_id = frame_id
            delete_marker.header.stamp = stamp
            delete_marker.ns = "holoassist_object"
            delete_marker.id = int(marker_id)
            delete_marker.action = Marker.DELETE
            self.object_marker_pub.publish(delete_marker)

        self._last_marker_ids = current_marker_ids

    def _publish_object_delete(self, frame_id: str, stamp) -> None:
        marker_ids = set(self._last_marker_ids)
        marker_ids.add(0)
        for marker_id in marker_ids:
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = "holoassist_object"
            marker.id = int(marker_id)
            marker.action = Marker.DELETE
            self.object_marker_pub.publish(marker)
        self._last_marker_ids.clear()

    def _publish_workspace_state(
        self,
        mode: str,
        level: int,
        message: str,
        extra: Optional[Dict[str, object]] = None,
    ) -> None:
        mode_msg = String()
        mode_msg.data = mode
        self.workspace_mode_pub.publish(mode_msg)

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "holoassist/workspace_frame"
        if isinstance(level, (bytes, bytearray)):
            status.level = bytes(level[:1])
        else:
            level_int = max(0, min(255, int(level)))
            status.level = bytes([level_int])
        status.hardware_id = "holoassist_depth_workspace"
        status.message = message

        values = [
            KeyValue(key="mode", value=mode),
            KeyValue(key="workspace_frame", value=self.workspace_frame),
            KeyValue(key="tag_frames", value=",".join(self.tag_frames)),
        ]
        if extra:
            for k, v in extra.items():
                values.append(KeyValue(key=str(k), value=str(v)))
        status.values = values

        diag.status = [status]
        self.workspace_diag_pub.publish(diag)
        self._last_mode = mode

    def _publish_object_metrics(
        self,
        foreground_points: int,
        cluster_count: int,
        selected_points: int,
        center_w: Optional[np.ndarray],
        persistence: float,
    ) -> None:
        msg = Float32MultiArray()
        if center_w is None:
            cx = cy = cz = float("nan")
        else:
            cx, cy, cz = float(center_w[0]), float(center_w[1]), float(center_w[2])

        msg.data = [
            float(foreground_points),
            float(cluster_count),
            float(selected_points),
            cx,
            cy,
            cz,
            float(persistence),
            float(len(self._background_scores)),
        ]
        self.object_metrics_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorkspacePerceptionNode()
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
