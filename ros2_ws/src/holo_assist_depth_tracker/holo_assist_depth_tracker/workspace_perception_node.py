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
from std_srvs.srv import Trigger
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


def _rotation_matrix_from_quaternion(
    x: float, y: float, z: float, w: float
) -> np.ndarray:
    """Return 3x3 rotation matrix from quaternion (x,y,z,w)."""
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
        dtype=np.float32,
    )


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
        self.robot_pose_pub = self.create_publisher(
            PoseStamped, self.robot_pose_topic, reliable_qos
        )
        self.robot_marker_pub = self.create_publisher(
            Marker, self.robot_marker_topic, reliable_qos
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.input_pointcloud_topic,
            self._on_pointcloud,
            qos_profile_sensor_data,
        )
        self.apriltag_object_pose_sub = self.create_subscription(
            PoseStamped,
            self.apriltag_object_pose_topic,
            self._on_apriltag_object_pose,
            qos_profile_sensor_data,
        )
        self.robot_pose_timer = self.create_timer(
            1.0 / self.robot_pose_publish_hz, self._on_robot_pose_timer
        )
        self.realign_service = self.create_service(
            Trigger,
            self.workspace_realign_service_name,
            self._on_realign_workspace_service,
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
        self._last_pair_x_axis_c: Optional[np.ndarray] = None
        self._last_pair_front_center_x: Optional[float] = None
        self._last_pair_front_edge_y: Optional[float] = None
        self._last_pair_interior_sign: Optional[float] = None
        self._last_pair_tag_points_w: Dict[str, np.ndarray] = {}
        self._latest_apriltag_object_pose: Optional[PoseStamped] = None
        self._latest_apriltag_object_pose_stamp: Optional[rclpy.time.Time] = None

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
        self._background_warmup_until_frame = self.background_warmup_frames
        self._last_resnap_time = self.get_clock().now()
        self._last_resnap_frame = 0
        self._resnap_count = 0
        self._last_resnap_reason = "startup"
        self._workspace_locked = False
        self._workspace_initial_snap_done = False
        self._manual_realign_requested = False

        self.get_logger().info(
            "Workspace perception started. input=%s workspace_frame=%s tags=%s"
            % (
                self.input_pointcloud_topic,
                self.workspace_frame,
                self.tag_frames,
            )
        )
        self.get_logger().info(
            "workspace roi dynamic=%s front_edge_anchor=%s resnap_interval_s=%.1f clear_background=%s clear_tracks=%s"
            % (
                self.roi_from_tags_enabled,
                self.roi_from_tags_use_front_edge,
                self.resnap_interval_s,
                self.resnap_clear_background,
                self.resnap_clear_tracks,
            )
        )
        self.get_logger().info(
            "workspace lock_after_initial_two_tag_snap=%s realign_service=%s"
            % (
                self.lock_workspace_after_initial_two_tag_snap,
                self.workspace_realign_service_name,
            )
        )
        self.get_logger().info(
            "robot_pose follow_workspace=%s from_tag_pair=%s back_edge_offset=%.3fm edge_clearance=%.3fm x_offset=%.3fm manual=(x=%.3f,y=%.3f,z=%.3f,yaw=%.1f) base_dia=%.3fm marker_topic=%s tf=%s child=%s hz=%.1f snap_origin_to_bench_center=%s"
            % (
                self.robot_pose_follow_workspace,
                self.robot_pose_from_tag_pair,
                self.robot_tag_pair_edge_offset_m,
                self.robot_edge_clearance_m,
                self.robot_pose_x_offset_m,
                self.robot_pose_x_m,
                self.robot_pose_y_m,
                self.robot_pose_z_m,
                self.robot_pose_yaw_deg,
                self.robot_base_diameter_m,
                self.robot_marker_topic,
                self.publish_robot_tf,
                self.robot_tf_child_frame,
                self.robot_pose_publish_hz,
                self.workspace_origin_snap_to_bench_center,
            )
        )
        self.get_logger().info(
            "object source mode=%s apriltag_topic=%s timeout=%.2fs size=%.3fm"
            % (
                self.object_source_mode,
                self.apriltag_object_pose_topic,
                self.apriltag_object_timeout_s,
                self.apriltag_object_size_m,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "input_pointcloud_topic", "/holo_assist_depth_tracker/pointcloud"
        )
        self.declare_parameter("workspace_frame", "workspace_frame")

        self.declare_parameter("use_tag_refinement", True)
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("tag_ids", [1, 0])
        self.declare_parameter("tag_frame_names", ["tag36h11:1", "tag36h11:0"])
        self.declare_parameter("tag_size_m", 0.032)
        self.declare_parameter("tag_timeout_s", 0.5)
        self.declare_parameter("tag_min_separation_m", 0.08)
        self.declare_parameter("project_tags_to_plane", True)
        self.declare_parameter("tag_plane_projection_max_distance_m", 0.30)
        self.declare_parameter("bench_plane_from_tags_only", True)
        self.declare_parameter("tag_markers_use_projected_points", False)
        self.declare_parameter("tag_markers_flatten_z_to_plane", True)
        self.declare_parameter("tag_markers_raw_z_offset_m", 0.0)
        self.declare_parameter("single_tag_axis_preference", "x")
        self.declare_parameter("single_tag_left_axis_sign", 1.0)
        self.declare_parameter("single_tag_right_axis_sign", 1.0)

        self.declare_parameter("plane_fit_max_points", 18000)
        self.declare_parameter("plane_fit_near_depth_quantile", 1.0)
        self.declare_parameter("plane_ransac_iterations", 140)
        self.declare_parameter("plane_inlier_threshold_m", 0.015)
        self.declare_parameter("plane_min_inlier_ratio", 0.35)
        self.declare_parameter("plane_min_inliers", 600)
        self.declare_parameter("plane_smoothing_alpha", 0.25)
        self.declare_parameter("resnap_interval_s", 0.0)
        self.declare_parameter("resnap_clear_background", False)
        self.declare_parameter("resnap_clear_tracks", False)
        self.declare_parameter("lock_workspace_after_initial_two_tag_snap", True)
        self.declare_parameter(
            "workspace_realign_service_name",
            "/holoassist/perception/realign_workspace",
        )

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
        self.declare_parameter("roi_from_tags_use_front_edge", False)
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
        self.declare_parameter("object_candidate_min_top_height_m", 0.025)
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
            "robot_pose_topic", "/holoassist/perception/ur3e_base_link0_pose"
        )
        self.declare_parameter(
            "robot_marker_topic", "/holoassist/perception/ur3e_base_link0_marker"
        )
        self.declare_parameter("publish_robot_pose_marker", True)
        self.declare_parameter("publish_robot_tf", True)
        self.declare_parameter("robot_tf_child_frame", "ur3e_base_link0")
        self.declare_parameter("robot_pose_publish_hz", 10.0)
        self.declare_parameter("robot_pose_follow_workspace", True)
        self.declare_parameter("robot_pose_from_tag_pair", True)
        # Robot center offset from board back edge (not from tag edge), in meters.
        # For UR3e base radius 64 mm, use 0.064.
        self.declare_parameter("robot_tag_pair_edge_offset_m", 0.064)
        self.declare_parameter("robot_edge_clearance_m", 0.0)
        # Centered workspace-frame default for a 700x500 board:
        # robot center = 450 mm from left edge -> x = +0.100 m
        self.declare_parameter("robot_pose_x_offset_m", 0.100)
        self.declare_parameter("robot_pose_x_m", 0.100)
        self.declare_parameter("robot_pose_y_m", 0.314)
        # UR3e base joint frame is 15 mm below board/plane surface.
        self.declare_parameter("robot_pose_z_m", -0.015)
        self.declare_parameter("robot_pose_yaw_deg", 0.0)
        self.declare_parameter("robot_base_diameter_m", 0.128)
        self.declare_parameter("robot_marker_height_m", 0.01)
        self.declare_parameter("workspace_origin_snap_to_bench_center", True)

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
        self.declare_parameter("marker_lifetime_s", 0.8)
        self.declare_parameter("plane_marker_thickness_m", 0.006)
        self.declare_parameter("plane_marker_from_four_tags_enabled", True)
        self.declare_parameter("plane_marker_four_tags_edge_offset_m", 0.016)
        self.declare_parameter("axes_marker_length_m", 0.12)
        self.declare_parameter("axes_marker_radius_m", 0.01)
        self.declare_parameter("axes_marker_lift_m", 0.0)
        self.declare_parameter("object_marker_min_extent_m", 0.02)
        self.declare_parameter("object_marker_max_extent_m", 0.18)
        self.declare_parameter("object_source_mode", "apriltag")
        self.declare_parameter(
            "apriltag_object_pose_topic", "/holoassist/perception/april_cube_pose"
        )
        self.declare_parameter("apriltag_object_timeout_s", 0.8)
        self.declare_parameter("apriltag_object_size_m", 0.075)
        self.declare_parameter("publish_object_marker", True)

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
        self.project_tags_to_plane = bool(
            self.get_parameter("project_tags_to_plane").value
        )
        self.tag_plane_projection_max_distance_m = float(
            self.get_parameter("tag_plane_projection_max_distance_m").value
        )
        self.bench_plane_from_tags_only = bool(
            self.get_parameter("bench_plane_from_tags_only").value
        )
        self.tag_markers_use_projected_points = bool(
            self.get_parameter("tag_markers_use_projected_points").value
        )
        self.tag_markers_flatten_z_to_plane = bool(
            self.get_parameter("tag_markers_flatten_z_to_plane").value
        )
        self.tag_markers_raw_z_offset_m = float(
            self.get_parameter("tag_markers_raw_z_offset_m").value
        )
        self.single_tag_axis_preference = str(
            self.get_parameter("single_tag_axis_preference").value
        ).strip().lower()
        self.single_tag_left_axis_sign = float(
            self.get_parameter("single_tag_left_axis_sign").value
        )
        self.single_tag_right_axis_sign = float(
            self.get_parameter("single_tag_right_axis_sign").value
        )

        self.plane_fit_max_points = int(
            self.get_parameter("plane_fit_max_points").value
        )
        self.plane_fit_near_depth_quantile = float(
            self.get_parameter("plane_fit_near_depth_quantile").value
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
        self.resnap_interval_s = float(self.get_parameter("resnap_interval_s").value)
        self.resnap_clear_background = bool(
            self.get_parameter("resnap_clear_background").value
        )
        self.resnap_clear_tracks = bool(self.get_parameter("resnap_clear_tracks").value)
        self.lock_workspace_after_initial_two_tag_snap = bool(
            self.get_parameter("lock_workspace_after_initial_two_tag_snap").value
        )
        self.workspace_realign_service_name = str(
            self.get_parameter("workspace_realign_service_name").value
        ).strip()

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
        self.roi_from_tags_use_front_edge = bool(
            self.get_parameter("roi_from_tags_use_front_edge").value
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
        self.object_candidate_min_top_height_m = float(
            self.get_parameter("object_candidate_min_top_height_m").value
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
        self.robot_pose_topic = str(self.get_parameter("robot_pose_topic").value)
        self.robot_marker_topic = str(self.get_parameter("robot_marker_topic").value)
        self.publish_robot_pose_marker = bool(
            self.get_parameter("publish_robot_pose_marker").value
        )
        self.publish_robot_tf = bool(self.get_parameter("publish_robot_tf").value)
        self.robot_tf_child_frame = str(
            self.get_parameter("robot_tf_child_frame").value
        )
        self.robot_pose_publish_hz = float(
            self.get_parameter("robot_pose_publish_hz").value
        )
        self.robot_pose_follow_workspace = bool(
            self.get_parameter("robot_pose_follow_workspace").value
        )
        self.robot_pose_from_tag_pair = bool(
            self.get_parameter("robot_pose_from_tag_pair").value
        )
        self.robot_tag_pair_edge_offset_m = float(
            self.get_parameter("robot_tag_pair_edge_offset_m").value
        )
        self.robot_edge_clearance_m = float(
            self.get_parameter("robot_edge_clearance_m").value
        )
        self.robot_pose_x_offset_m = float(
            self.get_parameter("robot_pose_x_offset_m").value
        )
        self.robot_pose_x_m = float(self.get_parameter("robot_pose_x_m").value)
        self.robot_pose_y_m = float(self.get_parameter("robot_pose_y_m").value)
        self.robot_pose_z_m = float(self.get_parameter("robot_pose_z_m").value)
        self.robot_pose_yaw_deg = float(self.get_parameter("robot_pose_yaw_deg").value)
        self.robot_base_diameter_m = float(
            self.get_parameter("robot_base_diameter_m").value
        )
        self.robot_marker_height_m = float(
            self.get_parameter("robot_marker_height_m").value
        )
        self.workspace_origin_snap_to_bench_center = bool(
            self.get_parameter("workspace_origin_snap_to_bench_center").value
        )
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
        self.marker_lifetime_s = float(self.get_parameter("marker_lifetime_s").value)
        self.plane_marker_thickness_m = float(
            self.get_parameter("plane_marker_thickness_m").value
        )
        self.plane_marker_from_four_tags_enabled = bool(
            self.get_parameter("plane_marker_from_four_tags_enabled").value
        )
        self.plane_marker_four_tags_edge_offset_m = float(
            self.get_parameter("plane_marker_four_tags_edge_offset_m").value
        )
        self.axes_marker_length_m = float(
            self.get_parameter("axes_marker_length_m").value
        )
        self.axes_marker_radius_m = float(
            self.get_parameter("axes_marker_radius_m").value
        )
        self.axes_marker_lift_m = float(
            self.get_parameter("axes_marker_lift_m").value
        )
        self.object_marker_min_extent_m = float(
            self.get_parameter("object_marker_min_extent_m").value
        )
        self.object_marker_max_extent_m = float(
            self.get_parameter("object_marker_max_extent_m").value
        )
        self.object_source_mode = str(
            self.get_parameter("object_source_mode").value
        ).strip().lower()
        self.apriltag_object_pose_topic = str(
            self.get_parameter("apriltag_object_pose_topic").value
        )
        self.apriltag_object_timeout_s = float(
            self.get_parameter("apriltag_object_timeout_s").value
        )
        self.apriltag_object_size_m = float(
            self.get_parameter("apriltag_object_size_m").value
        )
        self.publish_object_marker = bool(
            self.get_parameter("publish_object_marker").value
        )

        self.random_seed = int(self.get_parameter("random_seed").value)

        if self.tag_frame_names:
            self.tag_frames = self.tag_frame_names[:2]
            self.tag_marker_frames = list(self.tag_frame_names)
        else:
            inferred_frames = [f"tag{self.tag_family}:{tag_id}" for tag_id in self.tag_ids]
            self.tag_frames = inferred_frames[:2]
            self.tag_marker_frames = inferred_frames

        if not self.tag_marker_frames:
            self.tag_marker_frames = list(self.tag_frames)
        if not self.tag_frames and self.tag_marker_frames:
            self.tag_frames = self.tag_marker_frames[:2]

        self.tag_marker_frame_to_id = {
            frame: idx for idx, frame in enumerate(self.tag_marker_frames)
        }
        self.workspace_front_edge_marker_id = 1000

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
        self.plane_marker_four_tags_edge_offset_m = max(
            0.0, self.plane_marker_four_tags_edge_offset_m
        )

        self.plane_fit_max_points = max(200, self.plane_fit_max_points)
        self.plane_fit_near_depth_quantile = min(
            max(self.plane_fit_near_depth_quantile, 0.05), 1.0
        )
        self.plane_ransac_iterations = max(20, self.plane_ransac_iterations)
        self.plane_inlier_threshold_m = max(0.001, self.plane_inlier_threshold_m)
        self.plane_min_inlier_ratio = min(max(self.plane_min_inlier_ratio, 0.05), 0.99)
        self.plane_min_inliers = max(100, self.plane_min_inliers)
        self.plane_smoothing_alpha = min(max(self.plane_smoothing_alpha, 0.0), 1.0)
        self.resnap_interval_s = max(0.0, self.resnap_interval_s)
        if not self.workspace_realign_service_name:
            self.workspace_realign_service_name = "/holoassist/perception/realign_workspace"

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
        self.object_candidate_min_top_height_m = max(
            0.0, self.object_candidate_min_top_height_m
        )
        self.object_marker_max_extent_m = max(
            self.object_marker_min_extent_m, self.object_marker_max_extent_m
        )
        if self.object_source_mode not in ("pointcloud", "apriltag"):
            self.object_source_mode = "apriltag"
        self.apriltag_object_timeout_s = max(0.05, self.apriltag_object_timeout_s)
        self.apriltag_object_size_m = max(0.01, self.apriltag_object_size_m)
        self.robot_base_diameter_m = max(0.01, self.robot_base_diameter_m)
        self.robot_marker_height_m = max(0.001, self.robot_marker_height_m)
        self.robot_tf_child_frame = self.robot_tf_child_frame.strip()
        if not self.robot_tf_child_frame:
            self.robot_tf_child_frame = "ur3e_base_link0"
        if self.robot_tf_child_frame == self.workspace_frame:
            self.get_logger().warn(
                "robot_tf_child_frame equals workspace_frame; disabling robot TF publish."
            )
            self.publish_robot_tf = False
        self.robot_pose_publish_hz = max(0.1, self.robot_pose_publish_hz)
        self.robot_tag_pair_edge_offset_m = max(0.0, self.robot_tag_pair_edge_offset_m)
        self.robot_edge_clearance_m = max(0.0, self.robot_edge_clearance_m)
        self.axes_marker_lift_m = max(0.0, self.axes_marker_lift_m)
        self.marker_lifetime_s = min(max(self.marker_lifetime_s, 0.0), 30.0)
        self.centroid_min_closer_m = max(0.0, self.centroid_min_closer_m)
        self.centroid_smoothing_alpha = min(max(self.centroid_smoothing_alpha, 0.0), 1.0)
        self.max_tracked_objects = max(1, self.max_tracked_objects)
        self.track_match_distance_m = max(0.01, self.track_match_distance_m)
        self.track_spawn_deadzone_m = max(0.0, self.track_spawn_deadzone_m)
        self.track_smoothing_alpha = min(max(self.track_smoothing_alpha, 0.0), 1.0)
        self.track_max_missed_frames = max(0, self.track_max_missed_frames)

        self.max_points_per_frame = max(500, self.max_points_per_frame)
        self.tag_size_m = max(0.01, self.tag_size_m)
        self._warn_expected_apriltag_size("tag_size_m", self.tag_size_m)
        self.tag_plane_projection_max_distance_m = max(
            0.0, self.tag_plane_projection_max_distance_m
        )
        if self.plane_marker_four_tags_edge_offset_m > 0.05:
            self.get_logger().warn(
                "plane_marker_four_tags_edge_offset_m=%.6f m is unusually large for 32 mm tags; check board tag center placement."
                % self.plane_marker_four_tags_edge_offset_m
            )
        self.tag_markers_raw_z_offset_m = float(self.tag_markers_raw_z_offset_m)
        if self.single_tag_axis_preference not in ("x", "y"):
            self.single_tag_axis_preference = "x"
        self.single_tag_left_axis_sign = (
            1.0 if self.single_tag_left_axis_sign >= 0.0 else -1.0
        )
        self.single_tag_right_axis_sign = (
            1.0 if self.single_tag_right_axis_sign >= 0.0 else -1.0
        )

    def _on_robot_pose_timer(self) -> None:
        self._publish_robot_pose(self.get_clock().now().to_msg())

    def _warn_expected_apriltag_size(self, name: str, value_m: float) -> None:
        expected = 0.032
        if abs(value_m - expected) > 1e-6:
            self.get_logger().warn(
                "%s=%.6f m but all current AprilTags are expected to be %.3f m."
                % (name, value_m, expected)
            )
        if value_m > 0.05:
            self.get_logger().warn(
                "%s=%.6f m is unusually large; workspace tag geometry may be wrong."
                % (name, value_m)
            )
        if value_m >= 1.0:
            self.get_logger().warn(
                "%s=%.6f looks like millimeters passed as meters (e.g. 32/40)."
                % (name, value_m)
            )

    def _on_apriltag_object_pose(self, msg: PoseStamped) -> None:
        self._latest_apriltag_object_pose = msg
        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds == 0:
                stamp = self.get_clock().now()
        except Exception:
            stamp = self.get_clock().now()
        self._latest_apriltag_object_pose_stamp = stamp

    def _on_realign_workspace_service(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        self._manual_realign_requested = True
        self._workspace_locked = False
        self._workspace_initial_snap_done = False
        response.success = True
        response.message = "workspace realign requested; waiting for next valid two-tag snap"
        self.get_logger().info("manual workspace realign requested via service call")
        return response

    def _apriltag_object_center_in_cloud(
        self,
        cloud_frame: str,
    ) -> Optional[np.ndarray]:
        if (
            self._latest_apriltag_object_pose is None
            or self._latest_apriltag_object_pose_stamp is None
        ):
            return None
        age_s = (
            self.get_clock().now() - self._latest_apriltag_object_pose_stamp
        ).nanoseconds / 1e9
        if age_s > self.apriltag_object_timeout_s:
            return None

        pose_msg = self._latest_apriltag_object_pose
        pose_frame = str(pose_msg.header.frame_id).strip()
        if not pose_frame:
            return None

        center_in_pose_frame = np.array(
            [
                float(pose_msg.pose.position.x),
                float(pose_msg.pose.position.y),
                float(pose_msg.pose.position.z),
            ],
            dtype=np.float32,
        )
        if pose_frame == cloud_frame:
            return center_in_pose_frame

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                cloud_frame,
                pose_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        rot = _rotation_matrix_from_quaternion(
            float(r.x),
            float(r.y),
            float(r.z),
            float(r.w),
        ).astype(np.float32)
        trans = np.array([float(t.x), float(t.y), float(t.z)], dtype=np.float32)
        return rot @ center_in_pose_frame + trans

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        self._frame_count += 1
        if self._manual_realign_requested:
            self._reset_workspace_state(
                msg.header.frame_id, msg.header.stamp, reason="manual_service"
            )
            self._last_resnap_time = self.get_clock().now()
            self._manual_realign_requested = False
        self._maybe_periodic_resnap(msg.header.frame_id, msg.header.stamp)

        xyz_all = self._extract_xyz(msg)
        if xyz_all is None or xyz_all.shape[0] < self.plane_min_inliers:
            self._tracks.clear()
            self._last_primary_track_id = None
            if self.publish_debug_markers:
                self._publish_workspace_markers_delete(msg.header.stamp)
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

        tag_observations = self._lookup_tag_observations(
            msg.header.frame_id, self.tag_frames
        )
        marker_tag_observations = tag_observations
        if self.tag_marker_frames != self.tag_frames:
            marker_tag_observations = self._lookup_tag_observations(
                msg.header.frame_id, self.tag_marker_frames
            )
        plane_tag_observations = (
            marker_tag_observations if marker_tag_observations else tag_observations
        )
        tag_plane = (
            self._plane_from_tags(plane_tag_observations)
            if self.bench_plane_from_tags_only
            else None
        )
        if tag_plane is not None:
            normal, d, plane_origin = tag_plane
            inlier_mask = np.ones(xyz.shape[0], dtype=bool)
            inlier_ratio = 1.0
            inlier_count = int(xyz.shape[0])
        else:
            if self.bench_plane_from_tags_only:
                self._tracks.clear()
                self._last_primary_track_id = None
                if self.publish_debug_markers:
                    self._publish_workspace_markers_delete(msg.header.stamp)
                self._publish_workspace_state(
                    mode="invalid",
                    level=DiagnosticStatus.ERROR,
                    message="missing tag plane",
                    extra={"tags_used": int(len(plane_tag_observations))},
                )
                self._publish_object_delete(msg.header.frame_id, msg.header.stamp)
                return
            plane_fit = self._fit_bench_plane(xyz)
            if plane_fit is None:
                self._tracks.clear()
                self._last_primary_track_id = None
                if self.publish_debug_markers:
                    self._publish_workspace_markers_delete(msg.header.stamp)
                self._publish_workspace_state(
                    mode="invalid",
                    level=DiagnosticStatus.ERROR,
                    message="plane fit failed",
                    extra={"points": int(xyz.shape[0])},
                )
                self._publish_object_delete(msg.header.frame_id, msg.header.stamp)
                return
            normal, d, inlier_mask, inlier_ratio, inlier_count, plane_origin = plane_fit

        tag_points_c_raw = {frame: value[0] for frame, value in tag_observations.items()}
        tag_points_c, tag_plane_projection_error_m, tag_projected_count = (
            self._project_tag_points_to_plane(
                tag_points_c=tag_points_c_raw,
                plane_normal=normal,
                plane_d=d,
            )
        )
        projected_tag_observations: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        for frame, (raw_point, rotation_c_t) in tag_observations.items():
            projected_point = tag_points_c.get(frame, raw_point)
            projected_tag_observations[frame] = (projected_point, rotation_c_t)
        basis = self._compute_workspace_basis(
            plane_normal=normal,
            plane_origin=plane_origin,
            tag_points_c=tag_points_c,
            tag_observations=projected_tag_observations,
        )
        if basis is None:
            self._tracks.clear()
            self._last_primary_track_id = None
            if self.publish_debug_markers:
                self._publish_workspace_markers_delete(msg.header.stamp)
            self._publish_workspace_state(
                mode="invalid",
                level=DiagnosticStatus.ERROR,
                message="could not derive workspace basis",
                extra={
                    "inliers": inlier_count,
                    "inlier_ratio": round(inlier_ratio, 3),
                    "tags_used": int(len(tag_points_c)),
                    "tags_projected": int(tag_projected_count),
                    "tag_plane_proj_err_m": round(tag_plane_projection_error_m, 4),
                },
            )
            self._publish_object_delete(msg.header.frame_id, msg.header.stamp)
            return

        basis_c, mode = basis
        if self._workspace_locked and self._last_origin_c is not None and self._last_basis_c is not None:
            origin_c = self._last_origin_c.copy()
            basis_c = self._last_basis_c.copy()
            roi_source = "locked"
        else:
            origin_c = plane_origin.copy()
            if self._last_origin_c is not None and self._last_basis_c is not None:
                alpha = self.plane_smoothing_alpha
                if alpha > 0.0:
                    origin_c = (1.0 - alpha) * self._last_origin_c + alpha * origin_c
                    z_axis = _normalize(
                        (1.0 - alpha) * self._last_basis_c[:, 2] + alpha * basis_c[:, 2]
                    )
                    x_axis = _normalize(
                        (1.0 - alpha) * self._last_basis_c[:, 0] + alpha * basis_c[:, 0]
                    )
                    if z_axis is not None and x_axis is not None:
                        y_axis = _normalize(np.cross(z_axis, x_axis))
                        if y_axis is not None:
                            x_axis = _normalize(np.cross(y_axis, z_axis))
                            if x_axis is not None:
                                basis_c = np.column_stack((x_axis, y_axis, z_axis))

            roi_source = self._compute_active_roi_bounds(tag_points_c, origin_c, basis_c)
            if self.workspace_origin_snap_to_bench_center:
                origin_c = self._snap_workspace_origin_to_bench_center(origin_c, basis_c)

            self._last_origin_c = origin_c
            self._last_basis_c = basis_c

            if (
                self.lock_workspace_after_initial_two_tag_snap
                and mode == "plane_plus_tags"
                and not self._workspace_initial_snap_done
            ):
                self._workspace_locked = True
                self._workspace_initial_snap_done = True
                self.get_logger().info(
                    "workspace locked after initial two-tag snap; use service %s to realign."
                    % self.workspace_realign_service_name
                )

        self._last_plane = (float(normal[0]), float(normal[1]), float(normal[2]), float(d))
        self._publish_plane_coefficients(normal, d)

        if self.publish_tf:
            self._broadcast_workspace_tf(
                parent_frame=msg.header.frame_id,
                stamp=msg.header.stamp,
                origin_c=origin_c,
                basis_c=basis_c,
            )

        if self.publish_debug_markers:
            tag_marker_points_c_raw = {
                frame: value[0] for frame, value in marker_tag_observations.items()
            }
            tag_marker_points_c, _, _ = self._project_tag_points_to_plane(
                tag_points_c=tag_marker_points_c_raw,
                plane_normal=normal,
                plane_d=d,
            )
            tag_marker_points_c = (
                tag_marker_points_c
                if self.tag_markers_use_projected_points
                else tag_marker_points_c_raw
            )
            self._publish_plane_marker(
                msg.header.stamp, tag_marker_points_c, origin_c, basis_c
            )
            self._publish_axes_markers(msg.header.stamp)
            self._publish_tag_markers(
                msg.header.stamp,
                tag_marker_points_c,
                origin_c,
                basis_c,
                force_plane_lift=self.tag_markers_use_projected_points,
                flatten_z_to_plane=self.tag_markers_flatten_z_to_plane,
            )

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

        if self.object_source_mode == "apriltag":
            apriltag_center_c = self._apriltag_object_center_in_cloud(msg.header.frame_id)
            if apriltag_center_c is None:
                self._last_published_center_w = None
                self._last_published_center_c = None
                self._last_published_extent_w = None
                self._last_primary_track_id = None
                self._object_persistence_score *= self.persistence_decay
                self._publish_object_delete(msg.header.frame_id, msg.header.stamp)
                self._publish_workspace_state(
                    mode=mode,
                    level=self._workspace_mode_level(mode),
                    message="workspace updated; no apriltag object pose",
                    extra={
                        "inliers": inlier_count,
                        "inlier_ratio": round(inlier_ratio, 3),
                        "cropped_points": int(cropped_points_c.shape[0]),
                        "foreground_points": int(fg_points_c.shape[0]),
                        "cluster_count": 0,
                        "roi_source": roi_source,
                        "roi_x_min": round(self._active_roi_x_min, 3),
                        "roi_x_max": round(self._active_roi_x_max, 3),
                        "roi_y_min": round(self._active_roi_y_min, 3),
                        "roi_y_max": round(self._active_roi_y_max, 3),
                        "culled_below_plane": culled_below_plane,
                        "tags_used": int(len(tag_points_c)),
                        "tags_projected": int(tag_projected_count),
                        "tag_plane_proj_err_m": round(tag_plane_projection_error_m, 4),
                    },
                )
                self._publish_object_metrics(
                    foreground_points=int(fg_points_c.shape[0]),
                    cluster_count=0,
                    selected_points=0,
                    center_w=None,
                    persistence=self._object_persistence_score,
                )
                return

            apriltag_center_w = self._to_workspace(
                apriltag_center_c.reshape(1, 3), origin_c, basis_c
            )[0]
            apriltag_extent_w = np.array(
                [self.apriltag_object_size_m] * 3, dtype=np.float32
            )
            self._last_published_center_w = apriltag_center_w.copy()
            self._last_published_center_c = apriltag_center_c.copy()
            self._last_published_extent_w = apriltag_extent_w.copy()
            self._last_primary_track_id = 1
            self._object_persistence_score = min(
                1.0, self._object_persistence_score + self.persistence_gain
            )

            object_pose_c = PoseStamped()
            object_pose_c.header.frame_id = msg.header.frame_id
            object_pose_c.header.stamp = msg.header.stamp
            object_pose_c.pose.position.x = float(apriltag_center_c[0])
            object_pose_c.pose.position.y = float(apriltag_center_c[1])
            object_pose_c.pose.position.z = float(apriltag_center_c[2])
            object_pose_c.pose.orientation.w = 1.0
            self.object_pose_pub.publish(object_pose_c)

            object_pose_w = PoseStamped()
            object_pose_w.header.frame_id = self.workspace_frame
            object_pose_w.header.stamp = msg.header.stamp
            object_pose_w.pose.position.x = float(apriltag_center_w[0])
            object_pose_w.pose.position.y = float(apriltag_center_w[1])
            object_pose_w.pose.position.z = float(apriltag_center_w[2])
            object_pose_w.pose.orientation.w = 1.0
            self.object_pose_workspace_pub.publish(object_pose_w)

            self._publish_object_markers(
                frame_id=msg.header.frame_id,
                stamp=msg.header.stamp,
                origin_c=origin_c,
                basis_c=basis_c,
                tracked_objects=[
                    {
                        "track_id": 1,
                        "center_w": apriltag_center_w,
                        "extent_w": apriltag_extent_w,
                        "points": 1,
                    }
                ],
                primary_track_id=1,
                primary_center_w=apriltag_center_w,
                primary_extent_w=apriltag_extent_w,
            )
            self._publish_workspace_state(
                mode=mode,
                level=self._workspace_mode_level(mode),
                message="workspace and apriltag object updated",
                extra={
                    "inliers": inlier_count,
                    "inlier_ratio": round(inlier_ratio, 3),
                    "cropped_points": int(cropped_points_c.shape[0]),
                    "foreground_points": int(fg_points_c.shape[0]),
                    "cluster_count": 1,
                    "selected_points": 1,
                    "culled_below_plane": culled_below_plane,
                    "roi_source": roi_source,
                    "roi_x_min": round(self._active_roi_x_min, 3),
                    "roi_x_max": round(self._active_roi_x_max, 3),
                    "roi_y_min": round(self._active_roi_y_min, 3),
                    "roi_y_max": round(self._active_roi_y_max, 3),
                    "tracked_objects": 1,
                    "primary_track_id": 1,
                    "centroid_updated": True,
                    "centroid_closer_delta_m": 0.0,
                    "tags_used": int(len(tag_points_c)),
                    "tags_projected": int(tag_projected_count),
                    "tag_plane_proj_err_m": round(tag_plane_projection_error_m, 4),
                },
            )
            self._publish_object_metrics(
                foreground_points=int(fg_points_c.shape[0]),
                cluster_count=1,
                selected_points=1,
                center_w=apriltag_center_w,
                persistence=self._object_persistence_score,
            )
            return

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
                level=self._workspace_mode_level(mode),
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
                    "tags_projected": int(tag_projected_count),
                    "tag_plane_proj_err_m": round(tag_plane_projection_error_m, 4),
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
            level=self._workspace_mode_level(mode),
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
                "tags_projected": int(tag_projected_count),
                "tag_plane_proj_err_m": round(tag_plane_projection_error_m, 4),
            },
        )
        self._publish_object_metrics(
            foreground_points=int(fg_points_c.shape[0]),
            cluster_count=int(cluster_count),
            selected_points=int(selected_points),
            center_w=center_w,
            persistence=self._object_persistence_score,
        )

    def _maybe_periodic_resnap(self, frame_id: str, stamp) -> None:
        if self.resnap_interval_s <= 0.0:
            return

        now = self.get_clock().now()
        elapsed_s = (now - self._last_resnap_time).nanoseconds / 1e9
        if elapsed_s < self.resnap_interval_s:
            return

        self._reset_workspace_state(frame_id, stamp, reason="periodic_timer")
        self._last_resnap_time = now

    def _reset_workspace_state(self, frame_id: str, stamp, reason: str) -> None:
        self._last_origin_c = None
        self._last_basis_c = None
        self._last_plane = None
        self._workspace_locked = False
        self._workspace_initial_snap_done = False

        self._active_roi_x_min = self.roi_x_min
        self._active_roi_x_max = self.roi_x_max
        self._active_roi_y_min = self.roi_y_min
        self._active_roi_y_max = self.roi_y_max

        if self.resnap_clear_background:
            self._background_scores.clear()
            self._background_warmup_until_frame = (
                self._frame_count + self.background_warmup_frames
            )

        if self.resnap_clear_tracks:
            self._tracks.clear()
            self._last_primary_track_id = None
            self._last_published_center_w = None
            self._last_published_center_c = None
            self._last_published_extent_w = None
            self._object_persistence_score = 0.0
            if self._last_marker_ids:
                self._publish_object_delete(frame_id, stamp)

        self._resnap_count += 1
        self._last_resnap_frame = self._frame_count
        self._last_resnap_reason = reason
        self.get_logger().info(
            "workspace re-snap #%d reason=%s clear_background=%s clear_tracks=%s"
            % (
                self._resnap_count,
                reason,
                self.resnap_clear_background,
                self.resnap_clear_tracks,
            )
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
        points_for_fit = points
        if (
            self.plane_fit_near_depth_quantile < 0.999
            and points.shape[0] >= max(50, self.plane_min_inliers)
        ):
            depth = points[:, 2]
            finite_depth = np.isfinite(depth)
            if np.any(finite_depth):
                depth_vals = depth[finite_depth]
                if depth_vals.size >= max(50, self.plane_min_inliers):
                    depth_cutoff = float(
                        np.quantile(depth_vals, self.plane_fit_near_depth_quantile)
                    )
                    near_mask = depth <= depth_cutoff
                    near_points = points[near_mask]
                    if near_points.shape[0] >= max(50, self.plane_min_inliers):
                        points_for_fit = near_points

        n_points = points_for_fit.shape[0]
        if n_points < 3:
            return None

        fit_points = points_for_fit
        if n_points > self.plane_fit_max_points:
            idx = self._rng.choice(n_points, self.plane_fit_max_points, replace=False)
            fit_points = points_for_fit[idx]

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

        full_dist = np.abs(points_for_fit @ normal + d)
        full_inlier_mask = full_dist <= self.plane_inlier_threshold_m
        full_inliers = int(np.count_nonzero(full_inlier_mask))
        full_ratio = full_inliers / max(1, points_for_fit.shape[0])
        if full_inliers < self.plane_min_inliers or full_ratio < self.plane_min_inlier_ratio:
            return None

        full_inlier_points = points_for_fit[full_inlier_mask]
        plane_origin = np.mean(full_inlier_points, axis=0)
        plane_origin = plane_origin - (np.dot(normal, plane_origin) + d) * normal

        return normal, d, full_inlier_mask, full_ratio, full_inliers, plane_origin

    def _lookup_tag_observations(
        self,
        cloud_frame: str,
        frames: Optional[List[str]] = None,
    ) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
        observations: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        frames_to_use = list(frames) if frames is not None else list(self.tag_frames)
        if not self.use_tag_refinement or len(frames_to_use) < 1:
            return observations

        now = self.get_clock().now()
        for frame in frames_to_use:
            try:
                tf_msg = self.tf_buffer.lookup_transform(cloud_frame, frame, rclpy.time.Time())
            except TransformException:
                continue

            stamp = rclpy.time.Time.from_msg(tf_msg.header.stamp)
            # Some TF publishers can emit tag transforms with a zero stamp
            # (timeless/static semantics). Accept those instead of rejecting
            # them as infinitely stale.
            if stamp.nanoseconds != 0:
                age_s = (now - stamp).nanoseconds / 1e9
                if age_s < 0.0:
                    age_s = abs(age_s)
                if age_s > self.tag_timeout_s:
                    continue

            translation = tf_msg.transform.translation
            rotation = tf_msg.transform.rotation
            point_c = np.array(
                [translation.x, translation.y, translation.z], dtype=np.float32
            )
            rot_c_t = _rotation_matrix_from_quaternion(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            )
            observations[frame] = (point_c, rot_c_t)

        return observations

    def _plane_from_tags(
        self, tag_observations: Dict[str, Tuple[np.ndarray, np.ndarray]]
    ) -> Optional[Tuple[np.ndarray, float, np.ndarray]]:
        if not tag_observations:
            return None

        normals: List[np.ndarray] = []
        points: List[np.ndarray] = []
        for frame in self.tag_frames:
            value = tag_observations.get(frame)
            if value is None:
                continue
            point_c, rot_c_t = value
            n = _normalize(np.asarray(rot_c_t[:, 2], dtype=np.float32))
            if n is None:
                continue
            normals.append(n)
            points.append(np.asarray(point_c, dtype=np.float32))

        if not normals or not points:
            return None

        normal = _normalize(np.mean(np.stack(normals, axis=0), axis=0))
        if normal is None:
            return None

        origin = np.mean(np.stack(points, axis=0), axis=0).astype(np.float32)
        sensor_vec = -origin
        if float(np.dot(normal, sensor_vec)) < 0.0:
            normal = -normal

        d = -float(np.dot(normal, origin))
        origin = origin - (np.dot(normal, origin) + d) * normal
        return normal.astype(np.float32), d, origin.astype(np.float32)

    def _project_tag_points_to_plane(
        self,
        tag_points_c: Dict[str, np.ndarray],
        plane_normal: np.ndarray,
        plane_d: float,
    ) -> Tuple[Dict[str, np.ndarray], float, int]:
        if not tag_points_c:
            return {}, 0.0, 0
        if not self.project_tags_to_plane:
            return tag_points_c, 0.0, 0

        projected: Dict[str, np.ndarray] = {}
        sum_abs_distance = 0.0
        projected_count = 0
        max_distance = self.tag_plane_projection_max_distance_m

        for frame, point in tag_points_c.items():
            signed_distance = float(np.dot(point, plane_normal) + plane_d)
            if max_distance > 0.0 and abs(signed_distance) > max_distance:
                projected[frame] = point
                continue
            projected[frame] = (point - signed_distance * plane_normal).astype(
                np.float32
            )
            projected_count += 1
            sum_abs_distance += abs(signed_distance)

        mean_abs_distance = (
            sum_abs_distance / float(projected_count) if projected_count > 0 else 0.0
        )
        return projected, mean_abs_distance, projected_count

    def _compute_workspace_basis(
        self,
        plane_normal: np.ndarray,
        plane_origin: np.ndarray,
        tag_points_c: Dict[str, np.ndarray],
        tag_observations: Dict[str, Tuple[np.ndarray, np.ndarray]],
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
                        self._last_pair_x_axis_c = x_axis.copy()

        if x_axis is None and tag_observations:
            if self._last_pair_x_axis_c is not None:
                x_from_pair = _normalize(
                    self._last_pair_x_axis_c
                    - np.dot(self._last_pair_x_axis_c, z_axis) * z_axis
                )
                if x_from_pair is not None:
                    x_axis = x_from_pair
                    mode = "plane_plus_single_tag_locked"

            observed_frames = [frame for frame in self.tag_frames if frame in tag_observations]
            if observed_frames and x_axis is None:
                frame = observed_frames[0]
                _, rot_c_t = tag_observations[frame]

                axis_x = rot_c_t[:, 0]
                axis_y = rot_c_t[:, 1]
                axis_x_proj = _normalize(axis_x - np.dot(axis_x, z_axis) * z_axis)
                axis_y_proj = _normalize(axis_y - np.dot(axis_y, z_axis) * z_axis)

                preferred = (
                    axis_x_proj
                    if self.single_tag_axis_preference == "x"
                    else axis_y_proj
                )
                fallback = axis_y_proj if preferred is axis_x_proj else axis_x_proj
                x_axis = preferred if preferred is not None else fallback

                if x_axis is not None:
                    if frame == self.tag_frames[0]:
                        side_sign = self.single_tag_left_axis_sign
                    elif len(self.tag_frames) >= 2 and frame == self.tag_frames[1]:
                        side_sign = self.single_tag_right_axis_sign
                    else:
                        side_sign = 1.0
                    if side_sign < 0.0:
                        x_axis = -x_axis

                    if self._last_basis_c is not None:
                        prev_x = self._last_basis_c[:, 0]
                        if float(np.dot(x_axis, prev_x)) < 0.0:
                            x_axis = -x_axis

                    mode = "plane_plus_single_tag"

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

        if self.roi_from_tags_enabled and len(tag_points_c) >= 1:
            ordered_points_w: List[Tuple[str, np.ndarray]] = []
            for frame in self.tag_frames:
                if frame not in tag_points_c:
                    continue
                point_w = self._to_workspace(
                    tag_points_c[frame].reshape(1, 3), origin_c, basis_c
                )[0]
                ordered_points_w.append((frame, point_w))

            if ordered_points_w:
                tag_arr = np.asarray([item[1] for item in ordered_points_w], dtype=np.float32)
                cfg_span_x = max(0.02, self.roi_x_max - self.roi_x_min)
                cfg_span_y = max(0.02, self.roi_y_max - self.roi_y_min)
                half_tag = 0.5 * max(0.01, self.tag_size_m)
                half_x = 0.5 * cfg_span_x
                center_offset_x = max(0.0, half_x - half_tag)

                if len(ordered_points_w) >= 2:
                    # Pair anchor:
                    # - left board corner is 0.5*tag_size to the left of left-tag center
                    # - right board corner is 0.5*tag_size to the right of right-tag center
                    # This keeps each tag center at half the configured tag size from
                    # its assigned corner (16 mm when tag_size_m=0.032), regardless of
                    # temporary tag separation noise.
                    frame_to_point = {frame: point for frame, point in ordered_points_w}
                    left_frame = self.tag_frames[0]
                    right_frame = self.tag_frames[1] if len(self.tag_frames) >= 2 else None

                    if right_frame is not None and left_frame in frame_to_point and right_frame in frame_to_point:
                        left_x = float(frame_to_point[left_frame][0])
                        right_x = float(frame_to_point[right_frame][0])
                        tx_min = left_x - half_tag
                        tx_max = right_x + half_tag
                        front_center_x = 0.5 * (tx_min + tx_max)
                        roi_source = "tag_front_pair_assigned_corners"
                    else:
                        center_x_candidates: List[float] = []
                        for frame, point_w in ordered_points_w:
                            if frame == self.tag_frames[0]:
                                center_x_candidates.append(float(point_w[0]) + center_offset_x)
                            elif len(self.tag_frames) >= 2 and frame == self.tag_frames[1]:
                                center_x_candidates.append(float(point_w[0]) - center_offset_x)
                            else:
                                center_x_candidates.append(float(point_w[0]))
                        front_center_x = float(np.mean(center_x_candidates))
                        tx_min = front_center_x - half_x
                        tx_max = front_center_x + half_x
                        roi_source = "tag_front_pair"

                    self._last_pair_front_center_x = front_center_x
                    self._last_pair_tag_points_w = {
                        frame: point.copy() for frame, point in ordered_points_w
                    }
                else:
                    frame, only_point = ordered_points_w[0]
                    if (
                        frame in self._last_pair_tag_points_w
                        and self._last_pair_front_center_x is not None
                    ):
                        prev_tag = self._last_pair_tag_points_w[frame]
                        delta = only_point - prev_tag
                        front_center_x = self._last_pair_front_center_x + float(delta[0])
                        roi_source = "tag_single_locked"
                    else:
                        if frame == self.tag_frames[0]:
                            front_center_x = float(only_point[0]) + center_offset_x
                            roi_source = "tag_single_left"
                        elif len(self.tag_frames) >= 2 and frame == self.tag_frames[1]:
                            front_center_x = float(only_point[0]) - center_offset_x
                            roi_source = "tag_single_right"
                        else:
                            front_center_x = float(only_point[0])
                            roi_source = "tag_single_unknown"

                if len(ordered_points_w) < 2:
                    tx_min = front_center_x - half_x
                    tx_max = front_center_x + half_x

                if self.roi_from_tags_use_front_edge:
                    tag_mid_y = float(np.mean(tag_arr[:, 1]))
                    if (
                        len(ordered_points_w) == 1
                        and frame in self._last_pair_tag_points_w
                        and self._last_pair_front_edge_y is not None
                        and self._last_pair_interior_sign is not None
                    ):
                        prev_tag = self._last_pair_tag_points_w[frame]
                        delta = only_point - prev_tag
                        interior_sign = self._last_pair_interior_sign
                        front_edge_y = self._last_pair_front_edge_y + float(delta[1])
                        back_edge_y = front_edge_y + interior_sign * cfg_span_y
                        roi_source = "tag_single_locked_front_edge"
                    else:
                        camera_w = self._to_workspace(
                            np.zeros((1, 3), dtype=np.float32), origin_c, basis_c
                        )[0]
                        # Camera-side of the tag line is treated as workspace front.
                        interior_sign = -1.0 if camera_w[1] >= tag_mid_y else 1.0
                        if len(ordered_points_w) >= 2:
                            front_edge_candidates: List[float] = []
                            for _, point_w in ordered_points_w:
                                front_edge_candidates.append(
                                    float(point_w[1]) - interior_sign * half_tag
                                )
                            front_edge_y = float(np.mean(front_edge_candidates))
                        else:
                            front_edge_y = tag_mid_y - interior_sign * half_tag
                        back_edge_y = front_edge_y + interior_sign * cfg_span_y

                    if len(ordered_points_w) >= 2:
                        self._last_pair_front_edge_y = float(front_edge_y)
                        self._last_pair_interior_sign = float(interior_sign)

                    ty_min = min(front_edge_y, back_edge_y) - self.roi_from_tags_margin_m
                    ty_max = max(front_edge_y, back_edge_y) + self.roi_from_tags_margin_m
                    roi_source = f"{roi_source}_front_edge"
                else:
                    tag_center_y = float(np.mean(tag_arr[:, 1]))
                    half_y = 0.5 * cfg_span_y
                    ty_min = tag_center_y - half_y - self.roi_from_tags_margin_m
                    ty_max = tag_center_y + half_y + self.roi_from_tags_margin_m
                    roi_source = f"{roi_source}_centered"

                target_x_min = tx_min
                target_x_max = tx_max
                target_y_min = ty_min
                target_y_max = ty_max

        alpha = self.roi_from_tags_smoothing_alpha if roi_source.startswith("tag_") else 1.0
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

    def _workspace_mode_level(self, mode: str) -> int:
        if mode == "plane_plus_tags" or mode.startswith("plane_plus_single_tag"):
            return DiagnosticStatus.OK
        return DiagnosticStatus.WARN

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

    def _shift_workspace_state(self, dx: float, dy: float) -> None:
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return

        self._active_roi_x_min += dx
        self._active_roi_x_max += dx
        self._active_roi_y_min += dy
        self._active_roi_y_max += dy

        if self._last_pair_front_center_x is not None:
            self._last_pair_front_center_x += dx
        if self._last_pair_front_edge_y is not None:
            self._last_pair_front_edge_y += dy

        if self._last_pair_tag_points_w:
            shifted: Dict[str, np.ndarray] = {}
            for frame, point in self._last_pair_tag_points_w.items():
                p = np.asarray(point, dtype=np.float32).copy()
                p[0] += dx
                p[1] += dy
                shifted[frame] = p
            self._last_pair_tag_points_w = shifted

        if self._last_object_center_w is not None:
            self._last_object_center_w = np.asarray(
                [
                    float(self._last_object_center_w[0]) + dx,
                    float(self._last_object_center_w[1]) + dy,
                    float(self._last_object_center_w[2]),
                ],
                dtype=np.float32,
            )

        if self._last_published_center_w is not None:
            self._last_published_center_w = np.asarray(
                [
                    float(self._last_published_center_w[0]) + dx,
                    float(self._last_published_center_w[1]) + dy,
                    float(self._last_published_center_w[2]),
                ],
                dtype=np.float32,
            )

        for track in self._tracks.values():
            center_w = np.asarray(track.get("center_w", np.zeros(3)), dtype=np.float32).copy()
            center_w[0] += dx
            center_w[1] += dy
            track["center_w"] = center_w

    def _snap_workspace_origin_to_bench_center(
        self, origin_c: np.ndarray, basis_c: np.ndarray
    ) -> np.ndarray:
        center_x_w = float((self._active_roi_x_min + self._active_roi_x_max) * 0.5)
        center_y_w = float((self._active_roi_y_min + self._active_roi_y_max) * 0.5)
        if abs(center_x_w) < 1e-9 and abs(center_y_w) < 1e-9:
            return origin_c

        center_w = np.array([center_x_w, center_y_w, 0.0], dtype=np.float32)
        snapped_origin_c = self._workspace_to_camera(center_w, origin_c, basis_c).astype(
            np.float32
        )
        # New workspace frame is translated by the ROI center, so convert cached
        # workspace-state values to the translated frame.
        self._shift_workspace_state(-center_x_w, -center_y_w)
        shift_m = math.hypot(center_x_w, center_y_w)
        # Large frame re-anchors can invalidate workspace-voxel history; restart
        # background/tracking state so perception settles quickly in the new frame.
        if shift_m > 0.05:
            self._background_scores.clear()
            self._background_warmup_until_frame = (
                self._frame_count + self.background_warmup_frames
            )
            self._tracks.clear()
            self._last_primary_track_id = None
            self._last_published_center_w = None
            self._last_published_center_c = None
            self._last_published_extent_w = None
            self._object_persistence_score = 0.0
        return snapped_origin_c

    def _infer_robot_far_side_sign(self, board_center_y_w: float) -> float:
        if self._last_pair_interior_sign is not None:
            return 1.0 if self._last_pair_interior_sign >= 0.0 else -1.0

        if self._last_pair_tag_points_w:
            tag_ys = [float(point[1]) for point in self._last_pair_tag_points_w.values()]
            if tag_ys:
                tag_mid_y = float(np.mean(tag_ys))
                return 1.0 if board_center_y_w >= tag_mid_y else -1.0

        return 1.0

    def _compute_robot_pose_workspace(self) -> Tuple[float, float, float, float]:
        if not self.robot_pose_follow_workspace:
            return (
                float(self.robot_pose_x_m),
                float(self.robot_pose_y_m),
                float(self.robot_pose_z_m),
                float(self.robot_pose_yaw_deg),
            )

        center_x_w = float((self._active_roi_x_min + self._active_roi_x_max) * 0.5)
        center_y_w = float((self._active_roi_y_min + self._active_roi_y_max) * 0.5)
        far_sign = self._infer_robot_far_side_sign(center_y_w)
        board_back_edge_y_w = (
            float(self._active_roi_y_max)
            if far_sign >= 0.0
            else float(self._active_roi_y_min)
        )

        robot_x_w = center_x_w + float(self.robot_pose_x_offset_m)
        # Always measure offset from the board back edge of the active blue plane.
        # This keeps robot placement invariant to tag size/placement details.
        edge_offset_m = float(self.robot_tag_pair_edge_offset_m) + float(
            self.robot_edge_clearance_m
        )
        robot_y_w = board_back_edge_y_w + far_sign * edge_offset_m
        return (
            float(robot_x_w),
            float(robot_y_w),
            float(self.robot_pose_z_m),
            float(self.robot_pose_yaw_deg),
        )

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

        if self._frame_count <= self._background_warmup_until_frame:
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

            # Keep near-plane speckle in clouds, but do not let it qualify as an
            # object marker candidate unless the cluster rises above threshold.
            top_height_m = float(maxs[2])
            if top_height_m < self.object_candidate_min_top_height_m:
                continue

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

    def _apply_marker_lifetime(self, marker: Marker) -> None:
        if self.marker_lifetime_s <= 0.0:
            return
        sec = int(self.marker_lifetime_s)
        nanosec = int((self.marker_lifetime_s - float(sec)) * 1e9)
        marker.lifetime.sec = max(0, sec)
        marker.lifetime.nanosec = max(0, min(999999999, nanosec))

    def _publish_workspace_markers_delete(self, stamp) -> None:
        plane = Marker()
        plane.header.frame_id = self.workspace_frame
        plane.header.stamp = stamp
        plane.ns = "holoassist_bench_plane"
        plane.id = 0
        plane.action = Marker.DELETE
        self.plane_marker_pub.publish(plane)

        for marker_id in self.tag_marker_frame_to_id.values():
            tags = Marker()
            tags.header.frame_id = self.workspace_frame
            tags.header.stamp = stamp
            tags.ns = "holoassist_workspace_tags"
            tags.id = marker_id
            tags.action = Marker.DELETE
            self.tag_marker_pub.publish(tags)

        front_edge = Marker()
        front_edge.header.frame_id = self.workspace_frame
        front_edge.header.stamp = stamp
        front_edge.ns = "holoassist_workspace_tags"
        front_edge.id = self.workspace_front_edge_marker_id
        front_edge.action = Marker.DELETE
        self.tag_marker_pub.publish(front_edge)

        for idx in range(3):
            axis = Marker()
            axis.header.frame_id = self.workspace_frame
            axis.header.stamp = stamp
            axis.ns = "holoassist_workspace_axes"
            axis.id = idx
            axis.action = Marker.DELETE
            self.axes_marker_pub.publish(axis)

    def _plane_marker_bounds_from_four_tags(
        self,
        tag_points_c: Dict[str, np.ndarray],
        origin_c: np.ndarray,
        basis_c: np.ndarray,
    ) -> Optional[Tuple[float, float, float, float]]:
        """Return workspace x/y bounds for the bench marker from the first 4 tag centres.

        The four-tag board layout has known physical geometry, so when all four
        marker tag centres are visible we can draw the bench plane marker from
        those centres plus the configured centre-to-edge offset. This prevents
        the bench marker from shrinking to a 2-tag front-edge estimate.
        """
        if not self.plane_marker_from_four_tags_enabled:
            return None
        if len(self.tag_marker_frames) < 4:
            return None

        points_w: List[np.ndarray] = []
        for frame in self.tag_marker_frames[:4]:
            point_c = tag_points_c.get(frame)
            if point_c is None:
                return None
            point_w = self._to_workspace(point_c.reshape(1, 3), origin_c, basis_c)[0]
            if not np.isfinite(point_w).all():
                return None
            points_w.append(point_w)

        tag_arr = np.asarray(points_w, dtype=np.float32)
        if tag_arr.shape != (4, 3):
            return None

        edge = float(self.plane_marker_four_tags_edge_offset_m)
        x_min = float(np.min(tag_arr[:, 0]) - edge)
        x_max = float(np.max(tag_arr[:, 0]) + edge)
        y_min = float(np.min(tag_arr[:, 1]) - edge)
        y_max = float(np.max(tag_arr[:, 1]) + edge)

        if not all(np.isfinite([x_min, x_max, y_min, y_max])):
            return None
        if x_max <= x_min or y_max <= y_min:
            return None
        return x_min, x_max, y_min, y_max

    def _publish_plane_marker(
        self,
        stamp,
        tag_points_c: Optional[Dict[str, np.ndarray]] = None,
        origin_c: Optional[np.ndarray] = None,
        basis_c: Optional[np.ndarray] = None,
    ) -> None:
        bounds = None
        if (
            tag_points_c is not None
            and origin_c is not None
            and basis_c is not None
        ):
            bounds = self._plane_marker_bounds_from_four_tags(
                tag_points_c, origin_c, basis_c
            )

        if bounds is None:
            x_min = float(self._active_roi_x_min)
            x_max = float(self._active_roi_x_max)
            y_min = float(self._active_roi_y_min)
            y_max = float(self._active_roi_y_max)
        else:
            x_min, x_max, y_min, y_max = bounds

        marker = Marker()
        marker.header.frame_id = self.workspace_frame
        marker.header.stamp = stamp
        marker.ns = "holoassist_bench_plane"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float((x_min + x_max) * 0.5)
        marker.pose.position.y = float((y_min + y_max) * 0.5)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(max(0.02, x_max - x_min))
        marker.scale.y = float(max(0.02, y_max - y_min))
        marker.scale.z = float(max(0.001, self.plane_marker_thickness_m))
        marker.color.r = 0.2
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = float(np.clip(self.marker_alpha, 0.05, 1.0))
        self._apply_marker_lifetime(marker)
        self.plane_marker_pub.publish(marker)

    def _publish_tag_markers(
        self,
        stamp,
        tag_points_c: Dict[str, np.ndarray],
        origin_c: np.ndarray,
        basis_c: np.ndarray,
        force_plane_lift: bool,
        flatten_z_to_plane: bool,
    ) -> None:
        ordered_tag_points_w: Dict[str, np.ndarray] = {}
        nominal_tag_size = max(0.01, self.tag_size_m)
        # In projected mode, keep tag visuals slightly above bench plane.
        # In raw-TF mode, use TF Z (plus optional offset) for true 3D centering.
        tag_visual_lift_m = max(
            0.001,
            float(self.plane_marker_thickness_m) + 0.5 * max(0.002, 0.05 * nominal_tag_size),
        )

        for frame in self.tag_marker_frames:
            marker = Marker()
            marker.header.frame_id = self.workspace_frame
            marker.header.stamp = stamp
            marker.ns = "holoassist_workspace_tags"
            marker.id = self.tag_marker_frame_to_id[frame]
            marker.type = Marker.CUBE
            marker.pose.orientation.w = 1.0
            marker.scale.x = nominal_tag_size
            marker.scale.y = nominal_tag_size
            marker.scale.z = max(0.002, 0.05 * nominal_tag_size)
            marker.color.r = 1.0
            marker.color.g = 0.9
            marker.color.b = 0.2
            marker.color.a = 0.9
            self._apply_marker_lifetime(marker)

            if frame in tag_points_c:
                point_w = self._to_workspace(
                    tag_points_c[frame].reshape(1, 3), origin_c, basis_c
                )[0]
                if force_plane_lift or flatten_z_to_plane:
                    point_w[2] = tag_visual_lift_m
                else:
                    point_w[2] = float(point_w[2]) + float(self.tag_markers_raw_z_offset_m)
                marker.action = Marker.ADD
                marker.pose.position.x = float(point_w[0])
                marker.pose.position.y = float(point_w[1])
                marker.pose.position.z = float(point_w[2])
                ordered_tag_points_w[frame] = point_w
            else:
                marker.action = Marker.DELETE

            self.tag_marker_pub.publish(marker)

        front_edge = Marker()
        front_edge.header.frame_id = self.workspace_frame
        front_edge.header.stamp = stamp
        front_edge.ns = "holoassist_workspace_tags"
        front_edge.id = self.workspace_front_edge_marker_id
        front_edge.type = Marker.LINE_STRIP
        front_edge.action = Marker.ADD
        front_edge.pose.orientation.w = 1.0
        front_edge.scale.x = max(0.002, 0.10 * nominal_tag_size)
        front_edge.color.r = 1.0
        front_edge.color.g = 0.6
        front_edge.color.b = 0.1
        front_edge.color.a = 0.95
        self._apply_marker_lifetime(front_edge)

        if (
            len(self.tag_frames) >= 2
            and self.tag_frames[0] in ordered_tag_points_w
            and self.tag_frames[1] in ordered_tag_points_w
        ):
            p0 = Point()
            p0.x = float(ordered_tag_points_w[self.tag_frames[0]][0])
            p0.y = float(ordered_tag_points_w[self.tag_frames[0]][1])
            p0.z = float(ordered_tag_points_w[self.tag_frames[0]][2])
            p1 = Point()
            p1.x = float(ordered_tag_points_w[self.tag_frames[1]][0])
            p1.y = float(ordered_tag_points_w[self.tag_frames[1]][1])
            p1.z = float(ordered_tag_points_w[self.tag_frames[1]][2])
            front_edge.points = [p0, p1]
        else:
            front_edge.action = Marker.DELETE

        self.tag_marker_pub.publish(front_edge)

    def _publish_axes_markers(self, stamp) -> None:
        axes_lift_m = (
            float(self.axes_marker_lift_m)
            if self.axes_marker_lift_m > 0.0
            else max(
                0.001,
                float(self.plane_marker_thickness_m) + float(self.axes_marker_radius_m),
            )
        )
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
            self._apply_marker_lifetime(marker)
            p0 = Point()
            p0.x = 0.0
            p0.y = 0.0
            p0.z = axes_lift_m
            p1 = Point()
            p1.x = float(end[0])
            p1.y = float(end[1])
            p1.z = float(end[2]) + axes_lift_m
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
        if not self.publish_object_marker:
            return
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
        if not self.publish_object_marker:
            self._last_marker_ids.clear()
            return
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

    def _publish_robot_pose(self, stamp) -> None:
        robot_x_w, robot_y_w, robot_z_w, robot_yaw_deg = self._compute_robot_pose_workspace()
        yaw_rad = math.radians(float(robot_yaw_deg))
        half_yaw = 0.5 * yaw_rad
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        pose = PoseStamped()
        pose.header.frame_id = self.workspace_frame
        pose.header.stamp = stamp
        pose.pose.position.x = float(robot_x_w)
        pose.pose.position.y = float(robot_y_w)
        pose.pose.position.z = float(robot_z_w)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        self.robot_pose_pub.publish(pose)

        if self.publish_robot_tf:
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = self.workspace_frame
            tf_msg.header.stamp = stamp
            tf_msg.child_frame_id = self.robot_tf_child_frame
            tf_msg.transform.translation.x = float(robot_x_w)
            tf_msg.transform.translation.y = float(robot_y_w)
            tf_msg.transform.translation.z = float(robot_z_w)
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = float(qz)
            tf_msg.transform.rotation.w = float(qw)
            self.tf_broadcaster.sendTransform(tf_msg)

        if not self.publish_robot_pose_marker:
            return

        marker = Marker()
        marker.header.frame_id = self.workspace_frame
        marker.header.stamp = stamp
        marker.ns = "holoassist_robot_pose"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(robot_x_w)
        marker.pose.position.y = float(robot_y_w)
        marker.pose.position.z = float(robot_z_w + 0.5 * self.robot_marker_height_m)
        marker.pose.orientation.z = float(qz)
        marker.pose.orientation.w = float(qw)
        marker.scale.x = float(self.robot_base_diameter_m)
        marker.scale.y = float(self.robot_base_diameter_m)
        marker.scale.z = float(self.robot_marker_height_m)
        marker.color.r = 0.95
        marker.color.g = 0.65
        marker.color.b = 0.15
        marker.color.a = 0.85
        self._apply_marker_lifetime(marker)
        self.robot_marker_pub.publish(marker)

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
            KeyValue(key="resnap_count", value=str(int(self._resnap_count))),
            KeyValue(
                key="resnap_interval_s",
                value=str(float(self.resnap_interval_s)),
            ),
            KeyValue(
                key="frames_since_resnap",
                value=str(int(max(0, self._frame_count - self._last_resnap_frame))),
            ),
            KeyValue(key="last_resnap_reason", value=self._last_resnap_reason),
            KeyValue(key="workspace_locked", value=str(bool(self._workspace_locked))),
            KeyValue(
                key="manual_realign_requested",
                value=str(bool(self._manual_realign_requested)),
            ),
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
