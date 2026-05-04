#!/usr/bin/env python3

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

try:
    from apriltag_msgs.msg import AprilTagDetectionArray

    HAVE_APRILTAG_MSGS = True
except Exception:
    AprilTagDetectionArray = None
    HAVE_APRILTAG_MSGS = False

SENTINEL_BBOX = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 0.0]


class DepthTrackerNode(Node):
    """Depth tracker with stable PointCloud2 output + blob-derived obstacle marker."""

    def __init__(self) -> None:
        super().__init__("holo_assist_depth_tracker")

        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/depth/camera_info")
        self.declare_parameter("min_depth_m", 2.0)
        self.declare_parameter("max_depth_m", 5.0)
        self.declare_parameter("min_area_px", 400)
        self.declare_parameter("morph_kernel", 5)
        self.declare_parameter("publish_pointcloud", True)
        self.declare_parameter(
            "pointcloud_topic", "/holo_assist_depth_tracker/pointcloud"
        )
        self.declare_parameter("point_stride_px", 4)
        self.declare_parameter("temporal_alpha", 0.35)
        self.declare_parameter("max_points", 15000)
        self.declare_parameter("publish_obstacle_marker", True)
        self.declare_parameter(
            "obstacle_topic", "/holo_assist_depth_tracker/obstacle_marker"
        )
        self.declare_parameter("obstacle_padding_m", 0.05)
        self.declare_parameter("obstacle_min_size_m", 0.08)
        self.declare_parameter("use_rgb_debug_image", True)
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("rgb_topic_fallback", "/camera/color/image_raw")
        self.declare_parameter("rgb_camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("rgb_timeout_s", 0.6)
        self.declare_parameter("object_pose_topic", "/holoassist/perception/object_pose")
        self.declare_parameter("object_pose_timeout_s", 0.8)
        self.declare_parameter("centroid_update_closer_only", True)
        self.declare_parameter("centroid_min_closer_m", 0.03)
        self.declare_parameter("centroid_smoothing_alpha", 0.25)
        self.declare_parameter("centroid_force_update_px", 80.0)
        self.declare_parameter("centroid_reset_after_missed_frames", 8)
        self.declare_parameter("max_debug_boxes", 6)
        self.declare_parameter("max_component_area_ratio", 0.30)
        self.declare_parameter("enable_apriltag_overlay", True)
        self.declare_parameter("apriltag_topic", "/detections_all")
        self.declare_parameter("apriltag_topic_fallback", "/detections")
        self.declare_parameter("apriltag_timeout_s", 1.0)
        self.declare_parameter("apriltag_distance_tracking_only", True)
        self.declare_parameter(
            "apriltag_track_tag_ids",
            [
                10,
                11,
                12,
                13,
                14,
                15,
                16,
                17,
                18,
                19,
                20,
                21,
                22,
                23,
                24,
                25,
                26,
                27,
                28,
                29,
                30,
                31,
                32,
                33,
            ],
        )
        self.declare_parameter("apriltag_bbox_padding_px", 8)
        self.declare_parameter("enable_workspace_overlay", True)
        self.declare_parameter("workspace_frame", "workspace_frame")
        self.declare_parameter("workspace_roi_x_min", -0.35)
        self.declare_parameter("workspace_roi_x_max", 0.35)
        self.declare_parameter("workspace_roi_y_min", -0.25)
        self.declare_parameter("workspace_roi_y_max", 0.25)
        self.declare_parameter("workspace_left_tag_id", 0)
        self.declare_parameter("workspace_right_tag_id", 1)
        self.declare_parameter("workspace_rear_left_tag_id", 2)
        self.declare_parameter("workspace_rear_right_tag_id", 3)
        self.declare_parameter("workspace_width_m", 0.70)
        self.declare_parameter("workspace_depth_m", 0.50)
        self.declare_parameter("workspace_tag_size_m", 0.032)
        self.declare_parameter("workspace_tag_center_edge_offset_m", 0.016)
        self.declare_parameter("workspace_left_corner_index_offset", 0)
        self.declare_parameter("workspace_right_corner_index_offset", 0)
        self.declare_parameter("workspace_left_corner_reverse", False)
        self.declare_parameter("workspace_right_corner_reverse", False)
        self.declare_parameter("workspace_outline_max_reproj_error_px", 24.0)

        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_area_px = int(self.get_parameter("min_area_px").value)
        self.morph_kernel = int(self.get_parameter("morph_kernel").value)
        self.publish_pointcloud = bool(self.get_parameter("publish_pointcloud").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.point_stride_px = int(self.get_parameter("point_stride_px").value)
        self.temporal_alpha = float(self.get_parameter("temporal_alpha").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.publish_obstacle_marker = bool(
            self.get_parameter("publish_obstacle_marker").value
        )
        self.obstacle_topic = str(self.get_parameter("obstacle_topic").value)
        self.obstacle_padding_m = float(self.get_parameter("obstacle_padding_m").value)
        self.obstacle_min_size_m = float(
            self.get_parameter("obstacle_min_size_m").value
        )
        self.use_rgb_debug_image = bool(
            self.get_parameter("use_rgb_debug_image").value
        )
        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        self.rgb_topic_fallback = str(
            self.get_parameter("rgb_topic_fallback").value
        )
        self.rgb_camera_info_topic = str(
            self.get_parameter("rgb_camera_info_topic").value
        )
        self.rgb_timeout_s = float(self.get_parameter("rgb_timeout_s").value)
        self.object_pose_topic = str(self.get_parameter("object_pose_topic").value)
        self.object_pose_timeout_s = float(
            self.get_parameter("object_pose_timeout_s").value
        )
        self.centroid_update_closer_only = bool(
            self.get_parameter("centroid_update_closer_only").value
        )
        self.centroid_min_closer_m = float(
            self.get_parameter("centroid_min_closer_m").value
        )
        self.centroid_smoothing_alpha = float(
            self.get_parameter("centroid_smoothing_alpha").value
        )
        self.centroid_force_update_px = float(
            self.get_parameter("centroid_force_update_px").value
        )
        self.centroid_reset_after_missed_frames = int(
            self.get_parameter("centroid_reset_after_missed_frames").value
        )
        self.max_debug_boxes = int(self.get_parameter("max_debug_boxes").value)
        self.max_component_area_ratio = float(
            self.get_parameter("max_component_area_ratio").value
        )
        self.enable_apriltag_overlay = bool(
            self.get_parameter("enable_apriltag_overlay").value
        )
        self.apriltag_topic = str(self.get_parameter("apriltag_topic").value)
        self.apriltag_topic_fallback = str(
            self.get_parameter("apriltag_topic_fallback").value
        ).strip()
        self.apriltag_timeout_s = float(
            self.get_parameter("apriltag_timeout_s").value
        )
        self.apriltag_distance_tracking_only = bool(
            self.get_parameter("apriltag_distance_tracking_only").value
        )
        self.apriltag_track_tag_ids = [
            int(v) for v in self.get_parameter("apriltag_track_tag_ids").value
        ]
        self.apriltag_bbox_padding_px = int(
            self.get_parameter("apriltag_bbox_padding_px").value
        )
        self.enable_workspace_overlay = bool(
            self.get_parameter("enable_workspace_overlay").value
        )
        self.workspace_frame = str(self.get_parameter("workspace_frame").value)
        self.workspace_roi_x_min = float(
            self.get_parameter("workspace_roi_x_min").value
        )
        self.workspace_roi_x_max = float(
            self.get_parameter("workspace_roi_x_max").value
        )
        self.workspace_roi_y_min = float(
            self.get_parameter("workspace_roi_y_min").value
        )
        self.workspace_roi_y_max = float(
            self.get_parameter("workspace_roi_y_max").value
        )
        self.workspace_left_tag_id = int(
            self.get_parameter("workspace_left_tag_id").value
        )
        self.workspace_right_tag_id = int(
            self.get_parameter("workspace_right_tag_id").value
        )
        self.workspace_rear_left_tag_id = int(
            self.get_parameter("workspace_rear_left_tag_id").value
        )
        self.workspace_rear_right_tag_id = int(
            self.get_parameter("workspace_rear_right_tag_id").value
        )
        self.workspace_width_m = float(self.get_parameter("workspace_width_m").value)
        self.workspace_depth_m = float(self.get_parameter("workspace_depth_m").value)
        self.workspace_tag_size_m = float(
            self.get_parameter("workspace_tag_size_m").value
        )
        self.workspace_tag_center_edge_offset_m = float(
            self.get_parameter("workspace_tag_center_edge_offset_m").value
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

        if self.max_depth_m <= self.min_depth_m:
            self.get_logger().warn(
                "max_depth_m must be greater than min_depth_m. Using defaults 2.0 to 5.0 m."
            )
            self.min_depth_m = 2.0
            self.max_depth_m = 5.0

        if self.morph_kernel < 1:
            self.get_logger().warn("morph_kernel must be >= 1. Using morph_kernel=1.")
            self.morph_kernel = 1
        if self.morph_kernel % 2 == 0:
            self.get_logger().warn(
                f"morph_kernel should be odd for centered morphology. Using {self.morph_kernel + 1}."
            )
            self.morph_kernel += 1
        if self.point_stride_px < 1:
            self.get_logger().warn("point_stride_px must be >= 1. Using point_stride_px=1.")
            self.point_stride_px = 1
        if not 0.0 < self.temporal_alpha <= 1.0:
            self.get_logger().warn(
                "temporal_alpha must be in (0, 1]. Using temporal_alpha=0.35."
            )
            self.temporal_alpha = 0.35
        if self.max_points < 1:
            self.get_logger().warn("max_points must be >= 1. Using max_points=15000.")
            self.max_points = 15000
        if self.obstacle_padding_m < 0.0:
            self.get_logger().warn(
                "obstacle_padding_m must be >= 0. Using obstacle_padding_m=0.0."
            )
            self.obstacle_padding_m = 0.0
        if self.obstacle_min_size_m <= 0.0:
            self.get_logger().warn(
                "obstacle_min_size_m must be > 0. Using obstacle_min_size_m=0.08."
            )
            self.obstacle_min_size_m = 0.08
        if self.rgb_timeout_s <= 0.0:
            self.get_logger().warn("rgb_timeout_s must be > 0. Using rgb_timeout_s=0.6.")
            self.rgb_timeout_s = 0.6
        if self.object_pose_timeout_s <= 0.0:
            self.get_logger().warn(
                "object_pose_timeout_s must be > 0. Using object_pose_timeout_s=0.8."
            )
            self.object_pose_timeout_s = 0.8
        if self.centroid_min_closer_m < 0.0:
            self.get_logger().warn(
                "centroid_min_closer_m must be >= 0. Using centroid_min_closer_m=0.03."
            )
            self.centroid_min_closer_m = 0.03
        if self.centroid_force_update_px < 0.0:
            self.get_logger().warn(
                "centroid_force_update_px must be >= 0. Using centroid_force_update_px=80."
            )
            self.centroid_force_update_px = 80.0
        if not 0.0 < self.centroid_smoothing_alpha <= 1.0:
            self.get_logger().warn(
                "centroid_smoothing_alpha must be in (0,1]. Using centroid_smoothing_alpha=0.25."
            )
            self.centroid_smoothing_alpha = 0.25
        if self.centroid_reset_after_missed_frames < 1:
            self.get_logger().warn(
                "centroid_reset_after_missed_frames must be >= 1. Using 8."
            )
            self.centroid_reset_after_missed_frames = 8
        if self.max_debug_boxes < 1:
            self.get_logger().warn("max_debug_boxes must be >= 1. Using 6.")
            self.max_debug_boxes = 6
        if self.apriltag_timeout_s <= 0.0:
            self.get_logger().warn(
                "apriltag_timeout_s must be > 0. Using apriltag_timeout_s=1.0."
            )
            self.apriltag_timeout_s = 1.0
        if self.apriltag_bbox_padding_px < 0:
            self.get_logger().warn(
                "apriltag_bbox_padding_px must be >= 0. Using apriltag_bbox_padding_px=0."
            )
            self.apriltag_bbox_padding_px = 0
        if self.workspace_roi_x_max <= self.workspace_roi_x_min:
            self.get_logger().warn(
                "workspace_roi_x_max must be > workspace_roi_x_min. Using [-0.35, 0.35]."
            )
            self.workspace_roi_x_min = -0.35
            self.workspace_roi_x_max = 0.35
        if self.workspace_roi_y_max <= self.workspace_roi_y_min:
            self.get_logger().warn(
                "workspace_roi_y_max must be > workspace_roi_y_min. Using [-0.25, 0.25]."
            )
            self.workspace_roi_y_min = -0.25
            self.workspace_roi_y_max = 0.25
        self.workspace_width_m = max(0.10, self.workspace_width_m)
        self.workspace_depth_m = max(0.10, self.workspace_depth_m)
        self.workspace_tag_size_m = max(0.01, self.workspace_tag_size_m)
        self.workspace_tag_center_edge_offset_m = max(
            0.0, self.workspace_tag_center_edge_offset_m
        )
        self._warn_expected_apriltag_size(
            name="workspace_tag_size_m",
            value_m=self.workspace_tag_size_m,
        )
        self.workspace_outline_max_reproj_error_px = max(
            1.0, self.workspace_outline_max_reproj_error_px
        )
        self.max_component_area_ratio = min(
            max(self.max_component_area_ratio, 0.02), 1.0
        )

        self.bridge = CvBridge()
        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_rgb_camera_info: Optional[CameraInfo] = None
        self.latest_rgb_bgr: Optional[np.ndarray] = None
        self.latest_rgb_stamp: Optional[rclpy.time.Time] = None
        self.latest_rgb_source_topic = ""
        self.latest_object_pose: Optional[PoseStamped] = None
        self.latest_object_pose_stamp: Optional[rclpy.time.Time] = None
        self.latest_apriltag_detections = []
        self.latest_apriltag_stamp: Optional[rclpy.time.Time] = None
        self.latest_apriltag_image_size: Optional[tuple[int, int]] = None
        self.apriltag_track_tag_ids_set = set(self.apriltag_track_tag_ids)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_debug_image_is_rgb = False
        self.depth_ema: Optional[np.ndarray] = None
        self.frame_count = 0
        self.published_bbox_count = 0
        self.published_pointcloud_count = 0
        self.last_point_count = 0
        self.obstacle_active = False
        self.last_depth_rx_time: Optional[rclpy.time.Time] = None
        self._stable_bbox_cx: Optional[float] = None
        self._stable_bbox_cy: Optional[float] = None
        self._stable_bbox_w: Optional[float] = None
        self._stable_bbox_h: Optional[float] = None
        self._stable_bbox_depth_m: Optional[float] = None
        self._stable_bbox_miss_count = 0

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            lambda msg: self.rgb_callback(msg, self.rgb_topic),
            qos_profile_sensor_data,
        )
        self.rgb_fallback_sub = None
        if self.rgb_topic_fallback and self.rgb_topic_fallback != self.rgb_topic:
            self.rgb_fallback_sub = self.create_subscription(
                Image,
                self.rgb_topic_fallback,
                lambda msg: self.rgb_callback(msg, self.rgb_topic_fallback),
                qos_profile_sensor_data,
            )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.rgb_camera_info_sub = self.create_subscription(
            CameraInfo,
            self.rgb_camera_info_topic,
            self.rgb_camera_info_callback,
            qos_profile_sensor_data,
        )
        self.object_pose_sub = self.create_subscription(
            PoseStamped,
            self.object_pose_topic,
            self.object_pose_callback,
            qos_profile_sensor_data,
        )
        self.apriltag_sub = None
        self.apriltag_fallback_sub = None
        if self.enable_apriltag_overlay:
            if HAVE_APRILTAG_MSGS and AprilTagDetectionArray is not None:
                self.apriltag_sub = self.create_subscription(
                    AprilTagDetectionArray,
                    self.apriltag_topic,
                    self.apriltag_callback,
                    10,
                )
                if (
                    self.apriltag_topic_fallback
                    and self.apriltag_topic_fallback != self.apriltag_topic
                ):
                    self.apriltag_fallback_sub = self.create_subscription(
                        AprilTagDetectionArray,
                        self.apriltag_topic_fallback,
                        self.apriltag_callback,
                        10,
                    )
            else:
                self.get_logger().warn(
                    "enable_apriltag_overlay=true but apriltag_msgs is unavailable. "
                    "Install apriltag_ros/apriltag_msgs or disable overlay."
                )

        reliable_pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            "/holo_assist_depth_tracker/debug_image",
            reliable_pub_qos,
        )
        self.bbox_pub = self.create_publisher(
            Float32MultiArray,
            "/holo_assist_depth_tracker/bbox",
            10,
        )
        self.pointcloud_pub = None
        if self.publish_pointcloud:
            self.pointcloud_pub = self.create_publisher(
                PointCloud2,
                self.pointcloud_topic,
                reliable_pub_qos,
            )

        self.obstacle_marker_pub = None
        if self.publish_obstacle_marker:
            self.obstacle_marker_pub = self.create_publisher(
                Marker,
                self.obstacle_topic,
                reliable_pub_qos,
            )

        self.point_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        self.status_timer = self.create_timer(2.0, self._status_timer_cb)

        self.get_logger().info(
            "Depth tracker started. bbox payload: [x_min, y_min, x_max, y_max, cx, cy, median_depth_m, area_px]"
        )
        self.get_logger().info(
            f"Subscribing depth={self.depth_topic}, camera_info={self.camera_info_topic}, "
            f"band=[{self.min_depth_m:.2f}, {self.max_depth_m:.2f}] m, "
            f"min_area_px={self.min_area_px}, morph_kernel={self.morph_kernel}, "
            f"pointcloud={'on' if self.publish_pointcloud else 'off'}"
        )
        if self.publish_pointcloud:
            self.get_logger().info(
                f"PointCloud2 topic={self.pointcloud_topic}, stride={self.point_stride_px}, "
                f"temporal_alpha={self.temporal_alpha:.2f}, max_points={self.max_points}"
            )
        if self.publish_obstacle_marker:
            self.get_logger().info(
                f"Obstacle marker topic={self.obstacle_topic}, padding={self.obstacle_padding_m:.3f} m, "
                f"min_size={self.obstacle_min_size_m:.3f} m"
            )
        self.get_logger().info(
            f"RGB debug={'on' if self.use_rgb_debug_image else 'off'} "
            f"rgb_topic={self.rgb_topic} rgb_topic_fallback={self.rgb_topic_fallback} "
            f"object_pose_topic={self.object_pose_topic}"
        )
        self.get_logger().info(
            f"AprilTag overlay={'on' if self.enable_apriltag_overlay else 'off'} "
            f"topic={self.apriltag_topic} fallback={self.apriltag_topic_fallback or '<none>'} "
            f"timeout={self.apriltag_timeout_s:.2f}s"
        )
        self.get_logger().info(
            "AprilTag distance tracking only=%s ids=%s bbox_padding_px=%d"
            % (
                self.apriltag_distance_tracking_only,
                sorted(self.apriltag_track_tag_ids_set),
                self.apriltag_bbox_padding_px,
            )
        )
        self.get_logger().info(
            "Workspace overlay board=(%.3fx%.3fm) tag_size=%.3fm tag_center_edge_offset=%.3fm"
            % (
                self.workspace_width_m,
                self.workspace_depth_m,
                self.workspace_tag_size_m,
                self.workspace_tag_center_edge_offset_m,
            )
        )
        self.get_logger().info(
            f"Centroid stabilization closer_only={self.centroid_update_closer_only} "
            f"min_closer={self.centroid_min_closer_m:.3f}m alpha={self.centroid_smoothing_alpha:.2f} "
            f"force_px={self.centroid_force_update_px:.1f}"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def rgb_camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_rgb_camera_info = msg

    def rgb_callback(self, msg: Image, source_topic: str = "") -> None:
        if not self.use_rgb_debug_image:
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            if self.frame_count % 60 == 0:
                self.get_logger().warn(f"RGB conversion failed: {exc}")
            return

        self.latest_rgb_bgr = bgr
        self.latest_rgb_source_topic = source_topic
        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds == 0:
                stamp = self.get_clock().now()
        except Exception:
            stamp = self.get_clock().now()
        self.latest_rgb_stamp = stamp

    def object_pose_callback(self, msg: PoseStamped) -> None:
        self.latest_object_pose = msg
        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds == 0:
                stamp = self.get_clock().now()
        except Exception:
            stamp = self.get_clock().now()
        self.latest_object_pose_stamp = stamp

    def apriltag_callback(self, msg) -> None:
        self.latest_apriltag_detections = list(msg.detections)
        try:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds == 0:
                stamp = self.get_clock().now()
        except Exception:
            stamp = self.get_clock().now()
        self.latest_apriltag_stamp = stamp

        if self.latest_rgb_camera_info is not None:
            w = int(self.latest_rgb_camera_info.width)
            h = int(self.latest_rgb_camera_info.height)
            if w > 0 and h > 0:
                self.latest_apriltag_image_size = (w, h)
                return
        if self.latest_rgb_bgr is not None:
            h, w = self.latest_rgb_bgr.shape[:2]
            self.latest_apriltag_image_size = (int(w), int(h))

    def _warn_expected_apriltag_size(self, name: str, value_m: float) -> None:
        expected = 0.032
        if abs(value_m - expected) > 1e-6:
            self.get_logger().warn(
                "%s=%.6f m but all current AprilTags are expected to be %.3f m."
                % (name, value_m, expected)
            )
        if value_m > 0.05:
            self.get_logger().warn(
                "%s=%.6f m is unusually large; AprilTag pose/overlay depth may be wrong."
                % (name, value_m)
            )
        if value_m >= 1.0:
            self.get_logger().warn(
                "%s=%.6f looks like millimeters passed as meters (e.g. 32/40)."
                % (name, value_m)
            )

    def depth_callback(self, msg: Image) -> None:
        self.frame_count += 1
        self.last_depth_rx_time = self.get_clock().now()
        depth_m = self._depth_to_meters(msg)
        if depth_m is None:
            self._publish_bbox(SENTINEL_BBOX)
            self.last_point_count = 0
            self._publish_obstacle_delete(msg.header.frame_id)
            return

        smoothed_depth_m = self._apply_temporal_filter(depth_m)
        valid_mask = np.isfinite(smoothed_depth_m) & (smoothed_depth_m > 0.0)
        band_mask = (
            valid_mask
            & (smoothed_depth_m >= self.min_depth_m)
            & (smoothed_depth_m <= self.max_depth_m)
        )
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"frame={self.frame_count} encoding={msg.encoding} "
                f"valid_px={int(np.count_nonzero(valid_mask))} "
                f"in_band_px={int(np.count_nonzero(band_mask))}"
            )

        if self.publish_pointcloud:
            self.last_point_count = self._publish_pointcloud(msg, band_mask, smoothed_depth_m)

        debug_bgr = self._create_debug_image(smoothed_depth_m)
        if self.use_rgb_debug_image and not self._last_debug_image_is_rgb:
            cv2.putText(
                debug_bgr,
                f"RGB missing: waiting on {self.rgb_topic} / {self.rgb_topic_fallback}",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        bbox_msg = Float32MultiArray()
        bbox_msg.data = list(SENTINEL_BBOX)
        if self.apriltag_distance_tracking_only:
            bbox_msg.data = self._estimate_primary_apriltag_bbox(smoothed_depth_m)
            self._publish_obstacle_delete(msg.header.frame_id)
            if bbox_msg.data[0] >= 0.0:
                sx, sy, x_max, y_max, cx, cy, sdepth, area_px = bbox_msg.data
                x0 = int(round(sx))
                y0 = int(round(sy))
                x1 = int(round(x_max))
                y1 = int(round(y_max))
                cv2.rectangle(debug_bgr, (x0, y0), (x1, y1), (255, 255, 255), 2)
                label = f"apriltag dist={sdepth:.2f}m area={int(area_px)}"
                cv2.putText(
                    debug_bgr,
                    label,
                    (x0, min(debug_bgr.shape[0] - 26, y1 + 16)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                pose_text = self._format_pose_text()
                if pose_text is not None:
                    cv2.putText(
                        debug_bgr,
                        pose_text,
                        (x0, min(debug_bgr.shape[0] - 8, y1 + 36)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.48,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
            else:
                cv2.putText(
                    debug_bgr,
                    "No tracked AprilTag target in depth",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
        else:
            binary_mask = (band_mask.astype(np.uint8) * 255)
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (self.morph_kernel, self.morph_kernel),
            )
            cleaned = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)
            cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
                (cleaned > 0).astype(np.uint8),
                connectivity=8,
            )
            image_area = float(smoothed_depth_m.shape[0] * smoothed_depth_m.shape[1])
            max_component_area_px = max(
                self.min_area_px,
                int(self.max_component_area_ratio * image_area),
            )
            components = []
            filtered_large_count = 0
            for label in range(1, num_labels):
                area_px = int(stats[label, cv2.CC_STAT_AREA])
                if area_px < self.min_area_px:
                    continue
                if area_px > max_component_area_px:
                    filtered_large_count += 1
                    continue

                x = int(stats[label, cv2.CC_STAT_LEFT])
                y = int(stats[label, cv2.CC_STAT_TOP])
                w = int(stats[label, cv2.CC_STAT_WIDTH])
                h = int(stats[label, cv2.CC_STAT_HEIGHT])
                cx, cy = centroids[label]

                blob_mask = labels == label
                blob_depths = smoothed_depth_m[blob_mask]
                blob_depths = blob_depths[np.isfinite(blob_depths) & (blob_depths > 0.0)]
                if blob_depths.size == 0:
                    continue
                median_depth_m = float(np.median(blob_depths))

                components.append(
                    {
                        "label": int(label),
                        "x": x,
                        "y": y,
                        "w": w,
                        "h": h,
                        "cx": float(cx),
                        "cy": float(cy),
                        "area_px": int(area_px),
                        "median_depth_m": float(median_depth_m),
                    }
                )

            components_by_area = sorted(
                components, key=lambda c: int(c["area_px"]), reverse=True
            )
            for idx, component in enumerate(components_by_area[: self.max_debug_boxes]):
                color = self._debug_color(idx)
                x = int(component["x"])
                y = int(component["y"])
                w = int(component["w"])
                h = int(component["h"])
                cx = float(component["cx"])
                cy = float(component["cy"])
                depth = float(component["median_depth_m"])
                area_px = int(component["area_px"])

                cv2.rectangle(debug_bgr, (x, y), (x + w, y + h), color, 2)
                cv2.circle(debug_bgr, (int(round(cx)), int(round(cy))), 3, color, -1)
                label_text = f"#{idx+1} d={depth:.2f}m a={area_px}"
                cv2.putText(
                    debug_bgr,
                    label_text,
                    (x, max(0, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    color,
                    2,
                    cv2.LINE_AA,
                )

            if components:
                primary_component = min(
                    components, key=lambda c: float(c["median_depth_m"])
                )
                x = int(primary_component["x"])
                y = int(primary_component["y"])
                w = int(primary_component["w"])
                h = int(primary_component["h"])
                area_px = int(primary_component["area_px"])
                cx = float(primary_component["cx"])
                cy = float(primary_component["cy"])
                median_depth_m = float(primary_component["median_depth_m"])

                sx, sy, sw, sh, sdepth = self._stabilize_blob_estimate(
                    x=x,
                    y=y,
                    w=w,
                    h=h,
                    cx=cx,
                    cy=cy,
                    median_depth_m=median_depth_m,
                )

                x0 = int(round(sx))
                y0 = int(round(sy))
                x1 = int(round(sx + sw))
                y1 = int(round(sy + sh))
                cv2.rectangle(debug_bgr, (x0, y0), (x1, y1), (255, 255, 255), 2)

                pose_text = self._format_pose_text()
                label = f"primary dist={sdepth:.2f}m area={area_px}"
                cv2.putText(
                    debug_bgr,
                    label,
                    (x0, min(debug_bgr.shape[0] - 26, y1 + 16)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                if pose_text is not None:
                    cv2.putText(
                        debug_bgr,
                        pose_text,
                        (x0, min(debug_bgr.shape[0] - 8, y1 + 36)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.48,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

                bbox_msg.data = [
                    float(sx),
                    float(sy),
                    float(sx + sw),
                    float(sy + sh),
                    float(sx + 0.5 * sw),
                    float(sy + 0.5 * sh),
                    float(sdepth),
                    float(area_px),
                ]
                self._publish_obstacle_from_mask(
                    msg,
                    labels == int(primary_component["label"]),
                )
            else:
                self._on_blob_missed()
                self._publish_obstacle_delete(msg.header.frame_id)
                no_blob_msg = "No valid components in depth band"
                if filtered_large_count > 0:
                    no_blob_msg = (
                        "Components too large filtered; lower depth band or "
                        "increase max_component_area_ratio"
                    )
                cv2.putText(
                    debug_bgr,
                    no_blob_msg,
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

        if self.latest_camera_info is None:
            if self.frame_count % 60 == 0:
                self.get_logger().warn(
                    "camera_info not received yet: pointcloud and obstacle marker are waiting for intrinsics."
                )
            cv2.putText(
                debug_bgr,
                "camera_info: waiting...",
                (12, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        self._draw_apriltag_overlay(debug_bgr)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_bgr, encoding="bgr8")
        debug_msg.header = msg.header

        self.debug_image_pub.publish(debug_msg)
        self._publish_bbox(bbox_msg.data)

    def _stabilize_blob_estimate(
        self,
        x: int,
        y: int,
        w: int,
        h: int,
        cx: float,
        cy: float,
        median_depth_m: float,
    ) -> tuple[float, float, float, float, float]:
        cx_raw = float(cx)
        cy_raw = float(cy)
        w_raw = float(max(1, w))
        h_raw = float(max(1, h))
        depth_raw = float(median_depth_m)

        if self._stable_bbox_cx is None:
            self._stable_bbox_cx = cx_raw
            self._stable_bbox_cy = cy_raw
            self._stable_bbox_w = w_raw
            self._stable_bbox_h = h_raw
            self._stable_bbox_depth_m = depth_raw
            self._stable_bbox_miss_count = 0
            return (
                float(x),
                float(y),
                w_raw,
                h_raw,
                depth_raw,
            )

        assert self._stable_bbox_cy is not None
        assert self._stable_bbox_w is not None
        assert self._stable_bbox_h is not None
        assert self._stable_bbox_depth_m is not None

        accept_update = True
        if self.centroid_update_closer_only:
            accept_update = False
            if depth_raw > 0.0 and self._stable_bbox_depth_m > 0.0:
                if depth_raw <= (self._stable_bbox_depth_m - self.centroid_min_closer_m):
                    accept_update = True
            dx = abs(cx_raw - self._stable_bbox_cx)
            dy = abs(cy_raw - self._stable_bbox_cy)
            if dx >= self.centroid_force_update_px or dy >= self.centroid_force_update_px:
                accept_update = True

        if accept_update:
            alpha = self.centroid_smoothing_alpha
            self._stable_bbox_cx = (1.0 - alpha) * self._stable_bbox_cx + alpha * cx_raw
            self._stable_bbox_cy = (1.0 - alpha) * self._stable_bbox_cy + alpha * cy_raw
            self._stable_bbox_w = (1.0 - alpha) * self._stable_bbox_w + alpha * w_raw
            self._stable_bbox_h = (1.0 - alpha) * self._stable_bbox_h + alpha * h_raw
            if depth_raw > 0.0:
                if self._stable_bbox_depth_m > 0.0:
                    self._stable_bbox_depth_m = (
                        (1.0 - alpha) * self._stable_bbox_depth_m + alpha * depth_raw
                    )
                else:
                    self._stable_bbox_depth_m = depth_raw

        self._stable_bbox_miss_count = 0
        x_stable = self._stable_bbox_cx - 0.5 * self._stable_bbox_w
        y_stable = self._stable_bbox_cy - 0.5 * self._stable_bbox_h
        return (
            float(x_stable),
            float(y_stable),
            float(self._stable_bbox_w),
            float(self._stable_bbox_h),
            float(self._stable_bbox_depth_m),
        )

    def _on_blob_missed(self) -> None:
        self._stable_bbox_miss_count += 1
        if self._stable_bbox_miss_count < self.centroid_reset_after_missed_frames:
            return
        self._stable_bbox_cx = None
        self._stable_bbox_cy = None
        self._stable_bbox_w = None
        self._stable_bbox_h = None
        self._stable_bbox_depth_m = None
        self._stable_bbox_miss_count = 0

    def _format_pose_text(self) -> Optional[str]:
        if self.latest_object_pose is None or self.latest_object_pose_stamp is None:
            return None
        age_s = (self.get_clock().now() - self.latest_object_pose_stamp).nanoseconds / 1e9
        if age_s > self.object_pose_timeout_s:
            return None
        pose = self.latest_object_pose.pose.position
        return f"pose[{self.latest_object_pose.header.frame_id}]: x={pose.x:.2f} y={pose.y:.2f} z={pose.z:.2f}"

    def _debug_color(self, idx: int) -> tuple[int, int, int]:
        palette = [
            (0, 255, 0),
            (255, 200, 0),
            (255, 0, 255),
            (0, 220, 255),
            (255, 120, 0),
            (120, 255, 120),
        ]
        return palette[idx % len(palette)]

    def _publish_bbox(self, data: list[float]) -> None:
        bbox_msg = Float32MultiArray()
        bbox_msg.data = data
        self.bbox_pub.publish(bbox_msg)
        self.published_bbox_count += 1

    def _fresh_apriltag_detections(self) -> list:
        if self.latest_apriltag_stamp is None:
            return []
        age_s = (self.get_clock().now() - self.latest_apriltag_stamp).nanoseconds / 1e9
        if age_s > self.apriltag_timeout_s:
            return []
        return list(self.latest_apriltag_detections)

    def _estimate_primary_apriltag_bbox(self, depth_m: np.ndarray) -> list[float]:
        detections = self._fresh_apriltag_detections()
        if not detections:
            self._on_blob_missed()
            return list(SENTINEL_BBOX)

        out_h, out_w = depth_m.shape[:2]
        src_w, src_h = out_w, out_h
        if self.latest_apriltag_image_size is not None:
            src_w = max(1, int(self.latest_apriltag_image_size[0]))
            src_h = max(1, int(self.latest_apriltag_image_size[1]))
        sx = float(out_w) / float(src_w)
        sy = float(out_h) / float(src_h)

        candidates = []
        finite_depth = np.isfinite(depth_m) & (depth_m > 0.0)
        for det in detections:
            tag_id = int(det.id)
            if self.apriltag_track_tag_ids_set and tag_id not in self.apriltag_track_tag_ids_set:
                continue
            if len(det.corners) < 4:
                continue

            corners = np.array(
                [[float(p.x) * sx, float(p.y) * sy] for p in det.corners],
                dtype=np.float32,
            )
            if not np.isfinite(corners).all():
                continue

            min_x = int(np.floor(np.min(corners[:, 0]))) - self.apriltag_bbox_padding_px
            max_x = int(np.ceil(np.max(corners[:, 0]))) + self.apriltag_bbox_padding_px
            min_y = int(np.floor(np.min(corners[:, 1]))) - self.apriltag_bbox_padding_px
            max_y = int(np.ceil(np.max(corners[:, 1]))) + self.apriltag_bbox_padding_px

            min_x = max(0, min(out_w - 1, min_x))
            min_y = max(0, min(out_h - 1, min_y))
            max_x = max(0, min(out_w - 1, max_x))
            max_y = max(0, min(out_h - 1, max_y))
            if max_x <= min_x or max_y <= min_y:
                continue

            poly = np.round(corners).astype(np.int32)
            poly[:, 0] = np.clip(poly[:, 0], 0, out_w - 1)
            poly[:, 1] = np.clip(poly[:, 1], 0, out_h - 1)

            mask = np.zeros((out_h, out_w), dtype=np.uint8)
            cv2.fillConvexPoly(mask, poly, 255)
            valid_mask = (mask > 0) & finite_depth
            depth_samples = depth_m[valid_mask]
            if depth_samples.size == 0:
                roi = depth_m[min_y : max_y + 1, min_x : max_x + 1]
                roi_mask = finite_depth[min_y : max_y + 1, min_x : max_x + 1]
                depth_samples = roi[roi_mask]
            if depth_samples.size == 0:
                continue

            median_depth_m = float(np.median(depth_samples))
            w = float(max_x - min_x)
            h = float(max_y - min_y)
            cx = float(0.5 * (min_x + max_x))
            cy = float(0.5 * (min_y + max_y))
            area_px = int(max(1.0, w * h))
            candidates.append(
                {
                    "x": int(min_x),
                    "y": int(min_y),
                    "w": w,
                    "h": h,
                    "cx": cx,
                    "cy": cy,
                    "median_depth_m": median_depth_m,
                    "area_px": area_px,
                }
            )

        if not candidates:
            self._on_blob_missed()
            return list(SENTINEL_BBOX)

        primary = min(candidates, key=lambda c: float(c["median_depth_m"]))
        sx0, sy0, sw, sh, sdepth = self._stabilize_blob_estimate(
            x=int(primary["x"]),
            y=int(primary["y"]),
            w=int(max(1.0, float(primary["w"]))),
            h=int(max(1.0, float(primary["h"]))),
            cx=float(primary["cx"]),
            cy=float(primary["cy"]),
            median_depth_m=float(primary["median_depth_m"]),
        )

        return [
            float(sx0),
            float(sy0),
            float(sx0 + sw),
            float(sy0 + sh),
            float(sx0 + 0.5 * sw),
            float(sy0 + 0.5 * sh),
            float(sdepth),
            float(primary["area_px"]),
        ]

    def _status_timer_cb(self) -> None:
        if self.last_depth_rx_time is None:
            age_s = -1.0
        else:
            age_s = (self.get_clock().now() - self.last_depth_rx_time).nanoseconds / 1e9

        self.get_logger().info(
            f"status: depth_frames={self.frame_count}, bbox_published={self.published_bbox_count}, "
            f"pointcloud_published={self.published_pointcloud_count}, last_point_count={self.last_point_count}, "
            f"obstacle_active={self.obstacle_active}, last_depth_age_s={age_s:.2f}"
        )

        # If depth is not arriving, still publish sentinel bbox so CLI tools show activity.
        if self.last_depth_rx_time is None or age_s > 2.5:
            self._publish_bbox(list(SENTINEL_BBOX))
            self.last_point_count = 0
            self._publish_obstacle_delete("")

    def _depth_to_meters(self, msg: Image) -> Optional[np.ndarray]:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            self.get_logger().error(f"CvBridge conversion failed: {exc}")
            return None

        if msg.encoding == "16UC1":
            return depth.astype(np.float32) * 0.001

        if msg.encoding == "32FC1":
            return depth.astype(np.float32)

        self.get_logger().warn(
            f"Unsupported depth encoding '{msg.encoding}'. Expected 16UC1 or 32FC1."
        )
        return None

    def _apply_temporal_filter(self, depth_m: np.ndarray) -> np.ndarray:
        current = depth_m.astype(np.float32, copy=False)
        valid_current = np.isfinite(current) & (current > 0.0)

        if self.depth_ema is None or self.depth_ema.shape != current.shape:
            self.depth_ema = current.copy()
            self.depth_ema[~valid_current] = np.nan
            return self.depth_ema

        valid_prev = np.isfinite(self.depth_ema) & (self.depth_ema > 0.0)
        both_valid = valid_current & valid_prev
        only_current_valid = valid_current & ~valid_prev

        self.depth_ema[both_valid] = (
            self.temporal_alpha * current[both_valid]
            + (1.0 - self.temporal_alpha) * self.depth_ema[both_valid]
        )
        self.depth_ema[only_current_valid] = current[only_current_valid]
        self.depth_ema[~valid_current] = np.nan
        return self.depth_ema

    def _camera_intrinsics(self) -> Optional[tuple[float, float, float, float]]:
        if self.latest_camera_info is None:
            return None
        k = self.latest_camera_info.k
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])
        if fx <= 0.0 or fy <= 0.0:
            if self.frame_count % 60 == 0:
                self.get_logger().warn(
                    f"Invalid intrinsics fx={fx:.4f}, fy={fy:.4f}. Skipping pointcloud projection."
                )
            return None
        return fx, fy, cx, cy

    def _project_depth_to_points(
        self,
        depth_m: np.ndarray,
        mask: np.ndarray,
    ) -> Optional[np.ndarray]:
        intrinsics = self._camera_intrinsics()
        if intrinsics is None:
            return None
        fx, fy, cx, cy = intrinsics

        height, width = depth_m.shape
        rows = np.arange(0, height, self.point_stride_px, dtype=np.int32)
        cols = np.arange(0, width, self.point_stride_px, dtype=np.int32)
        vv, uu = np.meshgrid(rows, cols, indexing="ij")

        sampled_mask = mask[vv, uu]
        if not np.any(sampled_mask):
            return np.empty((0, 3), dtype=np.float32)

        z = depth_m[vv, uu][sampled_mask].astype(np.float32, copy=False)
        u = uu[sampled_mask].astype(np.float32, copy=False)
        v = vv[sampled_mask].astype(np.float32, copy=False)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points = np.column_stack((x, y, z)).astype(np.float32, copy=False)

        if points.shape[0] > self.max_points:
            idx = np.linspace(0, points.shape[0] - 1, self.max_points, dtype=np.int32)
            points = points[idx]

        return points

    def _publish_pointcloud(self, depth_msg: Image, mask: np.ndarray, depth_m: np.ndarray) -> int:
        if self.pointcloud_pub is None:
            return 0

        points = self._project_depth_to_points(depth_m, mask)
        if points is None:
            return 0

        cloud = PointCloud2()
        cloud.header = depth_msg.header
        cloud.height = 1
        cloud.width = int(points.shape[0])
        cloud.fields = self.point_fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False
        cloud.data = points.tobytes()
        self.pointcloud_pub.publish(cloud)
        self.published_pointcloud_count += 1
        return int(points.shape[0])

    def _publish_obstacle_from_mask(self, depth_msg: Image, blob_mask: np.ndarray) -> None:
        if self.obstacle_marker_pub is None:
            return

        depth_m = self.depth_ema
        if depth_m is None:
            self._publish_obstacle_delete(depth_msg.header.frame_id)
            return

        valid_blob_mask = blob_mask & np.isfinite(depth_m) & (depth_m > 0.0)
        points = self._project_depth_to_points(depth_m, valid_blob_mask)
        if points is None or points.shape[0] == 0:
            self._publish_obstacle_delete(depth_msg.header.frame_id)
            return

        mins = np.min(points, axis=0)
        maxs = np.max(points, axis=0)
        center = (mins + maxs) * 0.5
        size = np.maximum(
            maxs - mins + (2.0 * self.obstacle_padding_m),
            self.obstacle_min_size_m,
        )

        marker = Marker()
        marker.header = depth_msg.header
        marker.ns = "holo_assist_depth_tracker_obstacle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(size[0])
        marker.scale.y = float(size[1])
        marker.scale.z = float(size[2])
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.35
        self.obstacle_marker_pub.publish(marker)
        self.obstacle_active = True

    def _publish_obstacle_delete(self, frame_id: str) -> None:
        if self.obstacle_marker_pub is None:
            return
        if not self.obstacle_active:
            return

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "holo_assist_depth_tracker_obstacle"
        marker.id = 0
        marker.action = Marker.DELETE
        self.obstacle_marker_pub.publish(marker)
        self.obstacle_active = False

    def _create_debug_image(self, depth_m: np.ndarray) -> np.ndarray:
        if self.use_rgb_debug_image:
            rgb = self._latest_rgb_for_shape(depth_m.shape)
            if rgb is not None:
                self._last_debug_image_is_rgb = True
                return rgb

        self._last_debug_image_is_rgb = False
        depth_range = max(1e-3, self.max_depth_m - self.min_depth_m)
        clipped = np.clip(depth_m, self.min_depth_m, self.max_depth_m)

        norm = ((clipped - self.min_depth_m) / depth_range * 255.0).astype(np.uint8)
        invalid = ~np.isfinite(depth_m) | (depth_m <= 0.0)
        norm[invalid] = 0

        return cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)

    def _latest_rgb_for_shape(self, depth_shape: tuple[int, int]) -> Optional[np.ndarray]:
        if self.latest_rgb_bgr is None or self.latest_rgb_stamp is None:
            return None
        age_s = (self.get_clock().now() - self.latest_rgb_stamp).nanoseconds / 1e9
        if age_s > self.rgb_timeout_s:
            return None

        height, width = depth_shape
        image = self.latest_rgb_bgr
        if image.shape[0] != height or image.shape[1] != width:
            image = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
        return image.copy()

    def _draw_apriltag_overlay(self, image_bgr: np.ndarray) -> None:
        if not self.enable_apriltag_overlay:
            return
        if self.latest_apriltag_stamp is None:
            return
        if not self.latest_apriltag_detections:
            return

        age_s = (self.get_clock().now() - self.latest_apriltag_stamp).nanoseconds / 1e9
        if age_s > self.apriltag_timeout_s:
            return

        out_h, out_w = image_bgr.shape[:2]
        src_w, src_h = out_w, out_h
        if self.latest_apriltag_image_size is not None:
            src_w = max(1, int(self.latest_apriltag_image_size[0]))
            src_h = max(1, int(self.latest_apriltag_image_size[1]))
        sx = float(out_w) / float(src_w)
        sy = float(out_h) / float(src_h)

        line_color = (0, 255, 255)
        arrow_color = (255, 255, 0)
        line_thickness = 2

        for det in self.latest_apriltag_detections:
            corners = []
            for p in det.corners:
                x = int(round(float(p.x) * sx))
                y = int(round(float(p.y) * sy))
                corners.append((x, y))

            if len(corners) == 4:
                for i in range(4):
                    cv2.line(
                        image_bgr,
                        corners[i],
                        corners[(i + 1) % 4],
                        line_color,
                        line_thickness,
                        cv2.LINE_AA,
                    )
                cv2.arrowedLine(
                    image_bgr,
                    corners[0],
                    corners[1],
                    arrow_color,
                    line_thickness,
                    cv2.LINE_AA,
                    tipLength=0.25,
                )

            cx = int(round(float(det.centre.x) * sx))
            cy = int(round(float(det.centre.y) * sy))
            cv2.circle(image_bgr, (cx, cy), 4, line_color, -1, cv2.LINE_AA)
            cv2.putText(
                image_bgr,
                f"id={int(det.id)}",
                (cx + 6, max(18, cy - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                line_color,
                2,
                cv2.LINE_AA,
            )

        self._draw_workspace_outline_from_tags(
            image_bgr=image_bgr,
            sx=sx,
            sy=sy,
            base_line_thickness=line_thickness,
        )

        cv2.putText(
            image_bgr,
            f"tags={len(self.latest_apriltag_detections)} age={age_s:.2f}s",
            (12, out_h - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            line_color,
            2,
            cv2.LINE_AA,
        )

    def _find_tag_detection(self, tag_id: int):
        for det in self.latest_apriltag_detections:
            if int(det.id) == int(tag_id):
                return det
        return None

    def _scaled_tag_corners(
        self, det, sx: float, sy: float
    ) -> Optional[np.ndarray]:
        if len(det.corners) != 4:
            return None
        corners = np.array(
            [[float(p.x) * sx, float(p.y) * sy] for p in det.corners],
            dtype=np.float32,
        )
        if not np.isfinite(corners).all():
            return None
        return corners

    def _mapped_tag_corners(
        self, corners: np.ndarray, index_offset: int, reverse: bool
    ) -> np.ndarray:
        mapped = corners[::-1].copy() if reverse else corners.copy()
        shift = int(index_offset) % 4
        if shift:
            mapped = np.roll(mapped, -shift, axis=0)
        return mapped

    def _draw_workspace_outline_from_tags(
        self,
        image_bgr: np.ndarray,
        sx: float,
        sy: float,
        base_line_thickness: int,
    ) -> None:
        if not self.enable_workspace_overlay:
            return

        w = float(self.workspace_width_m)
        d = float(self.workspace_depth_m)
        edge = float(self.workspace_tag_center_edge_offset_m)
        s = float(self.workspace_tag_size_m)
        right_x0 = w - s
        board_outline = np.array(
            [[[0.0, 0.0], [w, 0.0], [w, d], [0.0, d]]], dtype=np.float32
        )

        def _is_convex_quad(quad_xy: np.ndarray) -> bool:
            if quad_xy.shape != (4, 2):
                return False
            signs = []
            for i in range(4):
                a = quad_xy[i]
                b = quad_xy[(i + 1) % 4]
                c = quad_xy[(i + 2) % 4]
                ab = b - a
                bc = c - b
                cross_z = float(ab[0] * bc[1] - ab[1] * bc[0])
                if abs(cross_z) < 1e-6:
                    return False
                signs.append(cross_z)
            first = signs[0]
            return all((s > 0.0) == (first > 0.0) for s in signs[1:])

        def _homography_projects_valid_outline(h: np.ndarray) -> bool:
            outline_xy = cv2.perspectiveTransform(board_outline, h).reshape(-1, 2)
            if not np.isfinite(outline_xy).all():
                return False
            if not _is_convex_quad(outline_xy):
                return False
            area2 = 0.0
            for i in range(4):
                x0, y0 = outline_xy[i]
                x1, y1 = outline_xy[(i + 1) % 4]
                area2 += float(x0 * y1 - y0 * x1)
            area_px2 = abs(area2) * 0.5
            return area_px2 >= 100.0

        # Preferred path: use all 4 workspace tag centers to estimate homography.
        # This draws board outer edges from known board geometry and tag center offsets.
        id_to_board_center = {
            int(self.workspace_left_tag_id): np.array([edge, edge], dtype=np.float32),
            int(self.workspace_right_tag_id): np.array([w - edge, edge], dtype=np.float32),
            int(self.workspace_rear_left_tag_id): np.array([edge, d - edge], dtype=np.float32),
            int(self.workspace_rear_right_tag_id): np.array([w - edge, d - edge], dtype=np.float32),
        }
        board_points_center = []
        image_points_center = []
        for tag_id, board_pt in id_to_board_center.items():
            det = self._find_tag_detection(tag_id)
            if det is None:
                continue
            cx = float(det.centre.x) * sx
            cy = float(det.centre.y) * sy
            if not np.isfinite(cx) or not np.isfinite(cy):
                continue
            board_points_center.append(board_pt.tolist())
            image_points_center.append([cx, cy])

        homography = None
        if len(board_points_center) >= 4 and len(image_points_center) >= 4:
            board_center_np = np.asarray(board_points_center, dtype=np.float32)
            image_center_np = np.asarray(image_points_center, dtype=np.float32)
            h_center, _ = cv2.findHomography(board_center_np, image_center_np, 0)
            if h_center is not None:
                projected_center = cv2.perspectiveTransform(
                    board_center_np.reshape(1, -1, 2), h_center
                ).reshape(-1, 2)
                reproj_err_center = float(
                    np.mean(np.linalg.norm(projected_center - image_center_np, axis=1))
                )
                if (
                    reproj_err_center <= self.workspace_outline_max_reproj_error_px
                    and _homography_projects_valid_outline(h_center)
                ):
                    homography = h_center

        # Geometric ordering (TL,TR,BR,BL) avoids ID-to-corner folding artifacts.
        if homography is None and len(image_points_center) >= 4:
            image_center_np = np.asarray(image_points_center, dtype=np.float32)
            sums = image_center_np[:, 0] + image_center_np[:, 1]
            diffs = image_center_np[:, 0] - image_center_np[:, 1]
            tl_idx = int(np.argmin(sums))
            br_idx = int(np.argmax(sums))
            tr_idx = int(np.argmax(diffs))
            bl_idx = int(np.argmin(diffs))
            if len({tl_idx, tr_idx, br_idx, bl_idx}) == 4:
                ordered_img = np.zeros((4, 2), dtype=np.float32)
                ordered_img[0] = image_center_np[tl_idx]  # top-left
                ordered_img[2] = image_center_np[br_idx]  # bottom-right
                ordered_img[1] = image_center_np[tr_idx]  # top-right
                ordered_img[3] = image_center_np[bl_idx]  # bottom-left

                board_center_np = np.array(
                    [
                        [edge, edge],
                        [w - edge, edge],
                        [w - edge, d - edge],
                        [edge, d - edge],
                    ],
                    dtype=np.float32,
                )
                h_center, _ = cv2.findHomography(board_center_np, ordered_img, 0)
                if h_center is not None:
                    projected_center = cv2.perspectiveTransform(
                        board_center_np.reshape(1, -1, 2), h_center
                    ).reshape(-1, 2)
                    reproj_err_center = float(
                        np.mean(np.linalg.norm(projected_center - ordered_img, axis=1))
                    )
                    if (
                        reproj_err_center <= self.workspace_outline_max_reproj_error_px
                        and _homography_projects_valid_outline(h_center)
                    ):
                        homography = h_center

        # Fallback path: legacy 2-tag-corner homography from front tags.
        left_det = self._find_tag_detection(self.workspace_left_tag_id)
        right_det = self._find_tag_detection(self.workspace_right_tag_id)
        if homography is None and left_det is None and right_det is None:
            return

        board_points = []
        image_points = []
        if left_det is not None:
            left_raw = self._scaled_tag_corners(left_det, sx, sy)
            if left_raw is not None:
                left = self._mapped_tag_corners(
                    left_raw,
                    self.workspace_left_corner_index_offset,
                    self.workspace_left_corner_reverse,
                )
                board_points.extend(
                    [[0.0, 0.0], [s, 0.0], [s, s], [0.0, s]]
                )
                image_points.extend(left.tolist())

        if right_det is not None:
            right_raw = self._scaled_tag_corners(right_det, sx, sy)
            if right_raw is not None:
                right = self._mapped_tag_corners(
                    right_raw,
                    self.workspace_right_corner_index_offset,
                    self.workspace_right_corner_reverse,
                )
                board_points.extend(
                    [[right_x0, 0.0], [w, 0.0], [w, s], [right_x0, s]]
                )
                image_points.extend(right.tolist())

        if homography is None:
            if len(board_points) < 4 or len(image_points) < 4:
                return

            board_tag_points = np.asarray(board_points, dtype=np.float32)
            image_tag_points = np.asarray(image_points, dtype=np.float32)
            homography, _ = cv2.findHomography(board_tag_points, image_tag_points, 0)
            if homography is None:
                return

            projected = cv2.perspectiveTransform(
                board_tag_points.reshape(1, -1, 2), homography
            ).reshape(-1, 2)
            reproj_err = float(np.mean(np.linalg.norm(projected - image_tag_points, axis=1)))
            if (
                reproj_err > self.workspace_outline_max_reproj_error_px
                or not _homography_projects_valid_outline(homography)
            ):
                return

        front_edge = np.array([[[0.0, 0.0], [w, 0.0]]], dtype=np.float32)
        outline_px = cv2.perspectiveTransform(board_outline, homography).reshape(-1, 2)
        front_px = cv2.perspectiveTransform(front_edge, homography).reshape(-1, 2)
        if not np.isfinite(outline_px).all() or not np.isfinite(front_px).all():
            return

        outline_int = np.round(outline_px).astype(np.int32).reshape((-1, 1, 2))
        front_int = np.round(front_px).astype(np.int32)
        outline_thickness = max(2, base_line_thickness + 1)
        front_thickness = max(3, base_line_thickness + 2)

        cv2.polylines(
            image_bgr,
            [outline_int],
            True,
            (255, 0, 255),
            outline_thickness,
            cv2.LINE_AA,
        )
        cv2.line(
            image_bgr,
            (int(front_int[0][0]), int(front_int[0][1])),
            (int(front_int[1][0]), int(front_int[1][1])),
            (0, 140, 255),
            front_thickness,
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
