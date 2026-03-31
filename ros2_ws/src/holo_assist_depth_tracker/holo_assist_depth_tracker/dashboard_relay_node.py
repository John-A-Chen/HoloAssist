#!/usr/bin/env python3

import json
import math
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Deque, Dict, Optional
from urllib.parse import urlparse

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import Bool, Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

SENTINEL_BBOX = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 0.0]


class DepthTrackerDashboardRelay(Node):
    """Expose HoloAssist perception + robot telemetry over HTTP for dashboard use."""

    def __init__(self) -> None:
        super().__init__("holo_assist_depth_tracker_dashboard_relay")

        # Relay / transport parameters
        self.declare_parameter("bind_host", "0.0.0.0")
        self.declare_parameter("bind_port", 8765)
        self.declare_parameter("stale_timeout_s", 1.5)
        self.declare_parameter("rate_window_s", 5.0)

        # Image streaming parameters
        self.declare_parameter("max_image_fps", 8.0)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter(
            "debug_image_topic", "/holo_assist_depth_tracker/debug_image"
        )
        self.declare_parameter("enable_rgb_fallback", True)
        self.declare_parameter("fallback_rgb_topic", "/image_raw")

        # Perception topics
        self.declare_parameter("bbox_topic", "/holo_assist_depth_tracker/bbox")
        self.declare_parameter(
            "pointcloud_topic", "/holo_assist_depth_tracker/pointcloud"
        )
        self.declare_parameter(
            "obstacle_topic", "/holo_assist_depth_tracker/obstacle_marker"
        )

        # Robot state / command topics
        self.declare_parameter("enable_robot_state", True)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("target_pose_topic", "/servo_target_pose")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter("eef_frame", "tool0")
        self.declare_parameter("eef_lookup_period_s", 0.05)
        self.declare_parameter("eef_motion_threshold_m", 0.01)

        # Unity / digital twin integration probes
        self.declare_parameter("unity_tcp_host", "127.0.0.1")
        self.declare_parameter("unity_tcp_port", 10000)
        self.declare_parameter("unity_probe_timeout_s", 0.2)
        self.declare_parameter("unity_probe_period_s", 1.0)
        self.declare_parameter("unity_map_loaded_topic", "/unity/map_loaded")

        self.bind_host = str(self.get_parameter("bind_host").value)
        self.bind_port = int(self.get_parameter("bind_port").value)
        self.stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        self.rate_window_s = float(self.get_parameter("rate_window_s").value)

        self.max_image_fps = float(self.get_parameter("max_image_fps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.enable_rgb_fallback = bool(
            self.get_parameter("enable_rgb_fallback").value
        )
        self.fallback_rgb_topic = str(self.get_parameter("fallback_rgb_topic").value)

        self.bbox_topic = str(self.get_parameter("bbox_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.obstacle_topic = str(self.get_parameter("obstacle_topic").value)

        self.enable_robot_state = bool(self.get_parameter("enable_robot_state").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.clicked_point_topic = str(self.get_parameter("clicked_point_topic").value)
        self.command_frame = str(self.get_parameter("command_frame").value)
        self.eef_frame = str(self.get_parameter("eef_frame").value)
        self.eef_lookup_period_s = float(
            self.get_parameter("eef_lookup_period_s").value
        )
        self.eef_motion_threshold_m = float(
            self.get_parameter("eef_motion_threshold_m").value
        )

        self.unity_tcp_host = str(self.get_parameter("unity_tcp_host").value)
        self.unity_tcp_port = int(self.get_parameter("unity_tcp_port").value)
        self.unity_probe_timeout_s = float(
            self.get_parameter("unity_probe_timeout_s").value
        )
        self.unity_probe_period_s = float(
            self.get_parameter("unity_probe_period_s").value
        )
        self.unity_map_loaded_topic = str(
            self.get_parameter("unity_map_loaded_topic").value
        )

        self._validate_parameters()

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._started_monotonic = time.monotonic()

        self._rate_streams: Dict[str, Deque[float]] = {
            "debug_image": deque(),
            "raw_image": deque(),
            "bbox": deque(),
            "pointcloud": deque(),
            "obstacle": deque(),
            "joint_states": deque(),
            "target_pose": deque(),
            "twist_cmd": deque(),
            "clicked_point": deque(),
            "eef_pose": deque(),
        }

        # Perception counters and last-seen data
        self.debug_image_rx_count = 0
        self.raw_image_rx_count = 0
        self.bbox_rx_count = 0
        self.pointcloud_rx_count = 0
        self.obstacle_rx_count = 0

        self.last_debug_rx_time: Optional[float] = None
        self.last_raw_rx_time: Optional[float] = None
        self.last_bbox_rx_time: Optional[float] = None
        self.last_pointcloud_rx_time: Optional[float] = None
        self.last_obstacle_rx_time: Optional[float] = None

        self.last_bbox = list(SENTINEL_BBOX)
        self.last_point_count = 0
        self.obstacle_active = False

        # Image relay cache
        self._latest_debug_jpeg: Optional[bytes] = None
        self._latest_debug_frame_id = ""
        self._latest_image_source = "none"
        self._latest_image_topic = ""
        self._last_jpeg_encode_time = 0.0

        # Robot state / commands
        self.joint_state_rx_count = 0
        self.target_pose_rx_count = 0
        self.twist_cmd_rx_count = 0
        self.clicked_point_rx_count = 0

        self.last_joint_rx_time: Optional[float] = None
        self.last_target_rx_time: Optional[float] = None
        self.last_twist_rx_time: Optional[float] = None
        self.last_clicked_point_rx_time: Optional[float] = None

        self.joint_names: list[str] = []
        self.joint_positions: list[float] = []
        self.joint_velocities: list[float] = []
        self.joint_efforts: list[float] = []

        self.last_target_transport_latency_ms: Optional[float] = None
        self.target_transport_latencies_ms: Deque[float] = deque(maxlen=30)

        self.last_eef_rx_time: Optional[float] = None
        self.eef_pose: Optional[dict] = None

        self.pending_goal_time_mono: Optional[float] = None
        self.pending_goal_baseline_xyz: Optional[tuple[float, float, float]] = None
        self.last_ee_response_latency_ms: Optional[float] = None

        # Unity integration probes
        self.unity_probe_last_time: Optional[float] = None
        self.unity_probe_last_success_time: Optional[float] = None
        self.unity_tcp_endpoint_reachable = False

        self.unity_map_signal_count = 0
        self.last_unity_map_rx_time: Optional[float] = None
        self.unity_map_loaded: Optional[bool] = None

        # Subscriptions
        self.create_subscription(
            Image,
            self.debug_image_topic,
            self._debug_image_callback,
            qos_profile_sensor_data,
        )
        if self.enable_rgb_fallback:
            self.create_subscription(
                Image,
                self.fallback_rgb_topic,
                self._rgb_fallback_callback,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Float32MultiArray,
            self.bbox_topic,
            self._bbox_callback,
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._pointcloud_callback,
            reliable_qos,
        )
        self.create_subscription(
            Marker,
            self.obstacle_topic,
            self._obstacle_callback,
            reliable_qos,
        )

        if self.enable_robot_state:
            self.create_subscription(
                JointState,
                self.joint_states_topic,
                self._joint_states_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PoseStamped,
                self.target_pose_topic,
                self._target_pose_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                TwistStamped,
                self.twist_topic,
                self._twist_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PointStamped,
                self.clicked_point_topic,
                self._clicked_point_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Bool,
                self.unity_map_loaded_topic,
                self._unity_map_loaded_callback,
                10,
            )

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.eef_timer = self.create_timer(
                self.eef_lookup_period_s,
                self._eef_lookup_timer_cb,
            )
        else:
            self.tf_buffer = None
            self.tf_listener = None
            self.eef_timer = None

        self.unity_probe_timer = self.create_timer(
            self.unity_probe_period_s,
            self._unity_probe_timer_cb,
        )

        self.http_server: Optional[ThreadingHTTPServer] = None
        self.http_thread: Optional[threading.Thread] = None
        self._start_http_server()

        self.get_logger().info(
            f"Dashboard relay started on http://{self.bind_host}:{self.bind_port}"
        )
        self.get_logger().info(
            "Endpoints: /api/perception/status, /api/perception/debug.jpg"
        )
        if self.enable_rgb_fallback:
            self.get_logger().info(
                f"RGB fallback enabled on topic={self.fallback_rgb_topic}"
            )
        if self.enable_robot_state:
            self.get_logger().info(
                f"Robot state enabled. joint_states={self.joint_states_topic}, "
                f"target_pose={self.target_pose_topic}, twist={self.twist_topic}, "
                f"clicked_point={self.clicked_point_topic}, eef={self.command_frame}->{self.eef_frame}"
            )

    def _validate_parameters(self) -> None:
        if self.bind_port <= 0 or self.bind_port > 65535:
            self.get_logger().warn("bind_port must be in 1..65535. Using 8765.")
            self.bind_port = 8765
        if self.stale_timeout_s <= 0.0:
            self.get_logger().warn("stale_timeout_s must be > 0. Using 1.5.")
            self.stale_timeout_s = 1.5
        if self.rate_window_s <= 0.2:
            self.get_logger().warn("rate_window_s must be > 0.2. Using 5.0.")
            self.rate_window_s = 5.0
        if self.max_image_fps <= 0.0:
            self.get_logger().warn("max_image_fps must be > 0. Using 8.0.")
            self.max_image_fps = 8.0
        self.jpeg_quality = max(40, min(95, self.jpeg_quality))

        if self.eef_lookup_period_s <= 0.01:
            self.eef_lookup_period_s = 0.05
        if self.eef_motion_threshold_m <= 0.0:
            self.eef_motion_threshold_m = 0.01

        if self.unity_tcp_port <= 0 or self.unity_tcp_port > 65535:
            self.unity_tcp_port = 10000
        if self.unity_probe_timeout_s <= 0.0:
            self.unity_probe_timeout_s = 0.2
        if self.unity_probe_period_s <= 0.1:
            self.unity_probe_period_s = 1.0

    def _record_rate_locked(self, name: str, now_mono: float) -> None:
        stream = self._rate_streams.get(name)
        if stream is None:
            return
        stream.append(now_mono)
        self._trim_rate_locked(stream, now_mono)

    def _trim_rate_locked(self, stream: Deque[float], now_mono: float) -> None:
        cutoff = now_mono - self.rate_window_s
        while stream and stream[0] < cutoff:
            stream.popleft()

    def _rate_hz_locked(self, name: str, now_mono: float) -> float:
        stream = self._rate_streams.get(name)
        if stream is None:
            return 0.0
        self._trim_rate_locked(stream, now_mono)
        if len(stream) <= 1:
            return 0.0
        window = max(1e-6, min(self.rate_window_s, now_mono - stream[0]))
        return float(len(stream) - 1) / window

    def _debug_image_callback(self, msg: Image) -> None:
        now = time.monotonic()

        with self._lock:
            self.debug_image_rx_count += 1
            self.last_debug_rx_time = now
            self._record_rate_locked("debug_image", now)
            should_encode = (
                now - self._last_jpeg_encode_time
            ) >= (1.0 / self.max_image_fps)

        if not should_encode:
            return

        self._encode_and_store_image(
            msg=msg,
            source="depth_tracker",
            source_topic=self.debug_image_topic,
            now=now,
        )

    def _rgb_fallback_callback(self, msg: Image) -> None:
        now = time.monotonic()

        with self._lock:
            self.raw_image_rx_count += 1
            self.last_raw_rx_time = now
            self._record_rate_locked("raw_image", now)

            debug_is_fresh = (
                self.last_debug_rx_time is not None
                and (now - self.last_debug_rx_time) <= self.stale_timeout_s
            )
            should_encode = (
                now - self._last_jpeg_encode_time
            ) >= (1.0 / self.max_image_fps)

        # Never override fresh depth-tracker debug imagery.
        if debug_is_fresh or not should_encode:
            return

        self._encode_and_store_image(
            msg=msg,
            source="rgb_fallback",
            source_topic=self.fallback_rgb_topic,
            now=now,
        )

    def _encode_and_store_image(
        self,
        msg: Image,
        source: str,
        source_topic: str,
        now: float,
    ) -> None:
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warn(
                f"Failed to convert image for dashboard relay ({source}): {exc}"
            )
            return

        ok, encoded = cv2.imencode(
            ".jpg",
            image_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self.get_logger().warn("JPEG encoding failed for debug image relay.")
            return

        with self._lock:
            self._latest_debug_jpeg = encoded.tobytes()
            self._latest_debug_frame_id = msg.header.frame_id
            self._latest_image_source = source
            self._latest_image_topic = source_topic
            self._last_jpeg_encode_time = now

    def _bbox_callback(self, msg: Float32MultiArray) -> None:
        now = time.monotonic()
        bbox_data = list(msg.data)

        with self._lock:
            self.bbox_rx_count += 1
            self.last_bbox_rx_time = now
            self._record_rate_locked("bbox", now)
            if len(bbox_data) >= 8:
                self.last_bbox = bbox_data[:8]
            else:
                self.last_bbox = list(SENTINEL_BBOX)

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        now = time.monotonic()
        point_count = int(msg.width * msg.height)

        with self._lock:
            self.pointcloud_rx_count += 1
            self.last_pointcloud_rx_time = now
            self.last_point_count = point_count
            self._record_rate_locked("pointcloud", now)

    def _obstacle_callback(self, msg: Marker) -> None:
        now = time.monotonic()
        active = msg.action not in (Marker.DELETE, Marker.DELETEALL)

        with self._lock:
            self.obstacle_rx_count += 1
            self.last_obstacle_rx_time = now
            self.obstacle_active = active
            self._record_rate_locked("obstacle", now)

    def _joint_states_callback(self, msg: JointState) -> None:
        now = time.monotonic()
        names = list(msg.name)
        positions = list(msg.position)
        velocities = list(msg.velocity)
        efforts = list(msg.effort)

        with self._lock:
            self.joint_state_rx_count += 1
            self.last_joint_rx_time = now
            self.joint_names = names
            self.joint_positions = positions
            self.joint_velocities = velocities
            self.joint_efforts = efforts
            self._record_rate_locked("joint_states", now)

    def _target_pose_callback(self, msg: PoseStamped) -> None:
        now_mono = time.monotonic()
        now_ros = self.get_clock().now()
        latency_ms: Optional[float] = None

        try:
            if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
                source_stamp = Time.from_msg(msg.header.stamp)
                delta_ns = (now_ros - source_stamp).nanoseconds
                if 0 <= delta_ns <= int(120.0 * 1e9):
                    latency_ms = delta_ns / 1e6
        except Exception:
            latency_ms = None

        with self._lock:
            self.target_pose_rx_count += 1
            self.last_target_rx_time = now_mono
            self._record_rate_locked("target_pose", now_mono)

            if latency_ms is not None:
                self.last_target_transport_latency_ms = latency_ms
                self.target_transport_latencies_ms.append(latency_ms)

            self.pending_goal_time_mono = now_mono
            if self.eef_pose is not None:
                self.pending_goal_baseline_xyz = (
                    float(self.eef_pose["x"]),
                    float(self.eef_pose["y"]),
                    float(self.eef_pose["z"]),
                )
            else:
                self.pending_goal_baseline_xyz = None

    def _twist_callback(self, _msg: TwistStamped) -> None:
        now = time.monotonic()
        with self._lock:
            self.twist_cmd_rx_count += 1
            self.last_twist_rx_time = now
            self._record_rate_locked("twist_cmd", now)

    def _clicked_point_callback(self, _msg: PointStamped) -> None:
        now = time.monotonic()
        with self._lock:
            self.clicked_point_rx_count += 1
            self.last_clicked_point_rx_time = now
            self._record_rate_locked("clicked_point", now)

    def _unity_map_loaded_callback(self, msg: Bool) -> None:
        now = time.monotonic()
        with self._lock:
            self.unity_map_signal_count += 1
            self.last_unity_map_rx_time = now
            self.unity_map_loaded = bool(msg.data)

    def _eef_lookup_timer_cb(self) -> None:
        if self.tf_buffer is None:
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.command_frame,
                self.eef_frame,
                Time(),
            )
        except TransformException:
            return

        now = time.monotonic()
        x = float(tf_msg.transform.translation.x)
        y = float(tf_msg.transform.translation.y)
        z = float(tf_msg.transform.translation.z)
        qx = float(tf_msg.transform.rotation.x)
        qy = float(tf_msg.transform.rotation.y)
        qz = float(tf_msg.transform.rotation.z)
        qw = float(tf_msg.transform.rotation.w)

        with self._lock:
            self.last_eef_rx_time = now
            self._record_rate_locked("eef_pose", now)
            self.eef_pose = {
                "frame": self.command_frame,
                "x": x,
                "y": y,
                "z": z,
                "qx": qx,
                "qy": qy,
                "qz": qz,
                "qw": qw,
            }

            if (
                self.pending_goal_time_mono is not None
                and self.pending_goal_baseline_xyz is not None
            ):
                bx, by, bz = self.pending_goal_baseline_xyz
                dist = math.sqrt((x - bx) ** 2 + (y - by) ** 2 + (z - bz) ** 2)
                if dist >= self.eef_motion_threshold_m:
                    self.last_ee_response_latency_ms = (
                        now - self.pending_goal_time_mono
                    ) * 1000.0
                    self.pending_goal_time_mono = None
                    self.pending_goal_baseline_xyz = None

    def _unity_probe_timer_cb(self) -> None:
        now = time.monotonic()
        reachable = False

        try:
            with socket.create_connection(
                (self.unity_tcp_host, self.unity_tcp_port),
                timeout=self.unity_probe_timeout_s,
            ):
                reachable = True
        except OSError:
            reachable = False

        with self._lock:
            self.unity_probe_last_time = now
            self.unity_tcp_endpoint_reachable = reachable
            if reachable:
                self.unity_probe_last_success_time = now

    def _age_seconds(self, now: float, previous: Optional[float]) -> float:
        if previous is None:
            return -1.0
        return max(0.0, now - previous)

    def _build_status_payload(self) -> dict:
        now_mono = time.monotonic()
        now_unix_ms = int(time.time() * 1000.0)

        with self._lock:
            debug_age_s = self._age_seconds(now_mono, self.last_debug_rx_time)
            raw_age_s = self._age_seconds(now_mono, self.last_raw_rx_time)
            bbox_age_s = self._age_seconds(now_mono, self.last_bbox_rx_time)
            pointcloud_age_s = self._age_seconds(now_mono, self.last_pointcloud_rx_time)
            obstacle_age_s = self._age_seconds(now_mono, self.last_obstacle_rx_time)

            bbox = list(self.last_bbox)
            debug_image_rx_count = self.debug_image_rx_count
            raw_image_rx_count = self.raw_image_rx_count
            bbox_rx_count = self.bbox_rx_count
            pointcloud_rx_count = self.pointcloud_rx_count
            obstacle_rx_count = self.obstacle_rx_count
            last_point_count = self.last_point_count
            obstacle_active = self.obstacle_active
            jpeg_available = self._latest_debug_jpeg is not None
            latest_debug_frame_id = self._latest_debug_frame_id
            latest_image_source = self._latest_image_source
            latest_image_topic = self._latest_image_topic

            tracker_connected = (
                debug_age_s >= 0.0 and debug_age_s <= self.stale_timeout_s
            )
            rgb_fallback_connected = (
                self.enable_rgb_fallback
                and raw_age_s >= 0.0
                and raw_age_s <= self.stale_timeout_s
            )

            if tracker_connected:
                source_mode = "depth_tracker"
            elif rgb_fallback_connected:
                source_mode = "rgb_fallback"
            else:
                source_mode = "no_data"

            pipeline_connected = tracker_connected or rgb_fallback_connected
            depth_supported = tracker_connected
            if source_mode == "rgb_fallback":
                depth_message = (
                    "This camera source does not provide depth; pointcloud and obstacle outputs are unavailable."
                )
            elif source_mode == "depth_tracker":
                depth_message = "Depth tracker stream active."
            else:
                depth_message = "No active image stream."

            bbox_valid = (
                len(bbox) >= 8
                and bbox[0] >= 0.0
                and bbox[1] >= 0.0
                and bbox[7] > 0.0
            )
            if bbox_valid:
                bbox_details = {
                    "x_min_px": bbox[0],
                    "y_min_px": bbox[1],
                    "x_max_px": bbox[2],
                    "y_max_px": bbox[3],
                    "cx_px": bbox[4],
                    "cy_px": bbox[5],
                    "median_depth_m": bbox[6],
                    "area_px": bbox[7],
                }
            else:
                bbox_details = None

            joint_age_s = self._age_seconds(now_mono, self.last_joint_rx_time)
            target_age_s = self._age_seconds(now_mono, self.last_target_rx_time)
            twist_age_s = self._age_seconds(now_mono, self.last_twist_rx_time)
            clicked_age_s = self._age_seconds(now_mono, self.last_clicked_point_rx_time)
            eef_age_s = self._age_seconds(now_mono, self.last_eef_rx_time)

            unity_probe_age_s = self._age_seconds(now_mono, self.unity_probe_last_time)
            unity_success_age_s = self._age_seconds(
                now_mono,
                self.unity_probe_last_success_time,
            )
            unity_map_age_s = self._age_seconds(now_mono, self.last_unity_map_rx_time)

            joint_names = list(self.joint_names)
            joint_positions = list(self.joint_positions)
            joint_velocities = list(self.joint_velocities)
            joint_efforts = list(self.joint_efforts)

            eef_pose = dict(self.eef_pose) if self.eef_pose is not None else None

            last_target_transport_latency_ms = self.last_target_transport_latency_ms
            avg_target_transport_latency_ms = (
                sum(self.target_transport_latencies_ms)
                / len(self.target_transport_latencies_ms)
                if self.target_transport_latencies_ms
                else None
            )

            last_ee_response_latency_ms = self.last_ee_response_latency_ms
            pending_goal = self.pending_goal_time_mono is not None

            rates_hz = {
                "debug_image": self._rate_hz_locked("debug_image", now_mono),
                "raw_image": self._rate_hz_locked("raw_image", now_mono),
                "bbox": self._rate_hz_locked("bbox", now_mono),
                "pointcloud": self._rate_hz_locked("pointcloud", now_mono),
                "obstacle": self._rate_hz_locked("obstacle", now_mono),
                "joint_states": self._rate_hz_locked("joint_states", now_mono),
                "target_pose": self._rate_hz_locked("target_pose", now_mono),
                "twist_cmd": self._rate_hz_locked("twist_cmd", now_mono),
                "clicked_point": self._rate_hz_locked("clicked_point", now_mono),
                "eef_pose": self._rate_hz_locked("eef_pose", now_mono),
            }

            unity_tcp_endpoint_reachable = self.unity_tcp_endpoint_reachable
            unity_map_loaded = self.unity_map_loaded
            unity_map_signal_count = self.unity_map_signal_count

            joint_state_rx_count = self.joint_state_rx_count
            target_pose_rx_count = self.target_pose_rx_count
            twist_cmd_rx_count = self.twist_cmd_rx_count
            clicked_point_rx_count = self.clicked_point_rx_count

        robot_joint_available = (
            self.enable_robot_state
            and joint_age_s >= 0.0
            and joint_age_s <= self.stale_timeout_s
        )
        eef_available = (
            self.enable_robot_state
            and eef_age_s >= 0.0
            and eef_age_s <= self.stale_timeout_s
            and eef_pose is not None
        )

        joint_rows = []
        max_joint_len = max(
            len(joint_names),
            len(joint_positions),
            len(joint_velocities),
            len(joint_efforts),
        )
        for idx in range(max_joint_len):
            joint_rows.append(
                {
                    "name": (
                        joint_names[idx]
                        if idx < len(joint_names)
                        else f"joint_{idx + 1}"
                    ),
                    "position_rad": (
                        float(joint_positions[idx])
                        if idx < len(joint_positions)
                        else None
                    ),
                    "velocity_rad_s": (
                        float(joint_velocities[idx])
                        if idx < len(joint_velocities)
                        else None
                    ),
                    "effort": (
                        float(joint_efforts[idx])
                        if idx < len(joint_efforts)
                        else None
                    ),
                }
            )

        if unity_tcp_endpoint_reachable:
            unity_message = "ROS-TCP endpoint is reachable (Unity bridge server listening)."
        else:
            unity_message = "ROS-TCP endpoint not reachable on configured host/port."

        return {
            "server_time_unix_ms": now_unix_ms,
            "bridge": {
                "bind_host": self.bind_host,
                "bind_port": self.bind_port,
                "uptime_s": round(now_mono - self._started_monotonic, 3),
                "stale_timeout_s": self.stale_timeout_s,
                "max_image_fps": self.max_image_fps,
                "jpeg_quality": self.jpeg_quality,
                "rate_window_s": self.rate_window_s,
            },
            "topics": {
                "debug_image_topic": self.debug_image_topic,
                "fallback_rgb_topic": (
                    self.fallback_rgb_topic if self.enable_rgb_fallback else ""
                ),
                "bbox_topic": self.bbox_topic,
                "pointcloud_topic": self.pointcloud_topic,
                "obstacle_topic": self.obstacle_topic,
                "joint_states_topic": self.joint_states_topic,
                "target_pose_topic": self.target_pose_topic,
                "twist_topic": self.twist_topic,
                "clicked_point_topic": self.clicked_point_topic,
                "unity_map_loaded_topic": self.unity_map_loaded_topic,
            },
            "source": {
                "mode": source_mode,
                "active_image_topic": latest_image_topic,
                "latest_image_source": latest_image_source,
            },
            "capabilities": {
                "depth_supported": depth_supported,
                "pointcloud_supported": depth_supported,
                "obstacle_supported": depth_supported,
                "message": depth_message,
            },
            "tracker_connection": {
                "connected": tracker_connected,
                "pipeline_connected": pipeline_connected,
                "stale": not pipeline_connected,
                "last_debug_age_s": round(debug_age_s, 3),
                "last_rgb_age_s": round(raw_age_s, 3),
                "last_bbox_age_s": round(bbox_age_s, 3),
                "last_pointcloud_age_s": round(pointcloud_age_s, 3),
                "last_obstacle_age_s": round(obstacle_age_s, 3),
            },
            "rates_hz": {k: round(v, 3) for k, v in rates_hz.items()},
            "counters": {
                "debug_image_rx_count": debug_image_rx_count,
                "raw_image_rx_count": raw_image_rx_count,
                "bbox_rx_count": bbox_rx_count,
                "pointcloud_rx_count": pointcloud_rx_count,
                "obstacle_rx_count": obstacle_rx_count,
                "joint_state_rx_count": joint_state_rx_count,
                "target_pose_rx_count": target_pose_rx_count,
                "twist_cmd_rx_count": twist_cmd_rx_count,
                "clicked_point_rx_count": clicked_point_rx_count,
                "last_point_count": last_point_count,
            },
            "latest": {
                "obstacle_active": obstacle_active,
                "bbox_valid": bbox_valid,
                "bbox_raw": bbox,
                "bbox": bbox_details,
                "jpeg_available": jpeg_available,
                "latest_debug_frame_id": latest_debug_frame_id,
                "image_source": latest_image_source,
            },
            "robot_state": {
                "enabled": self.enable_robot_state,
                "joint_state_available": robot_joint_available,
                "joint_state_age_s": round(joint_age_s, 3),
                "joint_count": max_joint_len,
                "joints": joint_rows,
                "eef_pose_available": eef_available,
                "eef_pose_age_s": round(eef_age_s, 3),
                "eef_pose": eef_pose,
                "command_frame": self.command_frame,
                "eef_frame": self.eef_frame,
                "command_metrics": {
                    "last_target_age_s": round(target_age_s, 3),
                    "last_twist_age_s": round(twist_age_s, 3),
                    "last_clicked_point_age_s": round(clicked_age_s, 3),
                    "last_target_transport_latency_ms": (
                        round(last_target_transport_latency_ms, 3)
                        if last_target_transport_latency_ms is not None
                        else None
                    ),
                    "avg_target_transport_latency_ms": (
                        round(avg_target_transport_latency_ms, 3)
                        if avg_target_transport_latency_ms is not None
                        else None
                    ),
                    "last_ee_response_latency_ms": (
                        round(last_ee_response_latency_ms, 3)
                        if last_ee_response_latency_ms is not None
                        else None
                    ),
                    "pending_goal": pending_goal,
                    "eef_motion_threshold_m": self.eef_motion_threshold_m,
                },
            },
            "unity_integration": {
                "tcp_host": self.unity_tcp_host,
                "tcp_port": self.unity_tcp_port,
                "tcp_endpoint_reachable": unity_tcp_endpoint_reachable,
                "last_probe_age_s": round(unity_probe_age_s, 3),
                "last_success_age_s": round(unity_success_age_s, 3),
                "message": unity_message,
                "map_signal_received": unity_map_age_s >= 0.0,
                "map_signal_age_s": round(unity_map_age_s, 3),
                "map_loaded": unity_map_loaded,
                "map_signal_count": unity_map_signal_count,
            },
        }

    def _build_status_json(self) -> bytes:
        return json.dumps(self._build_status_payload(), separators=(",", ":")).encode(
            "utf-8"
        )

    def _get_latest_debug_jpeg(self) -> Optional[bytes]:
        with self._lock:
            if self._latest_debug_jpeg is None:
                return None
            return bytes(self._latest_debug_jpeg)

    def _start_http_server(self) -> None:
        relay = self

        class RequestHandler(BaseHTTPRequestHandler):
            def _send_headers(self, status_code: int, content_type: str) -> None:
                self.send_response(status_code)
                self.send_header("Content-Type", content_type)
                self.send_header(
                    "Cache-Control",
                    "no-store, no-cache, must-revalidate, max-age=0",
                )
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()

            def do_OPTIONS(self) -> None:  # noqa: N802
                self._send_headers(204, "text/plain")

            def do_GET(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                path = parsed.path

                if path == "/api/perception/status":
                    payload = relay._build_status_json()
                    self._send_headers(200, "application/json; charset=utf-8")
                    self.wfile.write(payload)
                    return

                if path == "/api/perception/debug.jpg":
                    jpeg = relay._get_latest_debug_jpeg()
                    if jpeg is None:
                        self._send_headers(503, "text/plain; charset=utf-8")
                        self.wfile.write(b"No image received yet")
                        return
                    self._send_headers(200, "image/jpeg")
                    self.wfile.write(jpeg)
                    return

                if path in ("/", "/healthz"):
                    self._send_headers(200, "application/json; charset=utf-8")
                    self.wfile.write(b'{"ok":true}')
                    return

                self._send_headers(404, "application/json; charset=utf-8")
                self.wfile.write(b'{"error":"not_found"}')

            def log_message(self, fmt: str, *args) -> None:
                relay.get_logger().debug(f"HTTP {self.command} - {fmt % args}")

        try:
            self.http_server = ThreadingHTTPServer(
                (self.bind_host, self.bind_port),
                RequestHandler,
            )
        except OSError as exc:
            raise RuntimeError(
                f"Failed to bind dashboard relay HTTP server on {self.bind_host}:{self.bind_port}: {exc}"
            ) from exc

        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            daemon=True,
        )
        self.http_thread.start()

    def shutdown_relay(self) -> None:
        if self.http_server is not None:
            self.http_server.shutdown()
            self.http_server.server_close()
            self.http_server = None
        if self.http_thread is not None:
            self.http_thread.join(timeout=1.0)
            self.http_thread = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthTrackerDashboardRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_relay()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
