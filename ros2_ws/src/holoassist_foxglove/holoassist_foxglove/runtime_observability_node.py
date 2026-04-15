#!/usr/bin/env python3

from __future__ import annotations

import json
import os
import socket
import time
from collections import deque
from typing import Deque, Dict, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image, JointState, PointCloud2
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from visualization_msgs.msg import Marker

try:
    from ur_dashboard_msgs.msg import RobotMode, SafetyMode

    HAVE_UR_DASHBOARD = True
except Exception:
    RobotMode = None
    SafetyMode = None
    HAVE_UR_DASHBOARD = False


HEARTBEAT_TOPIC_BY_SUBSYSTEM = {
    "xr_interface": "/xr_interface/heartbeat",
    "perception": "/perception/heartbeat",
    "planning": "/planning/heartbeat",
    "control": "/control/heartbeat",
    "rviz": "/rviz/heartbeat",
    "unity_bridge": "/unity_bridge/heartbeat",
    "robot_bringup": "/robot_bringup/heartbeat",
}


class RuntimeObservabilityNode(Node):
    """Aggregates runtime health, metrics, events, and heartbeats for Foxglove."""

    def __init__(self) -> None:
        super().__init__("holoassist_runtime_observability")

        self.declare_parameter("diagnostics_rate_hz", 1.0)
        self.declare_parameter("rate_window_s", 5.0)
        self.declare_parameter("stale_timeout_s", 2.5)
        self.declare_parameter("heartbeat_interval_s", 1.0)
        self.declare_parameter("unity_tcp_host", "127.0.0.1")
        self.declare_parameter("unity_tcp_port", 10000)
        self.declare_parameter("unity_probe_timeout_s", 0.2)
        self.declare_parameter("foxglove_bridge_host", "127.0.0.1")
        self.declare_parameter("foxglove_bridge_port", 8765)
        self.declare_parameter("foxglove_probe_timeout_s", 0.2)

        self.declare_parameter("debug_image_topic", "/holo_assist_depth_tracker/debug_image")
        self.declare_parameter("headset_image_topic", "/headset/image_compressed")
        self.declare_parameter("bbox_topic", "/holo_assist_depth_tracker/bbox")
        self.declare_parameter("pointcloud_topic", "/holo_assist_depth_tracker/pointcloud")
        self.declare_parameter("obstacle_topic", "/holo_assist_depth_tracker/obstacle_marker")
        self.declare_parameter("planner_marker_topic", "/clicked_goal_marker")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("target_pose_topic", "/servo_target_pose")
        self.declare_parameter("twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("unity_map_loaded_topic", "/unity/map_loaded")
        self.declare_parameter("keyboard_command_topic", "/ur3_keyboard/command_text")
        self.declare_parameter("robot_command_topic", "/ur3_keyboard/robot_command_text")
        self.declare_parameter("robot_mode_topic", "/io_and_status_controller/robot_mode")
        self.declare_parameter("safety_mode_topic", "/io_and_status_controller/safety_mode")

        self.diagnostics_rate_hz = max(
            0.1, float(self.get_parameter("diagnostics_rate_hz").value)
        )
        self.rate_window_s = max(1.0, float(self.get_parameter("rate_window_s").value))
        self.stale_timeout_s = max(
            0.5, float(self.get_parameter("stale_timeout_s").value)
        )
        self.heartbeat_interval_s = max(
            0.5, float(self.get_parameter("heartbeat_interval_s").value)
        )

        self.unity_tcp_host = str(self.get_parameter("unity_tcp_host").value)
        self.unity_tcp_port = int(self.get_parameter("unity_tcp_port").value)
        self.unity_probe_timeout_s = float(
            self.get_parameter("unity_probe_timeout_s").value
        )
        self.foxglove_bridge_host = str(
            self.get_parameter("foxglove_bridge_host").value
        )
        self.foxglove_bridge_port = int(
            self.get_parameter("foxglove_bridge_port").value
        )
        self.foxglove_probe_timeout_s = float(
            self.get_parameter("foxglove_probe_timeout_s").value
        )

        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.headset_image_topic = str(self.get_parameter("headset_image_topic").value)
        self.bbox_topic = str(self.get_parameter("bbox_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.obstacle_topic = str(self.get_parameter("obstacle_topic").value)
        self.planner_marker_topic = str(self.get_parameter("planner_marker_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.clicked_point_topic = str(self.get_parameter("clicked_point_topic").value)
        self.unity_map_loaded_topic = str(
            self.get_parameter("unity_map_loaded_topic").value
        )
        self.keyboard_command_topic = str(
            self.get_parameter("keyboard_command_topic").value
        )
        self.robot_command_topic = str(self.get_parameter("robot_command_topic").value)
        self.robot_mode_topic = str(self.get_parameter("robot_mode_topic").value)
        self.safety_mode_topic = str(self.get_parameter("safety_mode_topic").value)

        self._rate_streams: Dict[str, Deque[float]] = {
            "debug_image": deque(),
            "headset_image": deque(),
            "bbox": deque(),
            "pointcloud": deque(),
            "obstacle": deque(),
            "planner_marker": deque(),
            "joint_states": deque(),
            "target_pose": deque(),
            "twist_cmd": deque(),
            "clicked_point": deque(),
            "unity_map_loaded": deque(),
            "keyboard_command": deque(),
            "robot_command": deque(),
        }
        self._last_seen: Dict[str, float] = {}
        self._last_diag_levels: Dict[str, int] = {}

        self._last_target_transport_latency_ms: Optional[float] = None
        self._last_twist_transport_latency_ms: Optional[float] = None
        self._last_joint_count: int = 0
        self._last_pointcloud_points: int = 0
        self._obstacle_active: bool = False
        self._unity_map_loaded: Optional[bool] = None
        self._robot_mode_value: Optional[int] = None
        self._safety_mode_value: Optional[int] = None
        self._last_keyboard_command: str = ""
        self._last_robot_command: str = ""
        self._last_event_text: str = ""
        self._last_event_time: float = 0.0

        self._latched_qos = QoSProfile(depth=1)
        self._latched_qos.reliability = ReliabilityPolicy.RELIABLE
        self._latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/holoassist/diagnostics", 10
        )
        self.events_pub = self.create_publisher(String, "/holoassist/events", 100)

        self.teleop_state_pub = self.create_publisher(
            String, "/holoassist/state/teleop", self._latched_qos
        )
        self.planner_state_pub = self.create_publisher(
            String, "/holoassist/state/planner", self._latched_qos
        )
        self.safety_state_pub = self.create_publisher(
            String, "/holoassist/state/safety", self._latched_qos
        )
        self.runtime_state_pub = self.create_publisher(
            String, "/holoassist/state/runtime", self._latched_qos
        )

        self.joint_rate_pub = self.create_publisher(
            Float32, "/holoassist/metrics/joint_states_hz", 10
        )
        self.pointcloud_rate_pub = self.create_publisher(
            Float32, "/holoassist/metrics/pointcloud_hz", 10
        )
        self.debug_image_rate_pub = self.create_publisher(
            Float32, "/holoassist/metrics/debug_image_hz", 10
        )
        self.target_latency_pub = self.create_publisher(
            Float32, "/holoassist/metrics/target_transport_latency_ms", 10
        )
        self.twist_latency_pub = self.create_publisher(
            Float32, "/holoassist/metrics/twist_transport_latency_ms", 10
        )
        self.unity_tcp_latency_pub = self.create_publisher(
            Float32, "/holoassist/metrics/unity_tcp_latency_ms", 10
        )
        self.foxglove_tcp_latency_pub = self.create_publisher(
            Float32, "/holoassist/metrics/foxglove_tcp_latency_ms", 10
        )

        self._heartbeat_pubs = {
            name: self.create_publisher(String, topic, 10)
            for name, topic in HEARTBEAT_TOPIC_BY_SUBSYSTEM.items()
        }

        self.create_subscription(
            Image,
            self.debug_image_topic,
            lambda _msg: self._mark_rx("debug_image"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CompressedImage,
            self.headset_image_topic,
            lambda _msg: self._mark_rx("headset_image"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32MultiArray,
            self.bbox_topic,
            lambda _msg: self._mark_rx("bbox"),
            10,
        )
        self.create_subscription(
            PointCloud2, self.pointcloud_topic, self._on_pointcloud, qos_profile_sensor_data
        )
        self.create_subscription(
            Marker, self.obstacle_topic, self._on_obstacle_marker, qos_profile_sensor_data
        )
        self.create_subscription(
            Marker,
            self.planner_marker_topic,
            lambda _msg: self._mark_rx("planner_marker"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState, self.joint_states_topic, self._on_joint_states, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, self.target_pose_topic, self._on_target_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            TwistStamped, self.twist_topic, self._on_twist, qos_profile_sensor_data
        )
        self.create_subscription(
            PointStamped,
            self.clicked_point_topic,
            lambda _msg: self._mark_rx("clicked_point"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool, self.unity_map_loaded_topic, self._on_unity_map_loaded, 10
        )
        self.create_subscription(
            String, self.keyboard_command_topic, self._on_keyboard_command, 10
        )
        self.create_subscription(
            String, self.robot_command_topic, self._on_robot_command, 10
        )

        if HAVE_UR_DASHBOARD and RobotMode is not None and SafetyMode is not None:
            self.create_subscription(RobotMode, self.robot_mode_topic, self._on_robot_mode, 10)
            self.create_subscription(
                SafetyMode, self.safety_mode_topic, self._on_safety_mode, 10
            )
        else:
            self.get_logger().warn(
                "ur_dashboard_msgs is unavailable; robot_mode/safety_mode monitoring disabled."
            )
            self._emit_event(
                "ur_dashboard_msgs unavailable; safety mode telemetry is disabled."
            )

        self.diagnostics_timer = self.create_timer(
            1.0 / self.diagnostics_rate_hz, self._publish_diagnostics
        )
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_interval_s, self._publish_heartbeats
        )

        self._emit_event("HoloAssist runtime observability started.")

    def _mark_rx(self, key: str) -> None:
        now = time.monotonic()
        self._last_seen[key] = now
        stream = self._rate_streams[key]
        stream.append(now)
        cutoff = now - self.rate_window_s
        while stream and stream[0] < cutoff:
            stream.popleft()

    def _rate_hz(self, key: str) -> float:
        stream = self._rate_streams[key]
        if len(stream) < 2:
            return 0.0
        duration = stream[-1] - stream[0]
        if duration <= 0.0:
            return 0.0
        return (len(stream) - 1) / duration

    def _age_s(self, key: str) -> float:
        now = time.monotonic()
        seen = self._last_seen.get(key)
        if seen is None:
            return float("inf")
        return now - seen

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        self._mark_rx("pointcloud")
        self._last_pointcloud_points = int(msg.width * msg.height)

    def _on_obstacle_marker(self, msg: Marker) -> None:
        self._mark_rx("obstacle")
        self._obstacle_active = msg.action not in (Marker.DELETE, Marker.DELETEALL)

    def _on_joint_states(self, msg: JointState) -> None:
        self._mark_rx("joint_states")
        self._last_joint_count = len(msg.name)

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self._mark_rx("target_pose")
        stamp_s = self._stamp_to_wall_time_s(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp_s is not None:
            self._last_target_transport_latency_ms = max(0.0, (time.time() - stamp_s) * 1000.0)

    def _on_twist(self, msg: TwistStamped) -> None:
        self._mark_rx("twist_cmd")
        stamp_s = self._stamp_to_wall_time_s(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp_s is not None:
            self._last_twist_transport_latency_ms = max(0.0, (time.time() - stamp_s) * 1000.0)

    def _on_unity_map_loaded(self, msg: Bool) -> None:
        self._mark_rx("unity_map_loaded")
        changed = self._unity_map_loaded is None or self._unity_map_loaded != msg.data
        self._unity_map_loaded = msg.data
        if changed:
            state = "loaded" if msg.data else "not_loaded"
            self._emit_event(f"Unity map state changed: {state}")

    def _on_keyboard_command(self, msg: String) -> None:
        self._mark_rx("keyboard_command")
        if msg.data != self._last_keyboard_command:
            self._last_keyboard_command = msg.data
            self._emit_event(f"Keyboard teleop: {msg.data}")

    def _on_robot_command(self, msg: String) -> None:
        self._mark_rx("robot_command")
        if msg.data != self._last_robot_command:
            self._last_robot_command = msg.data
            self._emit_event(f"Robot command status: {msg.data}")

    def _on_robot_mode(self, msg: RobotMode) -> None:
        prev = self._robot_mode_value
        self._robot_mode_value = int(msg.mode)
        if prev is None or prev != self._robot_mode_value:
            self._emit_event(f"Robot mode changed: {self._robot_mode_value}")

    def _on_safety_mode(self, msg: SafetyMode) -> None:
        prev = self._safety_mode_value
        self._safety_mode_value = int(msg.mode)
        if prev is None or prev != self._safety_mode_value:
            self._emit_event(f"Safety mode changed: {self._safety_mode_value}")

    @staticmethod
    def _stamp_to_wall_time_s(sec: int, nanosec: int) -> Optional[float]:
        if sec == 0 and nanosec == 0:
            return None
        return float(sec) + float(nanosec) * 1e-9

    @staticmethod
    def _read_mem_available_mb() -> Optional[float]:
        try:
            with open("/proc/meminfo", "r", encoding="utf-8") as handle:
                for line in handle:
                    if line.startswith("MemAvailable:"):
                        parts = line.split()
                        kb = float(parts[1])
                        return kb / 1024.0
        except Exception:
            return None
        return None

    @staticmethod
    def _probe_tcp(host: str, port: int, timeout_s: float) -> Tuple[bool, Optional[float]]:
        start = time.perf_counter()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout_s)
        try:
            sock.connect((host, port))
            latency_ms = (time.perf_counter() - start) * 1000.0
            return True, latency_ms
        except Exception:
            return False, None
        finally:
            sock.close()

    def _make_diag_status(
        self, name: str, level: int, message: str, values: Dict[str, object]
    ) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = "holoassist"
        status.level = level
        status.message = message
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        return status

    def _publish_state(self, pub, value: str) -> None:
        msg = String()
        msg.data = value
        pub.publish(msg)

    def _publish_metric(self, pub, value: Optional[float]) -> None:
        if value is None:
            return
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)

    def _emit_event(self, text: str) -> None:
        now = time.monotonic()
        if text == self._last_event_text and (now - self._last_event_time) < 0.5:
            return
        stamp = time.strftime("%H:%M:%S")
        msg = String()
        msg.data = f"[{stamp}] {text}"
        self.events_pub.publish(msg)
        self._last_event_text = text
        self._last_event_time = now

    def _report_diag_transition(self, status: DiagnosticStatus) -> None:
        prev = self._last_diag_levels.get(status.name)
        if prev is None:
            self._last_diag_levels[status.name] = status.level
            return
        if prev != status.level:
            level_name = {
                DiagnosticStatus.OK: "OK",
                DiagnosticStatus.WARN: "WARN",
                DiagnosticStatus.ERROR: "ERROR",
                DiagnosticStatus.STALE: "STALE",
            }.get(status.level, str(status.level))
            self._emit_event(f"{status.name} -> {level_name}: {status.message}")
            self._last_diag_levels[status.name] = status.level

    def _publish_diagnostics(self) -> None:
        pointcloud_age = self._age_s("pointcloud")
        debug_image_age = self._age_s("debug_image")
        joint_age = self._age_s("joint_states")
        target_age = self._age_s("target_pose")
        twist_age = self._age_s("twist_cmd")
        clicked_age = self._age_s("clicked_point")
        planner_marker_age = self._age_s("planner_marker")

        pointcloud_hz = self._rate_hz("pointcloud")
        debug_hz = self._rate_hz("debug_image")
        joint_hz = self._rate_hz("joint_states")

        perception_level = DiagnosticStatus.OK
        perception_message = "perception streams healthy"
        if pointcloud_age > self.stale_timeout_s * 3.0 and debug_image_age > self.stale_timeout_s * 3.0:
            perception_level = DiagnosticStatus.ERROR
            perception_message = "pointcloud and debug image are stale"
        elif pointcloud_age > self.stale_timeout_s or debug_image_age > self.stale_timeout_s:
            perception_level = DiagnosticStatus.WARN
            perception_message = "a perception stream is stale"

        teleop_level = DiagnosticStatus.OK
        teleop_message = "teleop command streams active"
        if target_age > self.stale_timeout_s * 3.0 and clicked_age > self.stale_timeout_s * 3.0:
            teleop_level = DiagnosticStatus.WARN
            teleop_message = "teleop goal streams are idle"
        if twist_age > self.stale_timeout_s * 3.0:
            teleop_level = max(teleop_level, DiagnosticStatus.WARN)
            teleop_message = "twist command stream is stale"

        planner_level = DiagnosticStatus.OK
        planner_message = "planner marker stream active"
        if planner_marker_age > self.stale_timeout_s * 3.0 and clicked_age > self.stale_timeout_s * 3.0:
            planner_level = DiagnosticStatus.WARN
            planner_message = "planner marker stream is idle"

        safety_level = DiagnosticStatus.OK
        safety_message = "robot safety state nominal"
        if joint_age > self.stale_timeout_s * 3.0:
            safety_level = DiagnosticStatus.ERROR
            safety_message = "joint_states stream is stale"

        if HAVE_UR_DASHBOARD and SafetyMode is not None and self._safety_mode_value is not None:
            safe_modes = {int(SafetyMode.NORMAL), int(SafetyMode.REDUCED)}
            if self._safety_mode_value not in safe_modes:
                safety_level = DiagnosticStatus.ERROR
                safety_message = f"safety mode {self._safety_mode_value} is not NORMAL/REDUCED"

        unity_ok, unity_latency_ms = self._probe_tcp(
            self.unity_tcp_host, self.unity_tcp_port, self.unity_probe_timeout_s
        )
        foxglove_ok, foxglove_latency_ms = self._probe_tcp(
            self.foxglove_bridge_host,
            self.foxglove_bridge_port,
            self.foxglove_probe_timeout_s,
        )

        network_level = DiagnosticStatus.OK
        network_message = "bridge endpoints reachable"
        if not unity_ok or not foxglove_ok:
            network_level = DiagnosticStatus.WARN
            network_message = "unity or foxglove endpoint is not reachable"

        load_1, load_5, load_15 = os.getloadavg()
        mem_available_mb = self._read_mem_available_mb()
        system_level = DiagnosticStatus.OK
        system_message = "system metrics nominal"
        if mem_available_mb is not None and mem_available_mb < 300.0:
            system_level = DiagnosticStatus.WARN
            system_message = "available memory is low"

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status = [
            self._make_diag_status(
                "holoassist/perception",
                perception_level,
                perception_message,
                {
                    "debug_image_hz": f"{debug_hz:.2f}",
                    "pointcloud_hz": f"{pointcloud_hz:.2f}",
                    "bbox_hz": f"{self._rate_hz('bbox'):.2f}",
                    "obstacle_hz": f"{self._rate_hz('obstacle'):.2f}",
                    "debug_image_age_s": f"{debug_image_age:.2f}",
                    "pointcloud_age_s": f"{pointcloud_age:.2f}",
                    "obstacle_active": self._obstacle_active,
                    "pointcloud_points": self._last_pointcloud_points,
                },
            ),
            self._make_diag_status(
                "holoassist/teleop",
                teleop_level,
                teleop_message,
                {
                    "target_pose_hz": f"{self._rate_hz('target_pose'):.2f}",
                    "twist_hz": f"{self._rate_hz('twist_cmd'):.2f}",
                    "clicked_point_hz": f"{self._rate_hz('clicked_point'):.2f}",
                    "target_age_s": f"{target_age:.2f}",
                    "twist_age_s": f"{twist_age:.2f}",
                    "keyboard_command_age_s": f"{self._age_s('keyboard_command'):.2f}",
                },
            ),
            self._make_diag_status(
                "holoassist/planning",
                planner_level,
                planner_message,
                {
                    "planner_marker_hz": f"{self._rate_hz('planner_marker'):.2f}",
                    "planner_marker_age_s": f"{planner_marker_age:.2f}",
                    "clicked_point_age_s": f"{clicked_age:.2f}",
                },
            ),
            self._make_diag_status(
                "holoassist/safety",
                safety_level,
                safety_message,
                {
                    "joint_states_hz": f"{joint_hz:.2f}",
                    "joint_states_age_s": f"{joint_age:.2f}",
                    "joint_count": self._last_joint_count,
                    "robot_mode": self._robot_mode_value,
                    "safety_mode": self._safety_mode_value,
                    "unity_map_loaded": self._unity_map_loaded,
                },
            ),
            self._make_diag_status(
                "holoassist/network",
                network_level,
                network_message,
                {
                    "unity_tcp_host": self.unity_tcp_host,
                    "unity_tcp_port": self.unity_tcp_port,
                    "unity_tcp_reachable": unity_ok,
                    "unity_tcp_latency_ms": f"{unity_latency_ms:.2f}" if unity_latency_ms is not None else "nan",
                    "foxglove_host": self.foxglove_bridge_host,
                    "foxglove_port": self.foxglove_bridge_port,
                    "foxglove_reachable": foxglove_ok,
                    "foxglove_latency_ms": f"{foxglove_latency_ms:.2f}" if foxglove_latency_ms is not None else "nan",
                },
            ),
            self._make_diag_status(
                "holoassist/system",
                system_level,
                system_message,
                {
                    "load_avg_1m": f"{load_1:.2f}",
                    "load_avg_5m": f"{load_5:.2f}",
                    "load_avg_15m": f"{load_15:.2f}",
                    "mem_available_mb": f"{mem_available_mb:.2f}" if mem_available_mb is not None else "nan",
                },
            ),
        ]

        for status in diag.status:
            self._report_diag_transition(status)

        self.diagnostics_pub.publish(diag)

        teleop_state = "ACTIVE" if min(target_age, clicked_age) < self.stale_timeout_s else "IDLE"
        planner_state = (
            "ACTIVE"
            if min(planner_marker_age, clicked_age) < self.stale_timeout_s
            else "IDLE"
        )
        safety_state = "OK"
        if safety_level == DiagnosticStatus.ERROR:
            safety_state = "ERROR"
        elif safety_level == DiagnosticStatus.WARN:
            safety_state = "WARN"

        runtime_summary = {
            "teleop_state": teleop_state,
            "planner_state": planner_state,
            "safety_state": safety_state,
            "unity_tcp_reachable": unity_ok,
            "foxglove_reachable": foxglove_ok,
            "pointcloud_hz": round(pointcloud_hz, 2),
            "joint_states_hz": round(joint_hz, 2),
        }

        self._publish_state(self.teleop_state_pub, teleop_state)
        self._publish_state(self.planner_state_pub, planner_state)
        self._publish_state(self.safety_state_pub, safety_state)
        self._publish_state(self.runtime_state_pub, json.dumps(runtime_summary))

        self._publish_metric(self.joint_rate_pub, joint_hz)
        self._publish_metric(self.pointcloud_rate_pub, pointcloud_hz)
        self._publish_metric(self.debug_image_rate_pub, debug_hz)
        self._publish_metric(self.target_latency_pub, self._last_target_transport_latency_ms)
        self._publish_metric(self.twist_latency_pub, self._last_twist_transport_latency_ms)
        self._publish_metric(self.unity_tcp_latency_pub, unity_latency_ms)
        self._publish_metric(self.foxglove_tcp_latency_pub, foxglove_latency_ms)

    def _publish_heartbeats(self) -> None:
        now_s = time.time()
        alive = {
            "xr_interface": min(self._age_s("target_pose"), self._age_s("keyboard_command")) < self.stale_timeout_s * 2.0,
            "perception": min(self._age_s("pointcloud"), self._age_s("debug_image")) < self.stale_timeout_s * 2.0,
            "planning": min(self._age_s("clicked_point"), self._age_s("planner_marker")) < self.stale_timeout_s * 2.0,
            "control": min(self._age_s("twist_cmd"), self._age_s("joint_states")) < self.stale_timeout_s * 2.0,
            "rviz": min(self._age_s("clicked_point"), self._age_s("planner_marker")) < self.stale_timeout_s * 3.0,
            "unity_bridge": self._age_s("unity_map_loaded") < self.stale_timeout_s * 4.0,
            "robot_bringup": self._age_s("joint_states") < self.stale_timeout_s * 2.0,
        }

        for name, pub in self._heartbeat_pubs.items():
            if not alive.get(name, False):
                continue
            msg = String()
            msg.data = f"{name}:{now_s:.3f}"
            pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RuntimeObservabilityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
