"""
ROS 2 interface for HoloAssist Dashboard.
Handles e-stop, velocity publishing, joint state monitoring, perception topics,
and controller switching.

Runs rclpy in a background thread; all public methods are thread-safe.
"""

import json
import os
import re
import subprocess
import threading
import time
from collections import deque
from datetime import datetime
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
    from std_msgs.msg import Float64MultiArray, Float32MultiArray, Float32, Bool, String
    from sensor_msgs.msg import JointState, Image, CompressedImage, PointCloud2
    from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped
    from visualization_msgs.msg import Marker
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

try:
    from holo_assist_depth_tracker_sim_interfaces.srv import PickCubeToBin
    PICK_CUBE_SERVICE_AVAILABLE = True
except ImportError:
    PICK_CUBE_SERVICE_AVAILABLE = False


class RobotState(Enum):
    DISCONNECTED = auto()
    RUNNING = auto()
    ESTOPPED = auto()
    RESUMING = auto()


class OperatingMode(Enum):
    TELEOP = auto()
    MOVEIT = auto()


@dataclass
class TopicStatus:
    """Tracks a single topic's receive rate and freshness."""
    rx_count: int = 0
    last_rx_time: float = 0.0
    hz: float = 0.0


@dataclass
class DashboardStatus:
    """Thread-safe snapshot of robot/ROS state."""
    robot_state: RobotState = RobotState.DISCONNECTED
    ros_connected: bool = False
    controller_active: bool = False
    # Joint state
    joint_names: list = field(default_factory=list)
    joint_positions: list = field(default_factory=list)
    joint_velocities: list = field(default_factory=list)
    last_joint_time: float = 0.0
    joint_hz: float = 0.0
    # E-stop
    events: list = field(default_factory=list)
    estop_zero_count: int = 0
    # Perception topic rates
    topic_rates: dict = field(default_factory=dict)  # name -> TopicStatus
    # Camera image (latest debug image as raw bytes, or None)
    camera_jpeg: Optional[bytes] = None
    camera_width: int = 0
    camera_height: int = 0
    # Headset image (JPEG compressed from Quest 3)
    headset_jpeg: Optional[bytes] = None
    # EEF pose from TF or target
    eef_pose: Optional[dict] = None  # {x, y, z, qx, qy, qz, qw}
    target_pose: Optional[dict] = None
    # Latency tracking
    last_target_age_s: float = -1.0
    last_twist_age_s: float = -1.0
    # Unity integration
    unity_map_loaded: Optional[bool] = None
    # Session metrics from Unity SessionLogger
    session_info: dict = field(default_factory=dict)
    # Gripper
    gripper_value: float = 0.0  # 0.0=open, 1.0=closed
    gripper_grips: int = 0
    # Collision
    collision_scale: float = 1.0
    collision_blocked: bool = False
    collision_events: int = 0
    # EE lock
    ee_locked: bool = False
    ee_lock_count: int = 0
    # Operating mode
    operating_mode: str = "TELEOP"
    # MoveIt pick/place
    pick_service_ready: bool = False
    pick_request_pending: bool = False
    last_pick_cube: str = ""
    last_pick_success: Optional[bool] = None
    last_pick_message: str = ""
    pick_place_status: str = ""
    pick_place_status_raw: str = ""
    pick_place_status_time: float = 0.0
    pick_place_status_lines: list = field(default_factory=list)
    pick_place_block_id: str = ""
    pick_place_destination: str = ""
    pick_place_step: str = ""
    pick_place_step_label: str = ""
    pick_place_step_index: int = 0
    pick_place_step_total: int = 0
    pick_place_state: str = ""
    pick_place_error: str = ""
    pick_place_error_detail: str = ""
    # Automated hand-eye calibration
    calibration_ready: bool = False
    calibration_running: bool = False
    calibration_state: str = ""
    calibration_message: str = ""
    calibration_sample_count: int = 0
    calibration_pose_index: int = 0
    calibration_pose_total: int = 0
    calibration_computed: bool = False
    calibration_result: dict = field(default_factory=dict)
    calibration_latest_path: str = ""
    calibration_archive_path: str = ""
    calibration_error: str = ""
    calibration_marker_frame: str = "tag36h11:1"
    camera_type: str = ""   # "realsense", "webcam", "brio", or ""
    headset_type: str = ""  # "quest2", "quest3", or ""
    # Rolling graph data (downsampled for display)
    velocity_history: list = field(default_factory=list)   # [(t, [v0..v5])]
    rate_history: list = field(default_factory=list)        # [(t, [joint%, vel%, headset%])]
    latency_history: list = field(default_factory=list)     # [(t, [joint_age_ms, vel_age_ms, cmd_interval_ms])]


# All topic names matching John's defaults
TOPIC_DEFAULTS = {
    "debug_image": "/holo_assist_depth_tracker/debug_image",
    "headset_image": "/headset/image_compressed",
    "bbox": "/holo_assist_depth_tracker/bbox",
    "pointcloud": "/holo_assist_depth_tracker/pointcloud",
    "obstacle": "/holo_assist_depth_tracker/obstacle_marker",
    "joint_states": "/joint_states",
    "target_pose": "/servo_target_pose",
    "twist_cmd": "/servo_node/delta_twist_cmds",
    "clicked_point": "/clicked_point",
    "unity_map_loaded": "/unity/map_loaded",
    "velocity_cmd": "/forward_velocity_controller/commands",
}

PICK_PLACE_MODE_TOPIC = "/pick_place/mode"
PICK_PLACE_STATUS_TOPIC = "/pick_place/status"
PICK_CUBE_SERVICE = "/holoassist/pick_cube_to_bin"
CALIBRATION_COMMAND_TOPIC = "/holoassist/calibration/command"
CALIBRATION_STATUS_TOPIC = "/holoassist/calibration/status"


class RosInterface:
    """
    Manages all ROS 2 communication for the dashboard.

    Usage:
        ros = RosInterface()
        ros.start()
        ros.emergency_stop()
        ros.resume()
        status = ros.get_status()
        ros.shutdown()
    """

    NUM_JOINTS = 6
    SAFETY_PUBLISH_HZ = 50
    RATE_WINDOW_S = 3.0  # rolling window for Hz calculation

    def __init__(self):
        self._lock = threading.Lock()
        self._status = DashboardStatus()
        self._events: list[tuple[float, str]] = []
        self._node = None
        self._spin_thread = None
        self._safety_thread = None
        self._safety_stop = threading.Event()
        self._running = False

        # Per-topic rate tracking: name -> deque of timestamps
        self._rate_streams: dict[str, deque] = {
            name: deque() for name in TOPIC_DEFAULTS
        }

        # Rolling graph data (downsampled)
        self._velocity_history: deque = deque(maxlen=300)   # 10Hz * 30s
        self._rate_history: deque = deque(maxlen=120)       # 2Hz * 60s
        self._latency_history: deque = deque(maxlen=300)    # 10Hz * 30s
        self._session_info: dict = {}
        self._pick_place_status_lines: deque = deque(maxlen=40)
        self._last_vel_cmd_time: float = 0.0                # timestamp of last velocity_cmd
        self._operating_mode: OperatingMode = OperatingMode.TELEOP
        self._fake_hardware: bool = False  # set by _check_controller_status on startup
        self._pick_cube_client = None
        self._camera_reconfigure_pub = None
        self._calibration_command_pub = None

    def start(self):
        """Initialize rclpy and start spinning in a background thread."""
        if not ROS_AVAILABLE:
            self._add_event("ROS 2 (rclpy) not available - running in offline mode")
            return False

        try:
            rclpy.init()
        except RuntimeError:
            pass

        self._node = rclpy.create_node("holoassist_dashboard")

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ──
        self._vel_pub = self._node.create_publisher(
            Float64MultiArray, TOPIC_DEFAULTS["velocity_cmd"], 10
        )
        self._pick_place_mode_pub = self._node.create_publisher(
            String, PICK_PLACE_MODE_TOPIC, 10
        )
        self._calibration_command_pub = self._node.create_publisher(
            String, CALIBRATION_COMMAND_TOPIC, 10
        )
        if PICK_CUBE_SERVICE_AVAILABLE:
            self._pick_cube_client = self._node.create_client(
                PickCubeToBin, PICK_CUBE_SERVICE
            )

        # ── Subscribers ──

        # Joint states
        self._node.create_subscription(
            JointState, TOPIC_DEFAULTS["joint_states"],
            self._joint_state_cb, qos_profile_sensor_data,
        )

        # Perception: debug image (camera view)
        self._node.create_subscription(
            Image, TOPIC_DEFAULTS["debug_image"],
            self._debug_image_cb, qos_profile_sensor_data,
        )

        # Headset: compressed image from Quest 3
        self._node.create_subscription(
            CompressedImage, TOPIC_DEFAULTS["headset_image"],
            self._headset_image_cb, qos_profile_sensor_data,
        )

        # Perception: bounding boxes
        self._node.create_subscription(
            Float32MultiArray, TOPIC_DEFAULTS["bbox"],
            lambda msg: self._tick_rate("bbox"), 10,
        )

        # Perception: point cloud
        self._node.create_subscription(
            PointCloud2, TOPIC_DEFAULTS["pointcloud"],
            lambda msg: self._tick_rate("pointcloud"), reliable_qos,
        )

        # Perception: obstacle marker
        self._node.create_subscription(
            Marker, TOPIC_DEFAULTS["obstacle"],
            lambda msg: self._tick_rate("obstacle"), reliable_qos,
        )

        # Robot commands: target pose
        self._node.create_subscription(
            PoseStamped, TOPIC_DEFAULTS["target_pose"],
            self._target_pose_cb, qos_profile_sensor_data,
        )

        # Robot commands: twist
        self._node.create_subscription(
            TwistStamped, TOPIC_DEFAULTS["twist_cmd"],
            self._twist_cb, qos_profile_sensor_data,
        )

        # Robot commands: clicked point
        self._node.create_subscription(
            PointStamped, TOPIC_DEFAULTS["clicked_point"],
            lambda msg: self._tick_rate("clicked_point"), qos_profile_sensor_data,
        )

        # Unity: map loaded signal
        self._node.create_subscription(
            Bool, TOPIC_DEFAULTS["unity_map_loaded"],
            self._unity_map_cb, 10,
        )

        # Velocity commands (from Unity — track rate + timestamp for latency)
        self._node.create_subscription(
            Float64MultiArray, TOPIC_DEFAULTS["velocity_cmd"],
            self._velocity_cmd_cb, qos_profile_sensor_data,
        )

        # Session status (from Unity SessionLogger)
        self._node.create_subscription(
            String, "/session/status",
            self._session_status_cb, 10,
        )

        # Session events (from Unity SessionLogger)
        self._node.create_subscription(
            String, "/session/events",
            self._session_event_cb, 10,
        )

        # Pick/place sequencer status
        self._node.create_subscription(
            String, PICK_PLACE_STATUS_TOPIC,
            self._pick_place_status_cb, 10,
        )
        self._node.create_subscription(
            String, CALIBRATION_STATUS_TOPIC,
            self._calibration_status_cb, 10,
        )

        # Camera type (published by launch/camera node, or auto-detected)
        self._node.create_subscription(
            String, "/holo_assist/camera_type",
            self._camera_type_cb, 10,
        )

        # Headset type (published by headset app)
        self._node.create_subscription(
            String, "/headset/device_type",
            self._headset_type_cb, 10,
        )

        # Camera reconfigure publisher
        self._camera_reconfigure_pub = self._node.create_publisher(
            String, "/holo_assist/camera_reconfigure", 10,
        )

        # Sampling timers for rolling graph data
        self._node.create_timer(0.1, self._sample_velocities)   # 10Hz
        self._node.create_timer(0.5, self._sample_rates)        # 2Hz
        self._node.create_timer(2.0, self._detect_camera_type)  # 0.5Hz node scan

        self._running = True
        with self._lock:
            self._status.ros_connected = True
            self._status.robot_state = RobotState.RUNNING
        self._add_event("ROS 2 node started")
        self._add_event(f"Subscribing to {len(TOPIC_DEFAULTS)} topics")
        if self._pick_cube_client is not None:
            self._add_event(f"Pick cube service client ready: {PICK_CUBE_SERVICE}")
        else:
            self._add_event("Pick cube service type unavailable")

        threading.Thread(target=self._check_controller_status, daemon=True).start()

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        return True

    def _spin(self):
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.05)
            except Exception:
                break
        with self._lock:
            self._status.ros_connected = False

    # ── Rate tracking helper ───────────────────────────────────────

    def _tick_rate(self, name: str):
        """Record a message arrival for Hz calculation."""
        now = time.time()
        stream = self._rate_streams.get(name)
        if stream is not None:
            stream.append(now)
            cutoff = now - self.RATE_WINDOW_S
            while stream and stream[0] < cutoff:
                stream.popleft()

    def _get_hz(self, name: str) -> float:
        stream = self._rate_streams.get(name)
        if not stream or len(stream) < 2:
            return 0.0
        duration = stream[-1] - stream[0]
        return (len(stream) - 1) / duration if duration > 0 else 0.0

    # ── Callbacks ──────────────────────────────────────────────────

    def _joint_state_cb(self, msg):
        now = time.time()
        self._tick_rate("joint_states")
        with self._lock:
            self._status.joint_names = list(msg.name)
            self._status.joint_positions = list(msg.position)
            self._status.joint_velocities = list(msg.velocity) if msg.velocity else [0.0] * len(msg.position)
            self._status.last_joint_time = now
            self._status.joint_hz = self._get_hz("joint_states")
            # Extract gripper finger_width (metres) → 0.0=open, 1.0=closed
            for i, name in enumerate(msg.name):
                if name == "finger_width":
                    width_m = msg.position[i]
                    self._status.gripper_value = 1.0 - min(width_m / 0.11, 1.0)
                    break

    def _debug_image_cb(self, msg):
        """Convert ROS Image to JPEG bytes for display."""
        self._tick_rate("debug_image")
        try:
            # Fast path: if already compressed, store directly
            # Otherwise convert raw image to simple format for display
            w, h = msg.width, msg.height
            encoding = msg.encoding
            data = bytes(msg.data)

            with self._lock:
                self._status.camera_width = w
                self._status.camera_height = h
                # Store raw RGB/BGR data — GUI will handle conversion
                self._status.camera_jpeg = data
                self._status._camera_encoding = encoding
        except Exception:
            pass

    def _headset_image_cb(self, msg):
        """CompressedImage from Quest 3 — already JPEG encoded."""
        self._tick_rate("headset_image")
        with self._lock:
            self._status.headset_jpeg = bytes(msg.data)

    def _target_pose_cb(self, msg):
        self._tick_rate("target_pose")
        now = time.time()
        p = msg.pose.position
        o = msg.pose.orientation
        with self._lock:
            self._status.target_pose = {
                "x": p.x, "y": p.y, "z": p.z,
                "qx": o.x, "qy": o.y, "qz": o.z, "qw": o.w,
            }
            # Compute age from header stamp
            if msg.header.stamp.sec > 0:
                stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self._status.last_target_age_s = now - stamp_s

    def _twist_cb(self, msg):
        self._tick_rate("twist_cmd")
        now = time.time()
        with self._lock:
            if msg.header.stamp.sec > 0:
                stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self._status.last_twist_age_s = now - stamp_s

    def _unity_map_cb(self, msg):
        with self._lock:
            self._status.unity_map_loaded = msg.data
        state = "loaded" if msg.data else "not loaded"
        self._add_event(f"Unity map: {state}")

    def _velocity_cmd_cb(self, msg):
        self._tick_rate("velocity_cmd")
        self._last_vel_cmd_time = time.time()

    def _session_status_cb(self, msg):
        try:
            info = json.loads(msg.data)
            with self._lock:
                self._session_info = info
        except (json.JSONDecodeError, Exception):
            pass

    def _session_event_cb(self, msg):
        try:
            evt = json.loads(msg.data)
            self._add_event(f"[Unity] {evt.get('type', '?')}: {evt.get('detail', '')}")
        except (json.JSONDecodeError, Exception):
            pass

    def _camera_type_cb(self, msg):
        with self._lock:
            self._status.camera_type = msg.data.strip().lower()

    def _headset_type_cb(self, msg):
        with self._lock:
            self._status.headset_type = msg.data.strip().lower()

    def _detect_camera_type(self):
        """Auto-detect camera type from running node names (runs every 2s)."""
        if not ROS_AVAILABLE or self._node is None:
            return
        with self._lock:
            if self._status.camera_type:
                return  # already known from explicit topic
        try:
            node_names = self._node.get_node_names()
        except Exception:
            return
        cam = ""
        for name in node_names:
            n = name.lower()
            if "realsense" in n or "rs_camera" in n:
                cam = "realsense"
                break
            if "brio" in n:
                cam = "brio"
                break
            if "webcam" in n:
                cam = "webcam"
                break
        if cam:
            with self._lock:
                self._status.camera_type = cam

    def _pick_place_status_cb(self, msg):
        now = time.time()
        raw = msg.data
        line = raw
        block_id = ""
        destination = ""
        step = ""
        step_label = ""
        step_index = 0
        step_total = 0
        state = ""
        error = ""
        error_detail = ""
        try:
            payload = json.loads(raw)
            if isinstance(payload, dict):
                message = str(payload.get("message", "")).strip()
                block_id = str(payload.get("block_id", "")).strip()
                destination = str(payload.get("destination", "")).strip()
                step = str(payload.get("step", "")).strip()
                step_label = str(payload.get("step_label", "")).strip()
                state = str(payload.get("state", "")).strip()
                error = str(payload.get("error", "")).strip()
                error_detail = str(payload.get("error_detail", "")).strip()
                try:
                    step_index = int(payload.get("step_index", 0) or 0)
                    step_total = int(payload.get("step_total", 0) or 0)
                except (TypeError, ValueError):
                    step_index = 0
                    step_total = 0
                extras = [
                    f"{key}={value}"
                    for key, value in payload.items()
                    if key
                    not in {
                        "message",
                        "stamp_sec",
                        "level",
                        "block_id",
                        "destination",
                        "step",
                        "step_label",
                        "step_index",
                        "step_total",
                        "state",
                        "error",
                        "error_detail",
                    }
                ]
                line = message or raw
                if extras:
                    line = f"{line} | {' '.join(extras)}"
        except (json.JSONDecodeError, Exception):
            pass

        timestamp = datetime.fromtimestamp(now).strftime("%H:%M:%S")
        display_line = f"[{timestamp}] {line}"
        with self._lock:
            self._status.pick_place_status = line
            self._status.pick_place_status_raw = raw
            self._status.pick_place_status_time = now
            self._pick_place_status_lines.append(display_line)
            self._status.pick_place_block_id = block_id
            self._status.pick_place_destination = destination
            self._status.pick_place_step = step
            self._status.pick_place_step_label = step_label
            self._status.pick_place_step_index = step_index
            self._status.pick_place_step_total = step_total
            self._status.pick_place_state = state
            self._status.pick_place_error = error
            self._status.pick_place_error_detail = error_detail

    def _calibration_status_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except (json.JSONDecodeError, Exception):
            return
        with self._lock:
            self._status.calibration_ready = bool(payload.get("ready", False))
            self._status.calibration_running = bool(payload.get("running", False))
            self._status.calibration_state = str(payload.get("state", ""))
            self._status.calibration_message = str(payload.get("message", ""))
            self._status.calibration_sample_count = int(payload.get("sample_count", 0) or 0)
            self._status.calibration_pose_index = int(payload.get("pose_index", 0) or 0)
            self._status.calibration_pose_total = int(payload.get("pose_total", 0) or 0)
            self._status.calibration_computed = bool(payload.get("computed", False))
            self._status.calibration_result = dict(payload.get("result", {}) or {})
            self._status.calibration_latest_path = str(payload.get("latest_path", ""))
            self._status.calibration_archive_path = str(payload.get("archive_path", ""))
            self._status.calibration_error = str(payload.get("error", ""))
            self._status.calibration_marker_frame = str(
                payload.get("marker_frame", "tag36h11:1")
            )

    def _sample_velocities(self):
        """Downsample joint velocities + latency to 10Hz for rolling graphs."""
        now = time.time()
        with self._lock:
            vels = list(self._status.joint_velocities) if self._status.joint_velocities else None
            joint_time = self._status.last_joint_time
        if vels and len(vels) == 6:
            self._velocity_history.append((now, vels))

        # Latency: joint state age, velocity cmd age, cmd interval
        joint_age_ms = (now - joint_time) * 1000 if joint_time > 0 else -1
        vel_age_ms = (now - self._last_vel_cmd_time) * 1000 if self._last_vel_cmd_time > 0 else -1
        vel_hz = self._get_hz("velocity_cmd")
        cmd_interval_ms = (1000.0 / vel_hz) if vel_hz > 0 else -1
        self._latency_history.append((now, [joint_age_ms, vel_age_ms, cmd_interval_ms]))

    def _sample_rates(self):
        """Sample topic health as % of expected rate, for rolling graph."""
        expected = [("joint_states", 500), ("velocity_cmd", 50), ("headset_image", 15)]
        pcts = []
        for topic, exp_hz in expected:
            hz = self._get_hz(topic)
            pcts.append(min(hz / exp_hz * 100, 120) if exp_hz > 0 else 0.0)
        self._rate_history.append((time.time(), pcts))

    # ── E-STOP ──────────────────────────────────────────────────────

    def emergency_stop(self):
        with self._lock:
            if self._status.robot_state == RobotState.ESTOPPED:
                return
            self._status.robot_state = RobotState.ESTOPPED
            self._status.estop_zero_count = 0

        self._add_event("EMERGENCY STOP TRIGGERED")
        self._publish_pick_place_mode("stop")
        self._publish_zeros(count=10)

        self._safety_stop.clear()
        self._safety_thread = threading.Thread(target=self._safety_publish_loop, daemon=True)
        self._safety_thread.start()

        threading.Thread(target=self._deactivate_controller, daemon=True).start()

    def _publish_zeros(self, count=1):
        if not ROS_AVAILABLE or self._node is None:
            return
        msg = Float64MultiArray()
        msg.data = [0.0] * self.NUM_JOINTS
        for _ in range(count):
            self._vel_pub.publish(msg)
        with self._lock:
            self._status.estop_zero_count += count

    def _safety_publish_loop(self):
        interval = 1.0 / self.SAFETY_PUBLISH_HZ
        while not self._safety_stop.is_set():
            self._publish_zeros(count=1)
            self._safety_stop.wait(timeout=interval)

    def _switch_controllers(self, activate: list, deactivate: list) -> tuple:
        """Switch controllers via ros2 service call. Returns (success: bool, error: str)."""
        def _fmt(names):
            if not names:
                return "[]"
            return "[" + ", ".join(f"'{n}'" for n in names) + "]"

        yaml_args = (
            f"{{activate_controllers: {_fmt(activate)}, "
            f"deactivate_controllers: {_fmt(deactivate)}, "
            f"strictness: 1}}"
        )
        try:
            result = subprocess.run(
                ["ros2", "service", "call",
                 "/controller_manager/switch_controller",
                 "controller_manager_msgs/srv/SwitchController",
                 yaml_args],
                capture_output=True, text=True, timeout=15,
            )
            if result.returncode != 0:
                return False, result.stderr.strip() or result.stdout.strip()
            if "ok=False" in result.stdout:
                return False, "controller_manager returned ok=False"
            return True, ""
        except subprocess.TimeoutExpired:
            return False, "service call timed out"
        except Exception as e:
            return False, str(e)

    def _deactivate_controller(self):
        with self._lock:
            mode = self._operating_mode

        if self._fake_hardware:
            ok, err = self._switch_controllers([], ["joint_trajectory_controller"])
            if ok:
                self._add_event("Fake HW: joint_trajectory_controller deactivated")
                with self._lock:
                    self._status.controller_active = False
            else:
                self._add_event(f"Fake HW: controller deactivation failed: {err}")
            return

        if mode == OperatingMode.MOVEIT:
            to_deactivate = ["scaled_joint_trajectory_controller", "finger_width_trajectory_controller"]
        else:
            to_deactivate = ["forward_velocity_controller", "finger_width_controller"]
        ok, err = self._switch_controllers([], to_deactivate)
        if ok:
            self._add_event(f"{mode.name} controllers deactivated")
            with self._lock:
                self._status.controller_active = False
        else:
            self._add_event(f"Controller deactivation failed: {err}")

    # ── RESUME ──────────────────────────────────────────────────────

    def resume(self):
        with self._lock:
            if self._status.robot_state != RobotState.ESTOPPED:
                return
            self._status.robot_state = RobotState.RESUMING

        self._add_event("Resuming - reactivating controller...")
        self._safety_stop.set()
        target_mode = self._operating_mode
        threading.Thread(
            target=self._activate_controller,
            args=(target_mode,),
            daemon=True,
        ).start()

    def _activate_controller(self, target_mode: OperatingMode):
        if self._fake_hardware:
            ok, err = self._switch_controllers(["joint_trajectory_controller"], [])
            if ok:
                self._add_event("Fake HW: joint_trajectory_controller reactivated - ROBOT LIVE")
                with self._lock:
                    self._status.robot_state = RobotState.RUNNING
                    self._status.controller_active = True
                    self._status.estop_zero_count = 0
                if target_mode == OperatingMode.MOVEIT:
                    self._publish_pick_place_mode("run")
            else:
                self._add_event(f"Fake HW: controller activation failed: {err}")
                with self._lock:
                    self._status.robot_state = RobotState.ESTOPPED
            return

        if target_mode == OperatingMode.MOVEIT:
            activate = ["scaled_joint_trajectory_controller", "finger_width_trajectory_controller"]
            deactivate = ["forward_velocity_controller", "finger_width_controller"]
            success_message = "MOVEIT controllers reactivated - pick/place live"
        else:
            activate = ["forward_velocity_controller", "finger_width_controller"]
            deactivate = ["scaled_joint_trajectory_controller", "finger_width_trajectory_controller"]
            success_message = "forward_velocity_controller reactivated - ROBOT LIVE"

        ok, err = self._switch_controllers(activate, deactivate)
        if ok:
            self._add_event(success_message)
            with self._lock:
                self._status.robot_state = RobotState.RUNNING
                self._status.controller_active = True
                self._status.estop_zero_count = 0
            if target_mode == OperatingMode.MOVEIT:
                self._publish_pick_place_mode("run")
        else:
            self._add_event(f"Controller activation failed: {err}")
            with self._lock:
                self._status.robot_state = RobotState.ESTOPPED

    # ── STATUS ──────────────────────────────────────────────────────

    def _check_controller_status(self):
        try:
            result = subprocess.run(
                ["ros2", "service", "call",
                 "/controller_manager/list_controllers",
                 "controller_manager_msgs/srv/ListControllers",
                 "{}"],
                capture_output=True, text=True, timeout=10,
            )
            if result.returncode != 0:
                self._add_event(f"Could not check controllers: {result.stderr.strip()}")
                return

            out = result.stdout
            # Fake hardware: both joint_trajectory_controller AND
            # scaled_joint_trajectory_controller are loaded, but
            # forward_velocity_controller is never activated (UR driver
            # spawner does not activate teleop controllers on fake hardware).
            has_fvc = "'forward_velocity_controller'" in out
            has_scaled_active = (
                "name='scaled_joint_trajectory_controller'" in out
                and "label='active'" in out
            )

            if has_scaled_active and not has_fvc:
                # scaled controller active but teleop controllers absent → fake HW
                self._fake_hardware = True
                self._add_event("Fake hardware detected — controller switching disabled")
                with self._lock:
                    self._status.controller_active = True
                return

            m = re.search(
                r"name='forward_velocity_controller'.*?label='(\w+)'",
                out,
                re.DOTALL,
            )
            m_scaled = re.search(
                r"name='scaled_joint_trajectory_controller'.*?label='(\w+)'",
                out,
                re.DOTALL,
            )
            if m:
                fvc_active = m.group(1) == "active"
                with self._lock:
                    self._status.controller_active = fvc_active
                self._add_event(
                    f"forward_velocity_controller is {'active' if fvc_active else 'inactive'}"
                )
                if not fvc_active and m_scaled and m_scaled.group(1) == "active":
                    self._operating_mode = OperatingMode.MOVEIT
                    with self._lock:
                        self._status.controller_active = True
                    self._add_event("Detected MoveIt startup mode (trajectory controllers active)")
            else:
                self._add_event("forward_velocity_controller not found in controller list")
        except Exception as e:
            self._add_event(f"Could not check controllers: {e}")

    def get_status(self) -> DashboardStatus:
        """Return a snapshot of current status. Called from GUI thread."""
        pick_service_ready = False
        if self._pick_cube_client is not None:
            try:
                pick_service_ready = self._pick_cube_client.service_is_ready()
            except Exception:
                pick_service_ready = False

        # Build topic rates snapshot
        topic_rates = {}
        for name in self._rate_streams:
            stream = self._rate_streams[name]
            hz = self._get_hz(name)
            last_time = stream[-1] if stream else 0.0
            topic_rates[name] = TopicStatus(
                rx_count=len(stream), last_rx_time=last_time, hz=hz
            )

        # Snapshot rolling graph data (GIL-safe deque → list)
        vel_hist = list(self._velocity_history)
        rate_hist = list(self._rate_history)
        lat_hist = list(self._latency_history)

        with self._lock:
            session_info_copy = dict(self._session_info)
            status = DashboardStatus(
                robot_state=self._status.robot_state,
                ros_connected=self._status.ros_connected,
                controller_active=self._status.controller_active,
                joint_names=list(self._status.joint_names),
                joint_positions=list(self._status.joint_positions),
                joint_velocities=list(self._status.joint_velocities),
                last_joint_time=self._status.last_joint_time,
                joint_hz=self._status.joint_hz,
                events=list(self._events),
                estop_zero_count=self._status.estop_zero_count,
                topic_rates=topic_rates,
                camera_jpeg=self._status.camera_jpeg,
                camera_width=self._status.camera_width,
                camera_height=self._status.camera_height,
                headset_jpeg=self._status.headset_jpeg,
                eef_pose=self._status.eef_pose.copy() if self._status.eef_pose else None,
                target_pose=self._status.target_pose.copy() if self._status.target_pose else None,
                last_target_age_s=self._status.last_target_age_s,
                last_twist_age_s=self._status.last_twist_age_s,
                unity_map_loaded=self._status.unity_map_loaded,
                session_info=session_info_copy,
                gripper_grips=session_info_copy.get("gripper_grips", 0),
                collision_scale=session_info_copy.get("collision_scale", 1.0),
                collision_blocked=session_info_copy.get("collision_blocked", False),
                collision_events=session_info_copy.get("collision_events", 0),
                ee_locked=session_info_copy.get("ee_locked", False),
                ee_lock_count=session_info_copy.get("ee_lock_count", 0),
                operating_mode=self._operating_mode.name,
                pick_service_ready=pick_service_ready,
                pick_request_pending=self._status.pick_request_pending,
                last_pick_cube=self._status.last_pick_cube,
                last_pick_success=self._status.last_pick_success,
                last_pick_message=self._status.last_pick_message,
                pick_place_status=self._status.pick_place_status,
                pick_place_status_raw=self._status.pick_place_status_raw,
                pick_place_status_time=self._status.pick_place_status_time,
                pick_place_status_lines=list(self._pick_place_status_lines),
                pick_place_block_id=self._status.pick_place_block_id,
                pick_place_destination=self._status.pick_place_destination,
                pick_place_step=self._status.pick_place_step,
                pick_place_step_label=self._status.pick_place_step_label,
                pick_place_step_index=self._status.pick_place_step_index,
                pick_place_step_total=self._status.pick_place_step_total,
                pick_place_state=self._status.pick_place_state,
                pick_place_error=self._status.pick_place_error,
                pick_place_error_detail=self._status.pick_place_error_detail,
                calibration_ready=self._status.calibration_ready,
                calibration_running=self._status.calibration_running,
                calibration_state=self._status.calibration_state,
                calibration_message=self._status.calibration_message,
                calibration_sample_count=self._status.calibration_sample_count,
                calibration_pose_index=self._status.calibration_pose_index,
                calibration_pose_total=self._status.calibration_pose_total,
                calibration_computed=self._status.calibration_computed,
                calibration_result=dict(self._status.calibration_result),
                calibration_latest_path=self._status.calibration_latest_path,
                calibration_archive_path=self._status.calibration_archive_path,
                calibration_error=self._status.calibration_error,
                calibration_marker_frame=self._status.calibration_marker_frame,
                velocity_history=vel_hist,
                rate_history=rate_hist,
                latency_history=lat_hist,
                camera_type=self._status.camera_type,
                headset_type=self._status.headset_type,
            )
        return status

    def _add_event(self, msg: str):
        entry = (time.time(), msg)
        with self._lock:
            self._events.append(entry)
            if len(self._events) > 100:
                self._events = self._events[-100:]

    def _publish_pick_place_mode(self, mode: str):
        if not ROS_AVAILABLE or self._node is None:
            return
        msg = String()
        msg.data = mode
        self._pick_place_mode_pub.publish(msg)
        self._add_event(f"Published {PICK_PLACE_MODE_TOPIC}: {mode}")

    def calibration_command(self, action: str):
        if not ROS_AVAILABLE or self._node is None or self._calibration_command_pub is None:
            self._add_event(f"Calibration command unavailable: {action}")
            return
        msg = String()
        msg.data = json.dumps({"action": str(action).strip().lower()})
        self._calibration_command_pub.publish(msg)
        self._add_event(f"Calibration command: {action}")

    # ── MOVEIT CUBE PICK/PLACE ─────────────────────────────────────

    def pick_cube_to_bin(self, cube_id: int, bin_id: Optional[int] = None) -> bool:
        """Queue a MoveIt pick/place request for april_cube_N -> bin_N."""
        try:
            cube_id = int(cube_id)
            bin_id = cube_id if bin_id is None else int(bin_id)
        except (TypeError, ValueError):
            self._add_event(f"Pick request ignored: invalid cube/bin {cube_id!r}/{bin_id!r}")
            return False

        if cube_id not in range(1, 5) or bin_id not in range(1, 5):
            self._add_event(f"Pick request ignored: cube/bin must be 1-4 ({cube_id}/{bin_id})")
            return False

        cube_name = f"april_cube_{cube_id}"
        bin_name = f"bin_{bin_id}"

        with self._lock:
            if self._status.pick_request_pending:
                self._add_event(f"Pick request ignored: {self._status.last_pick_cube} is already pending")
                return False
            if self._status.robot_state == RobotState.ESTOPPED:
                self._add_event(f"Pick {cube_name} ignored: robot is E-stopped")
                return False

        if self._operating_mode != OperatingMode.MOVEIT:
            self._add_event(f"Pick {cube_name} ignored: switch to MOVEIT mode first")
            return False

        if not ROS_AVAILABLE or self._node is None:
            self._add_event(f"Pick {cube_name} unavailable: ROS is offline")
            return False

        if not PICK_CUBE_SERVICE_AVAILABLE or self._pick_cube_client is None:
            self._add_event("Pick cube unavailable: PickCubeToBin service type is not installed")
            return False

        self._set_pick_status(
            pending=True,
            cube_name=cube_name,
            success=None,
            message=f"Requesting {cube_name} -> {bin_name}",
        )
        self._add_event(f"Pick requested: {cube_name} -> {bin_name}")
        threading.Thread(
            target=self._do_pick_cube_to_bin,
            args=(cube_name, bin_name),
            daemon=True,
        ).start()
        return True

    def _do_pick_cube_to_bin(self, cube_name: str, bin_name: str):
        try:
            if not self._pick_cube_client.wait_for_service(timeout_sec=1.0):
                self._set_pick_status(
                    pending=False,
                    cube_name=cube_name,
                    success=False,
                    message=f"{PICK_CUBE_SERVICE} not available",
                )
                self._add_event(f"Pick {cube_name} failed: service not available")
                return

            req = PickCubeToBin.Request()
            req.cube_name = cube_name
            req.bin_id = bin_name
            future = self._pick_cube_client.call_async(req)

            start = time.time()
            while self._running and not future.done() and time.time() - start < 5.0:
                time.sleep(0.05)

            if not future.done():
                self._set_pick_status(
                    pending=False,
                    cube_name=cube_name,
                    success=False,
                    message="Service call timed out",
                )
                self._add_event(f"Pick {cube_name} failed: service call timed out")
                return

            result = future.result()
            success = bool(result.success)
            message = result.message or ("queued" if success else "failed")
            self._set_pick_status(
                pending=False,
                cube_name=cube_name,
                success=success,
                message=message,
            )
            self._add_event(f"Pick {cube_name}: {message}")
        except Exception as e:
            self._set_pick_status(
                pending=False,
                cube_name=cube_name,
                success=False,
                message=str(e),
            )
            self._add_event(f"Pick {cube_name} error: {e}")

    def _set_pick_status(
        self,
        pending: bool,
        cube_name: str,
        success: Optional[bool],
        message: str,
    ):
        with self._lock:
            self._status.pick_request_pending = pending
            self._status.last_pick_cube = cube_name
            self._status.last_pick_success = success
            self._status.last_pick_message = message

    def reconfigure_camera(self, width: int, height: int, fps: float):
        """Publish camera reconfigure and attempt ros2 param set on the active camera node."""
        if ROS_AVAILABLE and self._camera_reconfigure_pub is not None:
            msg = String()
            msg.data = f"{width}x{height}@{fps:.0f}"
            self._camera_reconfigure_pub.publish(msg)

        with self._lock:
            cam_type = self._status.camera_type

        def _do():
            profile = f"{width}x{height}x{int(fps)}"
            if "realsense" in cam_type:
                for node in ("/camera/camera", "/camera"):
                    for param in ("color_module.color_profile", "depth_module.depth_profile"):
                        try:
                            subprocess.run(
                                ["ros2", "param", "set", node, param, profile],
                                capture_output=True, text=True, timeout=3,
                            )
                        except Exception:
                            pass
            else:
                for node in ("/holo_assist_webcam_image_publisher", "/usb_cam"):
                    for param, val in [("width", str(width)), ("height", str(height)), ("fps", str(fps))]:
                        try:
                            subprocess.run(
                                ["ros2", "param", "set", node, param, val],
                                capture_output=True, text=True, timeout=3,
                            )
                        except Exception:
                            pass
            self._add_event(f"Camera reconfigure: {width}×{height}@{int(fps)}Hz")

        threading.Thread(target=_do, daemon=True).start()

    # ── MODE SWITCHING ─────────────────────────────────────────────

    def switch_to_teleop(self):
        self._add_event("Switching to TELEOP mode...")
        self._operating_mode = OperatingMode.TELEOP
        self._publish_pick_place_mode("stop")
        threading.Thread(target=self._do_switch_teleop, daemon=True).start()

    def switch_to_moveit(self):
        self._add_event("Switching to MOVEIT mode...")
        self._operating_mode = OperatingMode.MOVEIT
        self._publish_pick_place_mode("run")
        threading.Thread(target=self._do_switch_moveit, daemon=True).start()

    def _do_switch_teleop(self):
        if self._fake_hardware:
            self._add_event("Fake hardware: TELEOP mode (UI only — no controller switch)")
            with self._lock:
                self._status.controller_active = True
            return
        ok, err = self._switch_controllers(
            ["forward_velocity_controller", "finger_width_controller"],
            ["scaled_joint_trajectory_controller", "finger_width_trajectory_controller"],
        )
        if ok:
            self._add_event("TELEOP mode active (velocity + gripper controllers)")
            with self._lock:
                self._status.controller_active = True
        else:
            self._add_event(f"TELEOP switch failed: {err}")

    def _do_switch_moveit(self):
        if self._fake_hardware:
            self._add_event("Fake hardware: MOVEIT mode (UI only — no controller switch)")
            with self._lock:
                self._status.controller_active = True
            return
        ok, err = self._switch_controllers(
            ["scaled_joint_trajectory_controller", "finger_width_trajectory_controller"],
            ["forward_velocity_controller", "finger_width_controller"],
        )
        if ok:
            self._add_event("MOVEIT mode active (trajectory controllers)")
            with self._lock:
                self._status.controller_active = True
        else:
            self._add_event(f"MOVEIT switch failed: {err}")

    # ── SHUTDOWN ────────────────────────────────────────────────────

    def shutdown(self):
        self._running = False
        self._safety_stop.set()
        self._save_session_log()
        if self._node is not None:
            self._node.destroy_node()
        if ROS_AVAILABLE:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        self._add_event("Dashboard shutdown")

    def _save_session_log(self):
        """Save dashboard-side session log (events, e-stop count) to JSON."""
        log_dir = os.path.expanduser("~/holoassist_sessions")
        os.makedirs(log_dir, exist_ok=True)
        filename = f"dashboard_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json"
        path = os.path.join(log_dir, filename)

        with self._lock:
            events_copy = list(self._events)
            session_copy = dict(self._session_info)

        estop_count = sum(1 for _, msg in events_copy if "EMERGENCY STOP" in msg)
        data = {
            "timestamp": datetime.now().isoformat(),
            "estop_count": estop_count,
            "collision_events": session_copy.get("collision_events", 0),
            "gripper_grips": session_copy.get("gripper_grips", 0),
            "ee_lock_count": session_copy.get("ee_lock_count", 0),
            "event_count": len(events_copy),
            "session_info_from_unity": session_copy,
            "events": [{"t": t, "msg": msg} for t, msg in events_copy],
        }

        try:
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            print(f"Session log saved to {path}")
        except Exception as e:
            print(f"Failed to save session log: {e}")
