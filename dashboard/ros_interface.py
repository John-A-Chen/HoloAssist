"""
ROS 2 interface for HoloAssist Dashboard.
Handles e-stop, velocity publishing, joint state monitoring, perception topics,
and controller switching.

Runs rclpy in a background thread; all public methods are thread-safe.
"""

import json
import os
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


class RobotState(Enum):
    DISCONNECTED = auto()
    RUNNING = auto()
    ESTOPPED = auto()
    RESUMING = auto()


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
    "gripper_cmd": "/gripper/command",
}


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
        self._last_vel_cmd_time: float = 0.0                # timestamp of last velocity_cmd

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

        # Gripper command (from Unity RobotController)
        self._node.create_subscription(
            Float32, TOPIC_DEFAULTS["gripper_cmd"],
            self._gripper_cmd_cb, qos_profile_sensor_data,
        )

        # Sampling timers for rolling graph data
        self._node.create_timer(0.1, self._sample_velocities)   # 10Hz
        self._node.create_timer(0.5, self._sample_rates)        # 2Hz

        self._running = True
        with self._lock:
            self._status.ros_connected = True
            self._status.robot_state = RobotState.RUNNING
        self._add_event("ROS 2 node started")
        self._add_event(f"Subscribing to {len(TOPIC_DEFAULTS)} topics")

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

    def _gripper_cmd_cb(self, msg):
        self._tick_rate("gripper_cmd")
        with self._lock:
            self._status.gripper_value = msg.data

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

    def _deactivate_controller(self):
        try:
            result = subprocess.run(
                ["ros2", "control", "switch_controllers",
                 "--deactivate", "forward_velocity_controller"],
                capture_output=True, text=True, timeout=10,
            )
            if result.returncode == 0:
                self._add_event("forward_velocity_controller deactivated")
                with self._lock:
                    self._status.controller_active = False
            else:
                self._add_event(f"Controller deactivation failed: {result.stderr.strip()}")
        except Exception as e:
            self._add_event(f"Controller deactivation error: {e}")

    # ── RESUME ──────────────────────────────────────────────────────

    def resume(self):
        with self._lock:
            if self._status.robot_state != RobotState.ESTOPPED:
                return
            self._status.robot_state = RobotState.RESUMING

        self._add_event("Resuming - reactivating controller...")
        self._safety_stop.set()
        threading.Thread(target=self._activate_controller, daemon=True).start()

    def _activate_controller(self):
        try:
            result = subprocess.run(
                ["ros2", "control", "switch_controllers",
                 "--activate", "forward_velocity_controller",
                 "--deactivate", "scaled_joint_trajectory_controller"],
                capture_output=True, text=True, timeout=10,
            )
            if result.returncode == 0:
                self._add_event("forward_velocity_controller reactivated - ROBOT LIVE")
                with self._lock:
                    self._status.robot_state = RobotState.RUNNING
                    self._status.controller_active = True
                    self._status.estop_zero_count = 0
            else:
                self._add_event(f"Controller activation failed: {result.stderr.strip()}")
                with self._lock:
                    self._status.robot_state = RobotState.ESTOPPED
        except Exception as e:
            self._add_event(f"Controller activation error: {e}")
            with self._lock:
                self._status.robot_state = RobotState.ESTOPPED

    # ── STATUS ──────────────────────────────────────────────────────

    def _check_controller_status(self):
        try:
            result = subprocess.run(
                ["ros2", "control", "list_controllers"],
                capture_output=True, text=True, timeout=10,
            )
            if result.returncode == 0:
                for line in result.stdout.splitlines():
                    if "forward_velocity_controller" in line:
                        active = "active" in line and "inactive" not in line
                        with self._lock:
                            self._status.controller_active = active
                        self._add_event(f"forward_velocity_controller is {'active' if active else 'inactive'}")
                        return
                self._add_event("forward_velocity_controller not found in controller list")
        except Exception as e:
            self._add_event(f"Could not check controllers: {e}")

    def get_status(self) -> DashboardStatus:
        """Return a snapshot of current status. Called from GUI thread."""
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
                velocity_history=vel_hist,
                rate_history=rate_hist,
                latency_history=lat_hist,
            )
        return status

    def _add_event(self, msg: str):
        entry = (time.time(), msg)
        with self._lock:
            self._events.append(entry)
            if len(self._events) > 100:
                self._events = self._events[-100:]

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
            "estop_count": estop_count,
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
