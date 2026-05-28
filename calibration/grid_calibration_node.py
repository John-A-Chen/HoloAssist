#!/usr/bin/env python3
"""
Auto grid hand-eye calibration for HoloAssist.

Sweeps the tool through a 3D Cartesian grid above the workspace, collects
easy_handeye2 samples at each reachable pose, then computes and saves the result.

RViz: publishes /holoassist/calibration/grid_markers (MarkerArray)
  grey  = pending
  yellow = current
  green  = sampled
  red    = skipped (unreachable / tag not seen)

Status: publishes /holoassist/calibration/status (JSON String, same schema as
coordinator_node.py so the existing dashboard CalibrationScreen works unchanged).

Orientation strategy
  Tool Z axis points in the +X direction (toward camera) tilted `tilt_deg` below
  horizontal.  Roll is cycled through a set of offsets across grid positions to
  give the rotational diversity that hand-eye calibration requires.
"""

import json
import math
import pathlib
import threading
import time
from typing import List, Tuple

import yaml

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

try:
    import easy_handeye2 as hec
    from easy_handeye2_msgs import srv as eh_srv
    EASY_HANDEYE_AVAILABLE = True
except ImportError:
    EASY_HANDEYE_AVAILABLE = False

COMMAND_TOPIC       = "/holoassist/calibration/command"
STATUS_TOPIC        = "/holoassist/calibration/status"
MOVE_STATE_TOPIC    = "/moveit_robot_control/state"
TARGET_POSE_TOPIC   = "/moveit_robot_control/target_pose"
TARGET_JOINT_TOPIC  = "/moveit_robot_control/target_joint_state"
MARKERS_TOPIC       = "/holoassist/calibration/grid_markers"

_ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

# Roll cycle for orientation diversity (degrees around tool Z axis).
_ROLL_CYCLE = [0.0, -60.0, 60.0, -120.0, 120.0, 180.0, -30.0, 30.0]


# ── Quaternion helpers ────────────────────────────────────────────────────────

def _mat_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """3×3 rotation matrix → (qx, qy, qz, qw) using Shepperd's method."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s, 0.25/s
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s
    if R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s
    s = 2.0 * math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
    return (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s


def _tool_quat(tilt_deg: float, roll_deg: float = 0.0) -> Tuple[float, float, float, float]:
    """
    Quaternion (qx, qy, qz, qw) for tool0 with Z axis pointing +X and
    tilted tilt_deg below horizontal.  roll_deg rotates around that tool Z.
    """
    tilt = math.radians(tilt_deg)
    tz = np.array([math.cos(tilt), 0.0, -math.sin(tilt)])
    wy = np.array([0.0, 1.0, 0.0])
    tx = np.cross(wy, tz)
    tx /= np.linalg.norm(tx)
    ty = np.cross(tz, tx)
    R = np.column_stack([tx, ty, tz])
    if abs(roll_deg) > 0.01:
        r = math.radians(roll_deg)
        cr, sr = math.cos(r), math.sin(r)
        R_roll = np.array([[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]])
        R = R @ R_roll
    return _mat_to_quat(R)


# ── Grid generation ───────────────────────────────────────────────────────────

def _generate_grid(x_lim, nx, y_lim, ny, z_lim, nz, tilt_deg) -> List[Pose]:
    xs = np.linspace(x_lim[0], x_lim[1], max(nx, 1))
    ys = np.linspace(y_lim[0], y_lim[1], max(ny, 1))
    zs = np.linspace(z_lim[0], z_lim[1], max(nz, 1))
    poses = []
    idx = 0
    for z in zs:
        for x in xs:
            for y in ys:
                roll = _ROLL_CYCLE[idx % len(_ROLL_CYCLE)]
                qx, qy, qz, qw = _tool_quat(tilt_deg, roll)
                p = Pose()
                p.position.x = float(x)
                p.position.y = float(y)
                p.position.z = float(z)
                p.orientation.x = qx
                p.orientation.y = qy
                p.orientation.z = qz
                p.orientation.w = qw
                poses.append(p)
                idx += 1
    return poses


# ── Node ─────────────────────────────────────────────────────────────────────

class GridCalibrationNode(Node):

    _PENDING  = "pending"
    _ACTIVE   = "active"
    _SAMPLED  = "sampled"
    _SKIPPED  = "skipped"

    def __init__(self):
        super().__init__("holoassist_grid_calibration")

        self.declare_parameter("x_min",           0.10)
        self.declare_parameter("x_max",           0.40)
        self.declare_parameter("nx",              4)
        self.declare_parameter("y_min",          -0.30)
        self.declare_parameter("y_max",           0.30)
        self.declare_parameter("ny",              4)
        self.declare_parameter("z_min",           0.18)
        self.declare_parameter("z_max",           0.34)
        self.declare_parameter("nz",              2)
        self.declare_parameter("tilt_deg",        35.0)
        self.declare_parameter("settle_s",         1.2)
        self.declare_parameter("move_timeout_s",  40.0)
        self.declare_parameter("tag_timeout_s",    8.0)
        self.declare_parameter("auto_start",      False)
        self.declare_parameter("calibration_name", "holoassist_calibration")
        self.declare_parameter("marker_frame",    "tag36h11:1")
        self.declare_parameter("poses_file",      "")  # if set, use recorded joint poses instead of grid

        g = self.get_parameter
        self._x_lim       = (g("x_min").value, g("x_max").value)
        self._y_lim       = (g("y_min").value, g("y_max").value)
        self._z_lim       = (g("z_min").value, g("z_max").value)
        self._nx          = int(g("nx").value)
        self._ny          = int(g("ny").value)
        self._nz          = int(g("nz").value)
        self._tilt_deg    = float(g("tilt_deg").value)
        self._settle_s    = float(g("settle_s").value)
        self._move_timeout = float(g("move_timeout_s").value)
        self._tag_timeout  = float(g("tag_timeout_s").value)
        self._auto_start   = bool(g("auto_start").value)
        self._cal_name     = str(g("calibration_name").value)
        self._marker_frame = str(g("marker_frame").value)
        self._poses_file   = str(g("poses_file").value).strip()

        # State
        self._lock        = threading.Lock()
        self._move_event  = threading.Event()
        self._last_move_state = ""
        self._running     = False
        self._cancel      = threading.Event()
        self._state       = "idle"
        self._message     = "Ready — press START AUTO in dashboard"
        self._pose_idx    = 0
        self._samples     = 0
        self._computed    = False
        self._error       = ""
        self._saved_path  = ""
        self._archive_path = ""
        self._server_ready = False

        # Poses: load from recorded file or generate Cartesian grid
        self._joint_poses: List[List[float]] = []  # non-empty when using recorded poses
        if self._poses_file:
            self._poses, self._joint_poses = self._load_poses_file(self._poses_file)
        else:
            self._poses = _generate_grid(
                self._x_lim, self._nx,
                self._y_lim, self._ny,
                self._z_lim, self._nz,
                self._tilt_deg,
            )
        self._flags = [self._PENDING] * len(self._poses)
        total = len(self._poses)

        # Publishers / subscribers
        self._status_pub  = self.create_publisher(String,      STATUS_TOPIC,      10)
        self._target_pub  = self.create_publisher(Pose,        TARGET_POSE_TOPIC, 10)
        self._joint_pub   = self.create_publisher(JointState,  TARGET_JOINT_TOPIC, 10)
        self._marker_pub  = self.create_publisher(MarkerArray, MARKERS_TOPIC,     10)
        self.create_subscription(String, COMMAND_TOPIC,    self._command_cb,    10)
        self.create_subscription(String, MOVE_STATE_TOPIC, self._move_state_cb, 10)

        # TF for tag visibility check
        self._tf_buf    = tf2_ros.Buffer()
        self._tf_listen = tf2_ros.TransformListener(self._tf_buf, self)

        # easy_handeye2 clients
        if EASY_HANDEYE_AVAILABLE:
            self._take_sample_cli = self.create_client(eh_srv.TakeSample,         hec.TAKE_SAMPLE_TOPIC)
            self._compute_cli     = self.create_client(eh_srv.ComputeCalibration, hec.COMPUTE_CALIBRATION_TOPIC)
            self._save_cli        = self.create_client(eh_srv.SaveCalibration,    hec.SAVE_CALIBRATION_TOPIC)
        else:
            self._take_sample_cli = self._compute_cli = self._save_cli = None

        self.create_timer(0.4, self._publish_status)
        self.create_timer(0.4, self._publish_markers)
        self.create_timer(2.0, self._check_server_ready)

        self.get_logger().info(
            f"Grid calibration: {total} poses "
            f"({self._nx}×{self._ny}×{self._nz}), tilt={self._tilt_deg}°"
        )

        if self._auto_start:
            threading.Thread(target=self._wait_then_start, daemon=True).start()

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _move_state_cb(self, msg: String) -> None:
        state = msg.data.strip().upper()
        with self._lock:
            self._last_move_state = state
        if state in ("COMPLETE", "FAILED", "CANCELLED", "REJECTED"):
            self._move_event.set()

    def _command_cb(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data).get("action", "").lower()
        except (json.JSONDecodeError, AttributeError):
            return
        if cmd == "start" and not self._running:
            threading.Thread(target=self._run_sequence, daemon=True).start()
        elif cmd == "stop":
            self._cancel.set()

    def _load_poses_file(self, path: str):
        """Load recorded_poses.yaml. Returns (Pose list for markers, joint-rad list for motion)."""
        p = pathlib.Path(path)
        with open(p) as f:
            data = yaml.safe_load(f)
        poses_deg = data.get("poses_deg", [])
        cart_poses, joint_poses = [], []
        for deg_list in poses_deg:
            rad_list = [math.radians(d) for d in deg_list]
            joint_poses.append(rad_list)
            # Dummy Cartesian pose just for marker rendering (position unknown without FK)
            dummy = Pose()
            dummy.orientation.w = 1.0
            cart_poses.append(dummy)
        self.get_logger().info(f"Loaded {len(joint_poses)} recorded poses from {p.name}")
        return cart_poses, joint_poses

    def _check_server_ready(self) -> None:
        if not EASY_HANDEYE_AVAILABLE:
            return
        ready = (
            self._take_sample_cli is not None
            and self._take_sample_cli.service_is_ready()
            and self._compute_cli.service_is_ready()
            and self._save_cli.service_is_ready()
        )
        with self._lock:
            self._server_ready = ready

    # ── Visualisation ────────────────────────────────────────────────────────

    def _publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        _COLOR = {
            self._PENDING: ColorRGBA(r=0.4, g=0.4, b=0.4, a=0.8),
            self._ACTIVE:  ColorRGBA(r=1.0, g=0.9, b=0.0, a=1.0),
            self._SAMPLED: ColorRGBA(r=0.0, g=0.9, b=0.2, a=0.9),
            self._SKIPPED: ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.7),
        }

        for i, (pose, flag) in enumerate(zip(self._poses, self._flags)):
            # Sphere at the target position
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = "base_link"
            m.ns = "calib_grid"
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose = pose
            m.scale.x = 0.06   # arrow length
            m.scale.y = 0.008  # arrow shaft diameter
            m.scale.z = 0.010  # arrow head diameter
            m.color = _COLOR.get(flag, _COLOR[self._PENDING])
            arr.markers.append(m)

        self._marker_pub.publish(arr)

    # ── Status ───────────────────────────────────────────────────────────────

    def _publish_status(self) -> None:
        with self._lock:
            payload = {
                "ready":        self._server_ready,
                "running":      self._running,
                "state":        self._state,
                "message":      self._message,
                "pose_index":   self._pose_idx,
                "pose_total":   len(self._poses),
                "sample_count": self._samples,
                "computed":     self._computed,
                "result":       {},
                "latest_path":  self._saved_path,
                "archive_path": self._archive_path,
                "error":        self._error,
                "marker_frame": self._marker_frame,
            }
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)

    def _set_state(self, state: str, message: str = "", error: str = "") -> None:
        with self._lock:
            self._state   = state
            self._message = message
            self._error   = error

    # ── Motion helpers ───────────────────────────────────────────────────────

    def _move_to(self, pose_idx: int) -> str:
        """Send a pose goal and wait.  Returns 'COMPLETE', 'FAILED', or 'CANCELLED'."""
        self._move_event.clear()
        with self._lock:
            self._last_move_state = ""
        if self._joint_poses:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = _ARM_JOINTS
            js.position = self._joint_poses[pose_idx]
            self._joint_pub.publish(js)
        else:
            self._target_pub.publish(self._poses[pose_idx])
        fired = self._move_event.wait(timeout=self._move_timeout)
        if not fired:
            return "FAILED"
        with self._lock:
            return self._last_move_state

    def _tag_visible(self) -> bool:
        """True if the calibration tag TF is available from the camera."""
        deadline = time.time() + self._tag_timeout
        while time.time() < deadline:
            try:
                self._tf_buf.lookup_transform(
                    "camera_link", self._marker_frame,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
                )
                return True
            except Exception:
                pass
            if self._cancel.is_set():
                return False
        return False

    def _call_service(self, client, request, timeout_sec: float = 8.0) -> bool:
        """Call a ROS service safely from a background thread (node is already spinning)."""
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        fired = done.wait(timeout=timeout_sec)
        return fired and future.done() and future.result() is not None

    def _take_sample(self) -> bool:
        if not EASY_HANDEYE_AVAILABLE or self._take_sample_cli is None:
            self.get_logger().warning("easy_handeye2 not available — skipping sample")
            return False
        return self._call_service(self._take_sample_cli, eh_srv.TakeSample.Request(), timeout_sec=5.0)

    def _compute(self) -> bool:
        if not EASY_HANDEYE_AVAILABLE or self._compute_cli is None:
            return False
        ok = self._call_service(self._compute_cli, eh_srv.ComputeCalibration.Request(), timeout_sec=15.0)
        with self._lock:
            self._computed = ok
        return ok

    def _save(self) -> bool:
        if not EASY_HANDEYE_AVAILABLE or self._save_cli is None:
            return False
        ok = self._call_service(self._save_cli, eh_srv.SaveCalibration.Request(), timeout_sec=10.0)
        if ok:
            import os, pathlib
            cal_dir = pathlib.Path.home() / ".ros2" / "easy_handeye2"
            latest = sorted(cal_dir.glob("*.calib"), key=os.path.getmtime)
            path = str(latest[-1]) if latest else ""
            with self._lock:
                self._saved_path = path
        return ok

    # ── Main sequence ────────────────────────────────────────────────────────

    def _wait_then_start(self) -> None:
        """Wait for easy_handeye2 to be ready, then start the sequence."""
        deadline = time.time() + 60.0
        while time.time() < deadline and not self._server_ready:
            time.sleep(1.0)
        if self._server_ready:
            self._run_sequence()
        else:
            self._set_state("error", "Timed out waiting for easy_handeye2 server")

    def _run_sequence(self) -> None:
        with self._lock:
            if self._running:
                return
            self._running = True
            self._flags   = [self._PENDING] * len(self._poses)
            self._pose_idx = 0
            self._samples  = 0
            self._computed = False
            self._error    = ""
        self._cancel.clear()

        self._set_state("moving", "Starting grid calibration sequence")
        self.get_logger().info(f"Starting grid sequence: {len(self._poses)} poses")

        for i, pose in enumerate(self._poses):
            if self._cancel.is_set():
                self._set_state("cancelled", f"Cancelled after {self._samples} samples")
                break

            with self._lock:
                self._pose_idx = i
                self._flags[i] = self._ACTIVE
            self._set_state("moving", f"Moving to pose {i+1}/{len(self._poses)}")

            result = self._move_to(pose)
            if result != "COMPLETE":
                self.get_logger().warning(f"Pose {i+1} move result: {result} — skipping")
                with self._lock:
                    self._flags[i] = self._SKIPPED
                continue

            if self._cancel.is_set():
                break

            # Settle
            self._set_state("settling", f"Pose {i+1}: waiting for settle + tag")
            time.sleep(self._settle_s)

            # Check tag visibility
            if not self._tag_visible():
                self.get_logger().warning(f"Pose {i+1}: tag not visible — skipping")
                with self._lock:
                    self._flags[i] = self._SKIPPED
                continue

            # Take sample
            self._set_state("sampling", f"Pose {i+1}: taking sample")
            ok = self._take_sample()
            if ok:
                with self._lock:
                    self._flags[i] = self._SAMPLED
                    self._samples += 1
                self.get_logger().info(f"Pose {i+1}: sample {self._samples} collected")
            else:
                self.get_logger().warning(f"Pose {i+1}: take_sample failed")
                with self._lock:
                    self._flags[i] = self._SKIPPED

        # Sequence done
        if not self._cancel.is_set():
            sampled = sum(1 for f in self._flags if f == self._SAMPLED)
            if sampled >= 8:
                self._set_state("computing", f"Computing calibration from {sampled} samples")
                ok = self._compute()
                if ok:
                    self._set_state("computed", f"Done — {sampled} samples. Press SAVE to write result.")
                else:
                    self._set_state("error", "Compute failed", error="ComputeCalibration service error")
            else:
                self._set_state(
                    "error",
                    f"Only {sampled} samples — need ≥8. Try rerunning.",
                    error=f"Insufficient samples: {sampled}",
                )

        with self._lock:
            self._running = False


def main(args=None):
    rclpy.init(args=args)
    node = GridCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
