#!/usr/bin/env python3
"""
HoloAssist Board Calibration Node

Robot-assisted "bed levelling" for the UR3e + workspace board setup.

Workflow
--------
1. Robot hovers above each of the 4 workspace tag centres in sequence.
2. User physically nudges the board until the tag is directly under the TCP.
3. Robot FK (tool0 pose in base_link) gives ground-truth tag positions.
4. SVD best-fit rigid transform is solved over all 4 tag pairs.
5. Calibrated workspace_frame transform is saved to YAML and applied as a
   static TF for the current session.

Physical tag layout (from workspace_perception_params.yaml):
  Tag 1  →  left-front   →  workspace_frame (-334, -234, 0) mm
  Tag 0  →  right-front  →  workspace_frame (+334, -234, 0) mm
  Tag 2  →  left-rear    →  workspace_frame (-334, +234, 0) mm
  Tag 3  →  right-rear   →  workspace_frame (+334, +234, 0) mm
  (+X = right, +Y = toward robot / rear, +Z = up from board surface)

Robot frame   : base_link        (UR3e planning frame, calibration ground truth)
EE frame      : tool0            (UR flange, FK measurement and hover target)
Planning group: ur_onrobot_manipulator

Hover orientation: tool0 Z pointing straight down in base_link (q = 1,0,0,0).

After calibration the node publishes a calibrated static TF
  base_link → workspace_frame
and saves parameters to ~/.holoassist/calibration/calibration_latest.yaml
(compatible with workspace_frame_tf node).

IMPORTANT — workspace_frame conflict
  workspace_board_node publishes   camera_color_optical_frame → workspace_frame
  workspace_frame_tf   publishes   base_link                  → workspace_frame
  Both cannot run simultaneously — workspace_frame can only have one parent.
  The calibration launch does NOT start workspace_board_node.
  workspace_board_node is started only for the optional camera verification pass.
  See board_calibration.launch.py for details.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformException,
    TransformListener,
)
from trajectory_msgs.msg import JointTrajectory

from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetMotionPlan
from shape_msgs.msg import SolidPrimitive

from holo_assist_depth_tracker.utils.math3d import (
    quaternion_from_rotation_matrix,
    rotation_matrix_from_quaternion,
)

# ── MoveIt / robot constants ──────────────────────────────────────────────────
_MOVE_GROUP = "ur_onrobot_manipulator"
_BASE_FRAME = "base_link"
_EE_LINK = "tool0"
_PLANNING_SVC = "/plan_kinematic_path"
_UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# tool0 pointing straight down: 180° around X  →  q = (x=1, y=0, z=0, w=0)
_TOOL_DOWN = (1.0, 0.0, 0.0, 0.0)  # (x, y, z, w) geometry_msgs convention
_MOVEIT_OK = 1

# Physical tag visit order: front-left, front-right, rear-left, rear-right
_VISIT_ORDER = [1, 0, 2, 3]
_TAG_CORNER = {1: "left-front", 0: "right-front", 2: "left-rear", 3: "right-rear"}


# ── Math helpers ──────────────────────────────────────────────────────────────

def _svd_rigid(src: np.ndarray, dst: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Best-fit rigid body transform: dst ≈ R @ src + t  (4+ point pairs)."""
    src_c = src.mean(axis=0)
    dst_c = dst.mean(axis=0)
    H = (src - src_c).T @ (dst - dst_c)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if float(np.linalg.det(R)) < 0.0:
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T
    t = dst_c - R @ src_c
    return R, t


def _rotation_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """Extract roll, pitch, yaw (ZYX extrinsic / XYZ intrinsic) from 3×3 R."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def _rotation_from_yaw(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _normalize_quaternion(x: float, y: float, z: float, w: float) -> Tuple[float, float, float, float]:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / n, y / n, z / n, w / n


# ── Calibration node ──────────────────────────────────────────────────────────

class BoardCalibrationNode(Node):
    """Interactive board calibration: robot-FK-based workspace_frame alignment."""

    def __init__(self) -> None:
        super().__init__("holoassist_board_calibration")

        # ── board geometry params ─────────────────────────────────────────────
        self.declare_parameter("board_width_m", 0.700)
        self.declare_parameter("board_depth_m", 0.500)
        self.declare_parameter("board_tag_edge_offset_m", 0.016)

        # ── approx initial workspace_frame position in base_link ──────────────
        # Use workspace_frame_tf.py defaults; calibration improves these.
        self.declare_parameter("approx_ws_x_m", -0.100)
        self.declare_parameter("approx_ws_y_m", -0.314)
        self.declare_parameter("approx_ws_z_m", 0.015)
        self.declare_parameter("approx_ws_yaw_rad", 0.0)

        # ── motion params ─────────────────────────────────────────────────────
        self.declare_parameter("hover_height_m", 0.150)
        self.declare_parameter("move_group_name", _MOVE_GROUP)
        self.declare_parameter("ee_link", _EE_LINK)
        self.declare_parameter("base_frame", _BASE_FRAME)
        self.declare_parameter("trajectory_topic", "/scaled_joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("velocity_scale", 0.08)
        self.declare_parameter("planning_time_s", 8.0)
        self.declare_parameter("position_tolerance_m", 0.010)
        self.declare_parameter("orientation_tolerance_rad", 0.5)
        self.declare_parameter("joint_settle_tolerance_rad", 0.05)
        self.declare_parameter("execution_timeout_scale", 20.0)

        # ── calibration output ────────────────────────────────────────────────
        self.declare_parameter("output_dir", os.path.expanduser("~/.holoassist/calibration"))
        self.declare_parameter("pass_tolerance_m", 0.005)
        self.declare_parameter("publish_calibrated_tf", True)

        # ── camera verification ───────────────────────────────────────────────
        self.declare_parameter("verify_with_camera", True)
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("workspace_frame", "workspace_frame")
        self.declare_parameter("camera_verify_timeout_s", 2.0)

        # load
        self._board_w = float(self.get_parameter("board_width_m").value)
        self._board_d = float(self.get_parameter("board_depth_m").value)
        self._edge = float(self.get_parameter("board_tag_edge_offset_m").value)
        self._approx_ws = np.array([
            float(self.get_parameter("approx_ws_x_m").value),
            float(self.get_parameter("approx_ws_y_m").value),
            float(self.get_parameter("approx_ws_z_m").value),
        ], dtype=np.float64)
        self._approx_ws_R = _rotation_from_yaw(
            float(self.get_parameter("approx_ws_yaw_rad").value)
        )
        self._hover_h = float(self.get_parameter("hover_height_m").value)
        self._move_group = str(self.get_parameter("move_group_name").value)
        self._ee_link = str(self.get_parameter("ee_link").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._traj_topic = str(self.get_parameter("trajectory_topic").value)
        self._vel_scale = float(self.get_parameter("velocity_scale").value)
        self._plan_time = float(self.get_parameter("planning_time_s").value)
        self._pos_tol = float(self.get_parameter("position_tolerance_m").value)
        self._ori_tol = float(self.get_parameter("orientation_tolerance_rad").value)
        self._settle_tol = float(self.get_parameter("joint_settle_tolerance_rad").value)
        self._exec_timeout_scale = float(self.get_parameter("execution_timeout_scale").value)
        self._output_dir = Path(str(self.get_parameter("output_dir").value))
        self._pass_tol = float(self.get_parameter("pass_tolerance_m").value)
        self._publish_cal_tf = bool(self.get_parameter("publish_calibrated_tf").value)
        self._verify_cam = bool(self.get_parameter("verify_with_camera").value)
        self._tag_family = str(self.get_parameter("tag_family").value)
        self._ws_frame = str(self.get_parameter("workspace_frame").value)
        self._cam_verify_timeout = float(self.get_parameter("camera_verify_timeout_s").value)

        # build tag model positions in centred workspace_frame
        hw, hd = self._board_w / 2.0, self._board_d / 2.0
        e = self._edge
        self._tag_model: Dict[int, np.ndarray] = {
            1: np.array([-hw + e, -hd + e, 0.0], dtype=np.float64),  # left-front
            0: np.array([+hw - e, -hd + e, 0.0], dtype=np.float64),  # right-front
            2: np.array([-hw + e, +hd - e, 0.0], dtype=np.float64),  # left-rear
            3: np.array([+hw - e, +hd - e, 0.0], dtype=np.float64),  # right-rear
        }

        # ── ROS connections ───────────────────────────────────────────────────
        self._current_joints: Optional[List[float]] = None
        self._joints_lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._joint_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joints, qos
        )
        self._traj_pub = self.create_publisher(JointTrajectory, self._traj_topic, qos)

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self._plan_client = self.create_client(GetMotionPlan, _PLANNING_SVC)
        self._static_tf_pub = StaticTransformBroadcaster(self)

        # user confirmation flag (set by stdin thread)
        self._user_confirmed = threading.Event()

        self.get_logger().info(
            "BoardCalibrationNode ready — board=%.3f×%.3fm hover=%.3fm "
            "trajectory=%s"
            % (self._board_w, self._board_d, self._hover_h, self._traj_topic)
        )

    # ── joint state callback ──────────────────────────────────────────────────

    def _on_joints(self, msg: JointState) -> None:
        joint_map = dict(zip(msg.name, msg.position))
        if all(j in joint_map for j in _UR_JOINTS):
            with self._joints_lock:
                self._current_joints = [float(joint_map[j]) for j in _UR_JOINTS]

    def _wait_joints(self, timeout: float = 15.0) -> List[float]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._joints_lock:
                if self._current_joints is not None:
                    return list(self._current_joints)
        raise RuntimeError("Timed out waiting for /joint_states")

    # ── hover position ────────────────────────────────────────────────────────

    def _hover_pose(self, tag_id: int) -> Pose:
        """Compute hover Pose (in base_link) above tag_id using approx transform."""
        tag_ws = self._tag_model[tag_id]
        tag_base = self._approx_ws_R @ tag_ws + self._approx_ws
        hover_base = tag_base + np.array([0.0, 0.0, self._hover_h])

        pose = Pose()
        pose.position.x = float(hover_base[0])
        pose.position.y = float(hover_base[1])
        pose.position.z = float(hover_base[2])
        pose.orientation.x = _TOOL_DOWN[0]
        pose.orientation.y = _TOOL_DOWN[1]
        pose.orientation.z = _TOOL_DOWN[2]
        pose.orientation.w = _TOOL_DOWN[3]
        return pose

    # ── MoveIt motion ─────────────────────────────────────────────────────────

    def _wait_moveit(self, timeout: float = 30.0) -> None:
        self.get_logger().info("Waiting for MoveIt planning service …")
        if not self._plan_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                f"{_PLANNING_SVC} not available after {timeout}s — "
                "is move_group running?"
            )

    def _build_pose_constraints(
        self, target: Pose, current_joints: List[float]
    ) -> Constraints:
        now = self.get_clock().now().to_msg()
        c = Constraints()
        c.name = "hover_goal"

        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self._pos_tol]
        region.primitives.append(sphere)
        from geometry_msgs.msg import Pose as _Pose
        rp = _Pose()
        rp.position.x = target.position.x
        rp.position.y = target.position.y
        rp.position.z = target.position.z
        region.primitive_poses.append(rp)
        pc = PositionConstraint()
        pc.header.stamp = now
        pc.header.frame_id = self._base_frame
        pc.link_name = self._ee_link
        pc.constraint_region = region
        pc.weight = 1.0
        c.position_constraints.append(pc)

        ox, oy, oz, ow = _normalize_quaternion(*_TOOL_DOWN)
        oc = OrientationConstraint()
        oc.header.stamp = now
        oc.header.frame_id = self._base_frame
        oc.link_name = self._ee_link
        oc.orientation.x = ox
        oc.orientation.y = oy
        oc.orientation.z = oz
        oc.orientation.w = ow
        oc.absolute_x_axis_tolerance = self._ori_tol
        oc.absolute_y_axis_tolerance = self._ori_tol
        oc.absolute_z_axis_tolerance = self._ori_tol
        oc.parameterization = OrientationConstraint.ROTATION_VECTOR
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        return c

    def _plan_to_pose(self, target: Pose, current_joints: List[float]) -> JointTrajectory:
        request = GetMotionPlan.Request()
        mp = request.motion_plan_request
        mp.group_name = self._move_group
        mp.num_planning_attempts = 3
        mp.allowed_planning_time = self._plan_time
        mp.max_velocity_scaling_factor = self._vel_scale
        mp.max_acceleration_scaling_factor = self._vel_scale
        mp.workspace_parameters.header.frame_id = self._base_frame
        mp.workspace_parameters.min_corner.x = -2.0
        mp.workspace_parameters.min_corner.y = -2.0
        mp.workspace_parameters.min_corner.z = -0.5
        mp.workspace_parameters.max_corner.x = 2.0
        mp.workspace_parameters.max_corner.y = 2.0
        mp.workspace_parameters.max_corner.z = 2.0
        mp.goal_constraints = [self._build_pose_constraints(target, current_joints)]
        mp.start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        mp.start_state.joint_state.name = list(_UR_JOINTS)
        mp.start_state.joint_state.position = list(current_joints)
        mp.start_state.is_diff = False

        future = self._plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._plan_time + 15.0)

        if not future.done() or future.exception():
            raise RuntimeError(
                "MoveIt planning service failed: "
                + str(future.exception() if future.done() else "timeout")
            )

        resp = future.result()
        if resp is None:
            raise RuntimeError("MoveIt returned no response")

        ec = resp.motion_plan_response.error_code.val
        if ec != _MOVEIT_OK:
            raise RuntimeError(f"MoveIt planning failed (error_code={ec})")

        traj = resp.motion_plan_response.trajectory.joint_trajectory
        if not traj.points:
            raise RuntimeError("MoveIt returned empty trajectory")

        self.get_logger().info(
            "Plan OK — %d points, estimated %.2f s"
            % (
                len(traj.points),
                float(traj.points[-1].time_from_start.sec)
                + float(traj.points[-1].time_from_start.nanosec) / 1e9,
            )
        )
        return traj

    def _execute_trajectory(self, traj: JointTrajectory) -> None:
        self._traj_pub.publish(traj)

        final_pt = traj.points[-1]
        dur_s = float(final_pt.time_from_start.sec) + float(final_pt.time_from_start.nanosec) / 1e9
        timeout = max(dur_s + 5.0, dur_s * self._exec_timeout_scale + 5.0)

        jname_to_idx = {n: i for i, n in enumerate(traj.joint_names)}
        target = [float(final_pt.positions[jname_to_idx[j]]) for j in _UR_JOINTS if j in jname_to_idx]

        self.get_logger().info(f"Executing trajectory (nominal {dur_s:.1f} s) …")
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._joints_lock:
                cur = self._current_joints
            if cur is None:
                continue
            err = max(abs(a - b) for a, b in zip(cur, target))
            if err < self._settle_tol:
                self.get_logger().info(f"Robot settled (max joint err {err:.4f} rad)")
                return

        raise RuntimeError(
            f"Execution did not settle within {timeout:.1f} s; "
            f"last joint error >{self._settle_tol:.3f} rad"
        )

    # ── FK measurement ────────────────────────────────────────────────────────

    def _measure_tag_position(self, tag_id: int) -> np.ndarray:
        """Return measured tag centre in base_link using robot FK (tool0 TF)."""
        for attempt in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tf = self._tf_buf.lookup_transform(
                    self._base_frame,
                    self._ee_link,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0),
                )
                t = tf.transform.translation
                tool0_xyz = np.array([float(t.x), float(t.y), float(t.z)])
                # Tag centre = directly below tool0 at board surface depth
                tag_xyz = tool0_xyz - np.array([0.0, 0.0, self._hover_h])
                self.get_logger().info(
                    "Tag %d FK measurement: base_link (%.4f, %.4f, %.4f) m"
                    % (tag_id, tag_xyz[0], tag_xyz[1], tag_xyz[2])
                )
                return tag_xyz
            except TransformException:
                time.sleep(0.05)
        raise RuntimeError(
            f"Could not look up {self._base_frame}→{self._ee_link} TF "
            "after 20 attempts. Is the robot driver running?"
        )

    # ── camera verification ───────────────────────────────────────────────────

    def _verify_with_camera(
        self, R_cal: np.ndarray, t_cal: np.ndarray
    ) -> Optional[Dict[int, float]]:
        """
        Optional camera verification pass.

        Looks up each workspace tag TF in workspace_frame (requires
        workspace_board_node to be running).  Compares observed positions to
        the board model and reports residuals.

        Returns a dict {tag_id: residual_m} or None if tags are not visible.
        """
        if not self._verify_cam:
            return None

        residuals: Dict[int, float] = {}
        any_found = False
        for tag_id in _VISIT_ORDER:
            frame = f"tag{self._tag_family}:{tag_id}"
            try:
                tf = self._tf_buf.lookup_transform(
                    self._ws_frame,
                    frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self._cam_verify_timeout),
                )
                t = tf.transform.translation
                observed = np.array([float(t.x), float(t.y), float(t.z)])
                expected = self._tag_model[tag_id]
                err = float(np.linalg.norm(observed[:2] - expected[:2]))
                residuals[tag_id] = err
                any_found = True
            except TransformException:
                residuals[tag_id] = float("nan")

        if not any_found:
            return None
        return residuals

    # ── SVD solve ─────────────────────────────────────────────────────────────

    def _solve(
        self, measured: Dict[int, np.ndarray]
    ) -> Tuple[np.ndarray, np.ndarray, Dict[int, float], float]:
        """
        SVD rigid-body solve.

        Returns (R, t, per_tag_residuals_m, rms_m) where:
          R: 3×3 rotation   workspace_frame in base_link
          t: translation     workspace_frame origin in base_link
        """
        tag_ids = list(measured.keys())
        src = np.stack([self._tag_model[i] for i in tag_ids], axis=0)
        dst = np.stack([measured[i] for i in tag_ids], axis=0)

        R, t = _svd_rigid(src, dst)

        predicted = (R @ src.T).T + t
        per_tag = {
            tid: float(np.linalg.norm(predicted[i] - dst[i]))
            for i, tid in enumerate(tag_ids)
        }
        rms = float(np.sqrt(np.mean([v ** 2 for v in per_tag.values()])))
        return R, t, per_tag, rms

    # ── YAML save ─────────────────────────────────────────────────────────────

    def _save_yaml(
        self,
        R: np.ndarray,
        t: np.ndarray,
        per_tag: Dict[int, float],
        rms: float,
        camera_residuals: Optional[Dict[int, float]],
    ) -> Path:
        """
        Save calibration result as YAML.

        Format is directly loadable by workspace_frame_tf node:
          ros2 run moveit_robot_control workspace_frame_tf \\
              --ros-args --params-file ~/.holoassist/calibration/calibration_latest.yaml

        Two files are written:
          {output_dir}/calibration_YYYY-MM-DD_HH-MM-SS.yaml  (timestamped)
          {output_dir}/calibration_latest.yaml                (always overwritten)
        """
        self._output_dir.mkdir(parents=True, exist_ok=True)

        roll, pitch, yaw = _rotation_to_rpy(R)
        qx, qy, qz, qw = quaternion_from_rotation_matrix(R)

        pass_flag = rms <= self._pass_tol
        ts = datetime.now().isoformat(timespec="seconds")

        data: dict = {
            "holoassist_workspace_frame_tf": {
                "ros__parameters": {
                    "parent_frame": self._base_frame,
                    "child_frame": self._ws_frame,
                    "x_m": float(round(t[0], 6)),
                    "y_m": float(round(t[1], 6)),
                    "z_m": float(round(t[2], 6)),
                    "roll_rad": float(round(roll, 6)),
                    "pitch_rad": float(round(pitch, 6)),
                    "yaw_rad": float(round(yaw, 6)),
                },
            },
            "_calibration_meta": {
                "timestamp": ts,
                "rms_residual_m": float(round(rms, 6)),
                "pass": bool(pass_flag),
                "tolerance_m": float(self._pass_tol),
                "per_tag_fk_residuals_m": {
                    f"tag_{k}": float(round(v, 6)) for k, v in per_tag.items()
                },
                "per_tag_camera_residuals_m": (
                    {
                        f"tag_{k}": (
                            float(round(v, 6)) if not math.isnan(v) else None
                        )
                        for k, v in camera_residuals.items()
                    }
                    if camera_residuals
                    else None
                ),
                "quaternion_xyzw": [
                    float(round(qx, 6)),
                    float(round(qy, 6)),
                    float(round(qz, 6)),
                    float(round(qw, 6)),
                ],
            },
        }

        try:
            import yaml as _yaml
            yaml_str = _yaml.dump(data, default_flow_style=False, sort_keys=False)
        except ImportError:
            # Minimal YAML serialisation without pyyaml
            lines = [
                "# HoloAssist board calibration result",
                f"# Generated: {ts}",
                f"# RMS: {rms * 1000:.1f} mm  pass={'yes' if pass_flag else 'NO'}",
                "",
                "holoassist_workspace_frame_tf:",
                "  ros__parameters:",
                f"    parent_frame: {self._base_frame}",
                f"    child_frame: {self._ws_frame}",
                f"    x_m: {t[0]:.6f}",
                f"    y_m: {t[1]:.6f}",
                f"    z_m: {t[2]:.6f}",
                f"    roll_rad: {roll:.6f}",
                f"    pitch_rad: {pitch:.6f}",
                f"    yaw_rad: {yaw:.6f}",
            ]
            yaml_str = "\n".join(lines) + "\n"

        ts_file = ts.replace(":", "-").replace("T", "_")
        timestamped = self._output_dir / f"calibration_{ts_file}.yaml"
        latest = self._output_dir / "calibration_latest.yaml"

        timestamped.write_text(yaml_str)
        latest.write_text(yaml_str)

        self.get_logger().info(f"Calibration saved → {timestamped}")
        self.get_logger().info(f"Calibration saved → {latest}")
        return latest

    # ── apply: publish calibrated static TF ──────────────────────────────────

    def _publish_calibrated_tf(self, R: np.ndarray, t: np.ndarray) -> None:
        """Publish calibrated base_link → workspace_frame static TF."""
        qx, qy, qz, qw = quaternion_from_rotation_matrix(R)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.child_frame_id = self._ws_frame
        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = float(t[2])
        msg.transform.rotation.x = float(qx)
        msg.transform.rotation.y = float(qy)
        msg.transform.rotation.z = float(qz)
        msg.transform.rotation.w = float(qw)
        self._static_tf_pub.sendTransform(msg)
        self.get_logger().info(
            "Published calibrated static TF: %s → %s  "
            "xyz=(%.4f, %.4f, %.4f) rpy_deg=(%.2f, %.2f, %.2f)"
            % (
                self._base_frame,
                self._ws_frame,
                t[0],
                t[1],
                t[2],
                *[math.degrees(a) for a in _rotation_to_rpy(R)],
            )
        )

    # ── user interaction ──────────────────────────────────────────────────────

    def _start_stdin_thread(self) -> None:
        def _read() -> None:
            sys.stdin.readline()
            self._user_confirmed.set()

        t = threading.Thread(target=_read, daemon=True)
        t.start()

    def _wait_user(self, prompt: str) -> None:
        """Prompt the user and block until they press ENTER."""
        self._user_confirmed.clear()
        self._start_stdin_thread()
        print(f"\n  {prompt}", flush=True)
        print("  >>> Press ENTER when ready <<<", flush=True)
        while rclpy.ok() and not self._user_confirmed.wait(timeout=0.2):
            rclpy.spin_once(self, timeout_sec=0.0)
        if not rclpy.ok():
            raise RuntimeError("ROS shutdown during user wait")

    # ── main calibration loop ─────────────────────────────────────────────────

    def run(self) -> None:
        """Execute the full calibration routine (blocking)."""
        self._print_banner()

        # Wait for joint states and MoveIt
        print("\n[1/6] Waiting for /joint_states …", flush=True)
        current_joints = self._wait_joints(timeout=30.0)
        print(f"      Joint state OK: {[round(j, 3) for j in current_joints]}", flush=True)

        print("[2/6] Waiting for MoveIt …", flush=True)
        self._wait_moveit(timeout=60.0)
        print("      MoveIt OK", flush=True)

        self._wait_user(
            "Setup check complete.\n"
            "  • Robot is connected and in External Control mode\n"
            "  • Board is roughly positioned in front of the robot\n"
            "  • You can slide the board freely during hover pauses\n"
            "Confirm ready to start calibration"
        )

        # ── step 3: visit all 4 tag hover points ─────────────────────────────
        print("\n[3/6] Visiting 4 tag hover points …", flush=True)
        measured: Dict[int, np.ndarray] = {}

        for step_n, tag_id in enumerate(_VISIT_ORDER, start=1):
            corner = _TAG_CORNER[tag_id]
            model_pos = self._tag_model[tag_id]
            hover = self._hover_pose(tag_id)

            self._print_step_header(step_n, tag_id, corner, model_pos, hover)

            # plan
            print(f"      Planning to hover above tag {tag_id} ({corner}) …", flush=True)
            with self._joints_lock:
                cur = list(self._current_joints or current_joints)
            traj = self._plan_to_pose(hover, cur)

            # execute
            print(f"      Executing …", flush=True)
            self._execute_trajectory(traj)

            # prompt user to align board
            self._wait_user(
                f"HOVER POINT {step_n}/4 — Tag {tag_id} ({corner})\n"
                f"  Robot is hovering {int(self._hover_h * 1000)} mm above the expected\n"
                f"  {corner} tag location.\n"
                f"  → Slide or rotate the board so tag {tag_id} is directly under\n"
                f"    the robot TCP (the end of the gripper).\n"
                f"  → Small yaw correction is OK.\n"
                f"  Board aligned with tag {tag_id} under TCP"
            )

            # record FK measurement
            print(f"      Recording FK measurement for tag {tag_id} …", flush=True)
            measured[tag_id] = self._measure_tag_position(tag_id)

        # ── step 4: SVD solve ─────────────────────────────────────────────────
        print("\n[4/6] Solving workspace_frame transform (SVD) …", flush=True)
        R_cal, t_cal, per_tag_fk, rms_fk = self._solve(measured)
        self._print_fk_results(per_tag_fk, rms_fk)

        # ── step 5: camera verification ───────────────────────────────────────
        print("\n[5/6] Camera verification …", flush=True)
        camera_residuals = self._verify_with_camera(R_cal, t_cal)
        self._print_camera_results(camera_residuals)

        # ── step 6: save + apply ──────────────────────────────────────────────
        print("\n[6/6] Saving calibration result …", flush=True)
        latest = self._save_yaml(R_cal, t_cal, per_tag_fk, rms_fk, camera_residuals)

        if self._publish_calibrated_tf:
            print("      Publishing calibrated static TF …", flush=True)
            self._publish_calibrated_tf(R_cal, t_cal)

        self._print_final_report(R_cal, t_cal, rms_fk, latest)

        # stay alive so the static TF keeps broadcasting
        if self._publish_calibrated_tf:
            print(
                "\n  Calibration node is STAYING ALIVE to publish the static TF.\n"
                "  Leave it running for the current session.\n"
                "  Press Ctrl+C to stop.\n",
                flush=True,
            )
            rclpy.spin(self)

    # ── terminal output helpers ───────────────────────────────────────────────

    def _print_banner(self) -> None:
        print("\n" + "=" * 70)
        print("  HoloAssist Board Calibration")
        print("  Board: %.0f × %.0f mm" % (self._board_w * 1000, self._board_d * 1000))
        print("  Hover: %d mm above board surface" % int(self._hover_h * 1000))
        print("  Tag visit order: %s" % " → ".join(
            f"Tag {t} ({_TAG_CORNER[t]})" for t in _VISIT_ORDER
        ))
        print("=" * 70)

    def _print_step_header(
        self,
        step: int,
        tag_id: int,
        corner: str,
        model_pos: np.ndarray,
        hover: Pose,
    ) -> None:
        print(
            "\n  ── Step %d/4: Tag %d (%s) ──"
            % (step, tag_id, corner)
        )
        print(
            "  Model position in workspace_frame: "
            "(%.1f, %.1f, %.1f) mm"
            % (model_pos[0] * 1000, model_pos[1] * 1000, model_pos[2] * 1000)
        )
        print(
            "  Approx hover target in base_link:  "
            "(%.4f, %.4f, %.4f) m"
            % (hover.position.x, hover.position.y, hover.position.z)
        )

    def _print_fk_results(
        self, per_tag: Dict[int, float], rms: float
    ) -> None:
        pass_str = "PASS" if rms <= self._pass_tol else "FAIL"
        tol_mm = self._pass_tol * 1000
        print(f"\n  FK residuals (robot ground truth):")
        for tid in _VISIT_ORDER:
            if tid in per_tag:
                mm = per_tag[tid] * 1000
                flag = "✓" if mm <= tol_mm else "✗"
                print(f"    {flag} Tag {tid} ({_TAG_CORNER[tid]}): {mm:.2f} mm")
        print(f"\n  RMS: {rms * 1000:.2f} mm  →  {pass_str} (target ≤ {tol_mm:.1f} mm)")

    def _print_camera_results(
        self, residuals: Optional[Dict[int, float]]
    ) -> None:
        if residuals is None:
            print(
                "  Camera verification SKIPPED "
                "(workspace_board_node not running or verify_with_camera=false)."
            )
            return
        print("  Camera residuals (tag positions in workspace_frame vs model):")
        tol_mm = self._pass_tol * 1000
        for tid in _VISIT_ORDER:
            v = residuals.get(tid, float("nan"))
            if math.isnan(v):
                print(f"    ? Tag {tid}: not visible")
            else:
                flag = "✓" if v * 1000 <= tol_mm else "✗"
                print(f"    {flag} Tag {tid} ({_TAG_CORNER[tid]}): {v * 1000:.2f} mm (XY)")

    def _print_final_report(
        self,
        R: np.ndarray,
        t: np.ndarray,
        rms: float,
        latest: Path,
    ) -> None:
        roll, pitch, yaw = _rotation_to_rpy(R)
        pass_flag = rms <= self._pass_tol

        print("\n" + "=" * 70)
        print("  CALIBRATION RESULT")
        print("=" * 70)
        print(f"  workspace_frame in base_link:")
        print(f"    x     = {t[0]:.4f} m")
        print(f"    y     = {t[1]:.4f} m")
        print(f"    z     = {t[2]:.4f} m")
        print(f"    roll  = {math.degrees(roll):.3f} °")
        print(f"    pitch = {math.degrees(pitch):.3f} °")
        print(f"    yaw   = {math.degrees(yaw):.3f} °")
        print(f"  RMS residual: {rms * 1000:.2f} mm  →  {'PASS ✓' if pass_flag else 'FAIL ✗'}")
        print(f"\n  Saved: {latest}")
        print()
        print("  To apply for the current session (static TF already published):")
        print("    The calibration node is publishing base_link → workspace_frame.")
        print("    workspace_board_node must NOT be running concurrently.")
        print()
        print("  To apply after a restart:")
        print(f"    ros2 run moveit_robot_control workspace_frame_tf \\")
        print(f"        --ros-args --params-file {latest}")
        print()
        if not pass_flag:
            print(
                "  ⚠  Residual exceeds tolerance. Consider re-running calibration\n"
                "     after repositioning the board more carefully."
            )
        print("=" * 70)


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoardCalibrationNode()
    try:
        node.run()
    except (KeyboardInterrupt, RuntimeError) as exc:
        if isinstance(exc, RuntimeError):
            node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
