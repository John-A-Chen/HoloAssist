#!/usr/bin/env python3
"""Eye-to-hand calibration: solve the static transform between a fixed camera
and the robot base by observing an AprilTag on the end-effector from multiple
robot poses.

Modes
-----
sim      Auto-generates random robot poses, computes synthetic camera
         observations (ground-truth + Gaussian noise), solves, and compares
         to ground truth.  No real robot or camera needed.

hardware Interactive: user moves robot (freedrive / teach pendant / MoveIt),
         presses Enter to capture each sample.  Uses real apriltag_ros
         detections.

The solver uses OpenCV calibrateHandEye (Tsai, Park, Horaud, Daniilidis).
Result is published as a static TF and saved to YAML.
"""

from __future__ import annotations

import math
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# UR3e joint limits (radians) — tighter than URDF limits for safe random poses
_UR3E_JOINT_LIMITS = [
    (-math.pi, math.pi),       # shoulder_pan
    (-math.pi, -0.2),          # shoulder_lift  (keep arm roughly upright)
    (-math.pi, math.pi),       # elbow
    (-math.pi, math.pi),       # wrist_1
    (-math.pi, math.pi),       # wrist_2
    (-math.pi, math.pi),       # wrist_3
]

_UR3E_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def _pose_to_R_t(translation, rotation_quat_xyzw):
    """Convert translation + quaternion (x,y,z,w) to 3x3 rotation matrix + 3x1 tvec."""
    R = Rotation.from_quat(rotation_quat_xyzw).as_matrix()
    t = np.array(translation, dtype=np.float64).reshape(3, 1)
    return R, t


def _tf_to_R_t(ts: TransformStamped):
    tr = ts.transform.translation
    ro = ts.transform.rotation
    return _pose_to_R_t(
        [tr.x, tr.y, tr.z],
        [ro.x, ro.y, ro.z, ro.w],
    )


def _make_4x4(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    return T


def _decompose_4x4(T):
    return T[:3, :3].copy(), T[:3, 3].reshape(3, 1).copy()


class EyeHandCalibrationNode(Node):
    def __init__(self):
        super().__init__("eye_hand_calibration")

        # --- Parameters ---
        self.declare_parameter("mode", "sim")
        self.declare_parameter("num_samples", 20)

        # Tag on end-effector
        self.declare_parameter("tag_frame", "tag36h11:50")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")

        # Tag offset from tool0 (metres / radians)
        self.declare_parameter("tag_offset_x", 0.0)
        self.declare_parameter("tag_offset_y", 0.0)
        self.declare_parameter("tag_offset_z", 0.03)
        self.declare_parameter("tag_offset_roll", 0.0)
        self.declare_parameter("tag_offset_pitch", 0.0)
        self.declare_parameter("tag_offset_yaw", 0.0)

        # Sim-only: ground-truth camera pose in world frame
        self.declare_parameter("sim_camera_x", 0.15)
        self.declare_parameter("sim_camera_y", -0.45)
        self.declare_parameter("sim_camera_z", 0.55)
        self.declare_parameter("sim_camera_roll", 0.0)
        self.declare_parameter("sim_camera_pitch", math.radians(55))
        self.declare_parameter("sim_camera_yaw", math.radians(90))
        self.declare_parameter("sim_noise_pos_stddev", 0.002)
        self.declare_parameter("sim_noise_rot_stddev_deg", 0.5)
        self.declare_parameter("sim_settle_time", 1.5)

        # Solver
        self.declare_parameter("solver_method", "tsai")

        # Output
        self.declare_parameter("output_dir", "~/.holoassist/calibration")
        self.declare_parameter("output_parent_frame", "world")
        self.declare_parameter("output_child_frame", "camera_link")

        # Trajectory controller for sim
        self.declare_parameter("trajectory_topic", "/joint_trajectory_controller/joint_trajectory")

        # Read params
        self._mode = self.get_parameter("mode").value
        self._num_samples = int(self.get_parameter("num_samples").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._ee_frame = str(self.get_parameter("ee_frame").value)
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._tag_frame = str(self.get_parameter("tag_frame").value)

        # Tag offset from EE
        tag_r = self.get_parameter("tag_offset_roll").value
        tag_p = self.get_parameter("tag_offset_pitch").value
        tag_y = self.get_parameter("tag_offset_yaw").value
        self._T_ee_tag = _make_4x4(
            Rotation.from_euler("xyz", [tag_r, tag_p, tag_y]).as_matrix(),
            np.array([
                self.get_parameter("tag_offset_x").value,
                self.get_parameter("tag_offset_y").value,
                self.get_parameter("tag_offset_z").value,
            ]).reshape(3, 1),
        )

        method_str = str(self.get_parameter("solver_method").value).lower()
        self._solver_method_name = method_str

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Marker publisher for RViz
        self._marker_pub = self.create_publisher(MarkerArray, "~/calibration_markers", 10)
        self._status_pub = self.create_publisher(String, "~/status", 10)

        # Trajectory publisher for sim
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("trajectory_topic").value),
            10,
        )

        # Storage
        self._R_gripper2base_list = []
        self._t_gripper2base_list = []
        self._R_target2cam_list = []
        self._t_target2cam_list = []
        self._ee_positions = []  # for RViz markers

        # Sim ground truth
        if self._mode == "sim":
            self._build_sim_ground_truth()

        self.get_logger().info(
            f"Eye-to-hand calibration node started (mode={self._mode}, "
            f"samples={self._num_samples}, solver={method_str})"
        )

        # Start calibration in a background thread
        self._thread = threading.Thread(target=self._run_calibration, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    # Sim ground truth
    # ------------------------------------------------------------------

    def _build_sim_ground_truth(self):
        """Build ground-truth T(world → camera) for simulation."""
        cx = self.get_parameter("sim_camera_x").value
        cy = self.get_parameter("sim_camera_y").value
        cz = self.get_parameter("sim_camera_z").value
        cr = self.get_parameter("sim_camera_roll").value
        cp = self.get_parameter("sim_camera_pitch").value
        cyw = self.get_parameter("sim_camera_yaw").value

        R_cam = Rotation.from_euler("xyz", [cr, cp, cyw]).as_matrix()
        t_cam = np.array([cx, cy, cz]).reshape(3, 1)
        self._T_world_camera_gt = _make_4x4(R_cam, t_cam)

        self.get_logger().info(
            f"Sim ground-truth camera pose: "
            f"xyz=({cx:.3f}, {cy:.3f}, {cz:.3f}) "
            f"rpy_deg=({math.degrees(cr):.1f}, {math.degrees(cp):.1f}, {math.degrees(cyw):.1f})"
        )

        # Publish the ground-truth camera frame so it shows in RViz
        self._publish_camera_tf_gt()

    def _publish_camera_tf_gt(self):
        """Publish ground-truth camera TF for visualisation in sim."""
        R, t = _decompose_4x4(self._T_world_camera_gt)
        q = Rotation.from_matrix(R).as_quat()  # xyzw

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = "world"
        ts.child_frame_id = self._camera_frame + "_gt"
        ts.transform.translation.x = float(t[0])
        ts.transform.translation.y = float(t[1])
        ts.transform.translation.z = float(t[2])
        ts.transform.rotation.x = float(q[0])
        ts.transform.rotation.y = float(q[1])
        ts.transform.rotation.z = float(q[2])
        ts.transform.rotation.w = float(q[3])
        self._tf_static_broadcaster.sendTransform(ts)

    # ------------------------------------------------------------------
    # Robot motion (sim)
    # ------------------------------------------------------------------

    def _send_joint_trajectory(self, joint_positions: list[float], duration_s: float = 2.0):
        """Send a joint trajectory goal to the fake hardware controller."""
        msg = JointTrajectory()
        msg.joint_names = list(_UR3E_JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = list(joint_positions)
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s % 1) * 1e9))
        msg.points = [pt]
        self._traj_pub.publish(msg)

    def _generate_random_joint_config(self, rng: np.random.Generator) -> list[float]:
        """Generate a random joint configuration within safe limits."""
        return [
            float(rng.uniform(lo, hi))
            for lo, hi in _UR3E_JOINT_LIMITS
        ]

    # ------------------------------------------------------------------
    # Synthetic observation (sim)
    # ------------------------------------------------------------------

    def _compute_synthetic_observation(self, T_base_ee: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Given the EE pose in base_link, compute what the camera would see.

        Returns (R_target2cam, t_target2cam) with added noise.
        """
        # T(world → base_link) is identity in sim (robot at origin)
        T_world_base = np.eye(4)
        try:
            ts = self._tf_buffer.lookup_transform("world", self._base_frame, rclpy.time.Time())
            T_world_base = _make_4x4(*_tf_to_R_t(ts))
        except Exception:
            pass

        # T(world → tag) = T(world→base) × T(base→ee) × T(ee→tag)
        T_world_tag = T_world_base @ T_base_ee @ self._T_ee_tag

        # T(camera → tag) = inv(T(world→camera)) × T(world→tag)
        T_cam_tag = np.linalg.inv(self._T_world_camera_gt) @ T_world_tag

        R_target2cam, t_target2cam = _decompose_4x4(T_cam_tag)

        # Add noise
        pos_noise = float(self.get_parameter("sim_noise_pos_stddev").value)
        rot_noise = math.radians(float(self.get_parameter("sim_noise_rot_stddev_deg").value))

        rng = np.random.default_rng()
        t_target2cam += rng.normal(0, pos_noise, (3, 1))

        noise_rotvec = rng.normal(0, rot_noise, 3)
        R_noise = Rotation.from_rotvec(noise_rotvec).as_matrix()
        R_target2cam = R_noise @ R_target2cam

        return R_target2cam, t_target2cam

    # ------------------------------------------------------------------
    # Capture a single sample
    # ------------------------------------------------------------------

    def _capture_from_tf(self) -> Optional[tuple]:
        """Read EE and tag poses from TF tree. Returns (R_g2b, t_g2b, R_t2c, t_t2c) or None."""
        try:
            ts_ee = self._tf_buffer.lookup_transform(
                self._base_frame, self._ee_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Cannot look up {self._base_frame}→{self._ee_frame}: {e}")
            return None

        R_base_ee, t_base_ee = _tf_to_R_t(ts_ee)
        T_base_ee = _make_4x4(R_base_ee, t_base_ee)

        if self._mode == "sim":
            R_t2c, t_t2c = self._compute_synthetic_observation(T_base_ee)
        else:
            # Hardware: read tag detection from TF
            try:
                ts_tag = self._tf_buffer.lookup_transform(
                    self._camera_frame, self._tag_frame, rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Cannot look up {self._camera_frame}→{self._tag_frame}: {e}"
                )
                return None
            R_t2c, t_t2c = _tf_to_R_t(ts_tag)

        # For eye-to-hand: gripper2base = inv(base2gripper) = inv(base2ee)
        T_ee_base = np.linalg.inv(T_base_ee)
        R_g2b, t_g2b = _decompose_4x4(T_ee_base)

        # Store EE position for markers
        self._ee_positions.append(t_base_ee.flatten().tolist())

        return R_g2b, t_g2b, R_t2c, t_t2c

    # ------------------------------------------------------------------
    # Solver
    # ------------------------------------------------------------------

    def _solve(self):
        """Run OpenCV calibrateHandEye and return T(camera → base) as 4x4."""
        n = len(self._R_gripper2base_list)
        self.get_logger().info(f"Solving eye-to-hand calibration with {n} samples...")

        # Run all 4 methods and compare
        methods = {
            "tsai": cv2.CALIB_HAND_EYE_TSAI,
            "park": cv2.CALIB_HAND_EYE_PARK,
            "horaud": cv2.CALIB_HAND_EYE_HORAUD,
            "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
        }

        results = {}
        for name, method in methods.items():
            try:
                R_cam2base, t_cam2base = cv2.calibrateHandEye(
                    self._R_gripper2base_list,
                    self._t_gripper2base_list,
                    self._R_target2cam_list,
                    self._t_target2cam_list,
                    method=method,
                )
                results[name] = _make_4x4(R_cam2base, t_cam2base)
            except Exception as e:
                self.get_logger().warn(f"Method {name} failed: {e}")

        if not results:
            self.get_logger().error("All solvers failed!")
            return None

        # Select method
        method_str = self._solver_method_name
        if method_str == "auto" or method_str not in results:
            # Auto-select: pick the method whose translation has the smallest
            # norm deviation from the median across all methods (most consistent).
            translations = {k: _decompose_4x4(T)[1] for k, T in results.items()}
            if len(translations) >= 2:
                median_t = np.median(
                    np.hstack(list(translations.values())), axis=1, keepdims=True
                )
                chosen = min(
                    translations,
                    key=lambda k: float(np.linalg.norm(translations[k] - median_t)),
                )
            else:
                chosen = next(iter(results))
            T_cam2base = results[chosen]
            if method_str != "auto":
                self.get_logger().warn(f"Method {method_str} failed, auto-selected {chosen}")
        else:
            T_cam2base = results[method_str]
            chosen = method_str

        # Log all results for comparison
        self.get_logger().info("=== Calibration Results (all methods) ===")
        for name, T in results.items():
            R, t = _decompose_4x4(T)
            rpy = Rotation.from_matrix(R).as_euler("xyz", degrees=True)
            tag = " ◄ SELECTED" if name == chosen else ""
            self.get_logger().info(
                f"  {name}: xyz=({t[0,0]:.4f}, {t[1,0]:.4f}, {t[2,0]:.4f}) "
                f"rpy_deg=({rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}){tag}"
            )

        # T_cam2base gives camera → base_link.
        # We want world → camera for TF publishing.
        # world → camera = world → base_link × inv(camera → base_link)
        #                = world → base_link × base_link → camera
        #                = T_world_base × inv(T_cam2base)
        T_world_base = np.eye(4)
        try:
            ts = self._tf_buffer.lookup_transform("world", self._base_frame, rclpy.time.Time())
            T_world_base = _make_4x4(*_tf_to_R_t(ts))
        except Exception:
            pass

        T_world_camera = T_world_base @ np.linalg.inv(T_cam2base)

        # In sim, compare to ground truth
        if self._mode == "sim" and hasattr(self, "_T_world_camera_gt"):
            gt = self._T_world_camera_gt
            _, t_gt = _decompose_4x4(gt)
            _, t_solved = _decompose_4x4(T_world_camera)
            pos_err = np.linalg.norm(t_gt - t_solved)

            R_gt, _ = _decompose_4x4(gt)
            R_sol, _ = _decompose_4x4(T_world_camera)
            rot_err_deg = np.degrees(
                np.arccos(np.clip((np.trace(R_gt.T @ R_sol) - 1) / 2, -1, 1))
            )

            self.get_logger().info("=== Ground Truth Comparison ===")
            rpy_gt = Rotation.from_matrix(R_gt).as_euler("xyz", degrees=True)
            rpy_sol = Rotation.from_matrix(R_sol).as_euler("xyz", degrees=True)
            self.get_logger().info(
                f"  Ground truth: xyz=({t_gt[0,0]:.4f}, {t_gt[1,0]:.4f}, {t_gt[2,0]:.4f}) "
                f"rpy=({rpy_gt[0]:.2f}, {rpy_gt[1]:.2f}, {rpy_gt[2]:.2f})"
            )
            self.get_logger().info(
                f"  Solved:       xyz=({t_solved[0,0]:.4f}, {t_solved[1,0]:.4f}, {t_solved[2,0]:.4f}) "
                f"rpy=({rpy_sol[0]:.2f}, {rpy_sol[1]:.2f}, {rpy_sol[2]:.2f})"
            )
            self.get_logger().info(
                f"  Position error: {pos_err*1000:.1f} mm  |  Rotation error: {rot_err_deg:.2f}°"
            )

            verdict = "PASS" if pos_err < 0.01 and rot_err_deg < 2.0 else "CHECK"
            self.get_logger().info(f"  Verdict: {verdict}")

        return T_world_camera, T_cam2base

    # ------------------------------------------------------------------
    # Save and publish result
    # ------------------------------------------------------------------

    def _publish_result(self, T_world_camera: np.ndarray):
        """Publish the solved camera pose as a static TF."""
        R, t = _decompose_4x4(T_world_camera)
        q = Rotation.from_matrix(R).as_quat()

        parent = str(self.get_parameter("output_parent_frame").value)
        child = str(self.get_parameter("output_child_frame").value)

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = parent
        ts.child_frame_id = child
        ts.transform.translation.x = float(t[0])
        ts.transform.translation.y = float(t[1])
        ts.transform.translation.z = float(t[2])
        ts.transform.rotation.x = float(q[0])
        ts.transform.rotation.y = float(q[1])
        ts.transform.rotation.z = float(q[2])
        ts.transform.rotation.w = float(q[3])
        self._tf_static_broadcaster.sendTransform(ts)

        self.get_logger().info(f"Published static TF: {parent} → {child}")

    def _save_result(self, T_world_camera: np.ndarray, T_cam2base: np.ndarray):
        """Save calibration result to YAML."""
        output_dir = os.path.expanduser(str(self.get_parameter("output_dir").value))
        os.makedirs(output_dir, exist_ok=True)

        R_wc, t_wc = _decompose_4x4(T_world_camera)
        rpy_wc = Rotation.from_matrix(R_wc).as_euler("xyz")
        q_wc = Rotation.from_matrix(R_wc).as_quat()

        R_cb, t_cb = _decompose_4x4(T_cam2base)
        rpy_cb = Rotation.from_matrix(R_cb).as_euler("xyz")
        q_cb = Rotation.from_matrix(R_cb).as_quat()

        parent = str(self.get_parameter("output_parent_frame").value)
        child = str(self.get_parameter("output_child_frame").value)

        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        yaml_content = f"""# Eye-to-hand calibration result
# Generated: {stamp}
# Solver: {self.get_parameter('solver_method').value}
# Samples: {len(self._R_gripper2base_list)}
# Mode: {self._mode}

# Static TF: {parent} → {child}
eye_hand_calibration:
  ros__parameters:
    parent_frame: "{parent}"
    child_frame: "{child}"
    x_m: {float(t_wc[0]):.6f}
    y_m: {float(t_wc[1]):.6f}
    z_m: {float(t_wc[2]):.6f}
    roll_rad: {float(rpy_wc[0]):.6f}
    pitch_rad: {float(rpy_wc[1]):.6f}
    yaw_rad: {float(rpy_wc[2]):.6f}
    quaternion_xyzw: [{q_wc[0]:.6f}, {q_wc[1]:.6f}, {q_wc[2]:.6f}, {q_wc[3]:.6f}]

# Camera → base_link (the raw solver output)
camera_to_base:
  x_m: {float(t_cb[0]):.6f}
  y_m: {float(t_cb[1]):.6f}
  z_m: {float(t_cb[2]):.6f}
  roll_rad: {float(rpy_cb[0]):.6f}
  pitch_rad: {float(rpy_cb[1]):.6f}
  yaw_rad: {float(rpy_cb[2]):.6f}
  quaternion_xyzw: [{q_cb[0]:.6f}, {q_cb[1]:.6f}, {q_cb[2]:.6f}, {q_cb[3]:.6f}]
"""

        latest_path = os.path.join(output_dir, "eye_hand_calibration_latest.yaml")
        stamped_path = os.path.join(output_dir, f"eye_hand_calibration_{stamp}.yaml")

        for path in [latest_path, stamped_path]:
            with open(path, "w") as f:
                f.write(yaml_content)

        self.get_logger().info(f"Saved calibration to:\n  {latest_path}\n  {stamped_path}")

    # ------------------------------------------------------------------
    # RViz markers
    # ------------------------------------------------------------------

    def _publish_markers(self, T_world_camera: Optional[np.ndarray] = None):
        """Publish RViz markers showing sampled EE positions and camera."""
        ma = MarkerArray()

        # Sampled EE positions as small spheres
        for i, pos in enumerate(self._ee_positions):
            m = Marker()
            m.header.frame_id = self._base_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "ee_samples"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = pos[0]
            m.pose.position.y = pos[1]
            m.pose.position.z = pos[2]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.015
            m.color.r = 0.2
            m.color.g = 0.8
            m.color.b = 0.2
            m.color.a = 1.0
            m.lifetime.sec = 0
            ma.markers.append(m)

        # Camera frustum indicator
        if T_world_camera is not None:
            R, t = _decompose_4x4(T_world_camera)
            q = Rotation.from_matrix(R).as_quat()

            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "camera_solved"
            m.id = 0
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(t[0])
            m.pose.position.y = float(t[1])
            m.pose.position.z = float(t[2])
            m.pose.orientation.x = float(q[0])
            m.pose.orientation.y = float(q[1])
            m.pose.orientation.z = float(q[2])
            m.pose.orientation.w = float(q[3])
            m.scale.x = 0.08
            m.scale.y = 0.04
            m.scale.z = 0.04
            m.color.r = 0.0
            m.color.g = 0.5
            m.color.b = 1.0
            m.color.a = 0.8
            m.lifetime.sec = 0
            ma.markers.append(m)

            # Arrow showing camera Z-axis (viewing direction)
            z_axis = R[:, 2]
            m2 = Marker()
            m2.header.frame_id = "world"
            m2.header.stamp = self.get_clock().now().to_msg()
            m2.ns = "camera_solved"
            m2.id = 1
            m2.type = Marker.ARROW
            m2.action = Marker.ADD
            from geometry_msgs.msg import Point
            p0 = Point(x=float(t[0]), y=float(t[1]), z=float(t[2]))
            p1 = Point(
                x=float(t[0] + z_axis[0] * 0.3),
                y=float(t[1] + z_axis[1] * 0.3),
                z=float(t[2] + z_axis[2] * 0.3),
            )
            m2.points = [p0, p1]
            m2.scale.x = 0.01
            m2.scale.y = 0.02
            m2.color.r = 0.0
            m2.color.g = 0.5
            m2.color.b = 1.0
            m2.color.a = 1.0
            m2.lifetime.sec = 0
            ma.markers.append(m2)

        if ma.markers:
            self._marker_pub.publish(ma)

    # ------------------------------------------------------------------
    # Main calibration loop
    # ------------------------------------------------------------------

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _run_calibration(self):
        """Main calibration routine — runs in a background thread."""
        # Wait for TF to be ready
        self.get_logger().info("Waiting for TF tree...")
        time.sleep(3.0)

        if self._mode == "sim":
            self._run_sim_calibration()
        else:
            self._run_hardware_calibration()

    def _run_sim_calibration(self):
        settle_time = float(self.get_parameter("sim_settle_time").value)
        rng = np.random.default_rng(42)

        self.get_logger().info(
            f"=== SIM MODE: collecting {self._num_samples} samples ==="
        )

        for i in range(self._num_samples):
            joints = self._generate_random_joint_config(rng)
            self._send_joint_trajectory(joints, duration_s=1.5)

            self._publish_status(f"Moving to pose {i+1}/{self._num_samples}...")
            time.sleep(settle_time)

            result = self._capture_from_tf()
            if result is None:
                self.get_logger().warn(f"Sample {i+1}: TF lookup failed, skipping")
                continue

            R_g2b, t_g2b, R_t2c, t_t2c = result
            self._R_gripper2base_list.append(R_g2b)
            self._t_gripper2base_list.append(t_g2b)
            self._R_target2cam_list.append(R_t2c)
            self._t_target2cam_list.append(t_t2c)

            self._publish_markers()
            self.get_logger().info(
                f"Sample {i+1}/{self._num_samples} captured "
                f"(EE at [{t_g2b[0,0]:.3f}, {t_g2b[1,0]:.3f}, {t_g2b[2,0]:.3f}])"
            )

        if len(self._R_gripper2base_list) < 3:
            self.get_logger().error("Need at least 3 samples to solve. Aborting.")
            return

        result = self._solve()
        if result is None:
            return

        T_world_camera, T_cam2base = result
        self._publish_result(T_world_camera)
        self._save_result(T_world_camera, T_cam2base)
        self._publish_markers(T_world_camera)

        self._publish_status("Calibration complete!")
        self.get_logger().info(
            "=== Calibration complete. Node stays alive to hold TF. ==="
        )

    def _run_hardware_calibration(self):
        self.get_logger().info(
            "=== HARDWARE MODE ===\n"
            f"  Tag frame: {self._tag_frame}\n"
            f"  EE frame:  {self._ee_frame}\n"
            f"  Camera:    {self._camera_frame}\n"
            f"  Target:    {self._num_samples} samples\n\n"
            "Move the robot to different poses where the camera can see the tag.\n"
            "Call the ~/capture service or publish to ~/capture_trigger to capture.\n"
            "Call ~/solve when done."
        )

        # Service triggers for hardware mode
        from std_srvs.srv import Trigger

        self.create_service(Trigger, "~/capture", self._capture_service_cb)
        self.create_service(Trigger, "~/solve", self._solve_service_cb)

        self._publish_status(
            f"Ready. 0/{self._num_samples} samples. "
            "Call ~/capture to capture, ~/solve to solve."
        )

    def _capture_service_cb(self, request, response):
        result = self._capture_from_tf()
        if result is None:
            response.success = False
            response.message = "TF lookup failed — is the tag visible?"
            return response

        R_g2b, t_g2b, R_t2c, t_t2c = result
        self._R_gripper2base_list.append(R_g2b)
        self._t_gripper2base_list.append(t_g2b)
        self._R_target2cam_list.append(R_t2c)
        self._t_target2cam_list.append(t_t2c)

        n = len(self._R_gripper2base_list)
        self._publish_markers()

        msg = f"Sample {n} captured. {max(0, self._num_samples - n)} more recommended."
        self._publish_status(msg)
        self.get_logger().info(msg)

        response.success = True
        response.message = msg
        return response

    def _solve_service_cb(self, request, response):
        if len(self._R_gripper2base_list) < 3:
            response.success = False
            response.message = f"Need at least 3 samples (have {len(self._R_gripper2base_list)})"
            return response

        result = self._solve()
        if result is None:
            response.success = False
            response.message = "Solver failed"
            return response

        T_world_camera, T_cam2base = result
        self._publish_result(T_world_camera)
        self._save_result(T_world_camera, T_cam2base)
        self._publish_markers(T_world_camera)

        response.success = True
        response.message = "Calibration solved and published"
        return response


def main():
    rclpy.init()
    node = EyeHandCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
