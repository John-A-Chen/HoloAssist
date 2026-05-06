#!/usr/bin/env python3
"""Publish world → camera_link by snapping the detected workspace_frame to its
known position in base_link.

Instead of a hand-tuned static camera mount TF, this node:
  1. Reads the known base_link → workspace_frame from parameters (same defaults
     as workspace_frame_tf, or loaded from a calibration YAML).
  2. Looks up camera_color_optical_frame → workspace_frame from TF (published
     by workspace_board_node once the board AprilTags are visible).
  3. Computes and publishes world → camera_link so the two align exactly.

Math:
  T(world→camera_link) = T(world→base_link)
                        × T(base_link→workspace_frame)   [from params]
                        × inv(T(camera_opt→workspace_frame))  [from TF]
                        × inv(T(camera_link→camera_opt))      [from TF]

The board can be temporarily hidden — the node republishes the last valid
camera TF at the configured rate so the TF tree stays connected.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf_transformations import euler_matrix, quaternion_from_matrix, quaternion_matrix


def _ts_to_matrix(ts: TransformStamped) -> np.ndarray:
    rot = ts.transform.rotation
    mat = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    mat[0, 3] = ts.transform.translation.x
    mat[1, 3] = ts.transform.translation.y
    mat[2, 3] = ts.transform.translation.z
    return mat


def _matrix_to_ts(mat: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    ts = TransformStamped()
    ts.header.stamp = stamp
    ts.header.frame_id = parent
    ts.child_frame_id = child
    ts.transform.translation.x = float(mat[0, 3])
    ts.transform.translation.y = float(mat[1, 3])
    ts.transform.translation.z = float(mat[2, 3])
    q = quaternion_from_matrix(mat)
    ts.transform.rotation.x = float(q[0])
    ts.transform.rotation.y = float(q[1])
    ts.transform.rotation.z = float(q[2])
    ts.transform.rotation.w = float(q[3])
    return ts


class WorkspaceAlignCameraTfNode(Node):
    def __init__(self) -> None:
        super().__init__("holoassist_workspace_align_camera_tf")

        # Known workspace_frame position in base_link.
        # Defaults match workspace_frame_tf hardcoded values — override with
        # calibration YAML or explicit launch args.
        self.declare_parameter("workspace_x_m", -0.10)
        self.declare_parameter("workspace_y_m", -0.314)
        self.declare_parameter("workspace_z_m", 0.015)
        self.declare_parameter("workspace_roll_rad", 0.0)
        self.declare_parameter("workspace_pitch_rad", 0.0)
        self.declare_parameter("workspace_yaw_rad", 0.0)

        # TF frame names.
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("camera_link_frame", "camera_link")
        self.declare_parameter("camera_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("workspace_frame", "workspace_frame")

        self.declare_parameter("publish_rate_hz", 10.0)

        wx = float(self.get_parameter("workspace_x_m").value)
        wy = float(self.get_parameter("workspace_y_m").value)
        wz = float(self.get_parameter("workspace_z_m").value)
        wr = float(self.get_parameter("workspace_roll_rad").value)
        wp = float(self.get_parameter("workspace_pitch_rad").value)
        wyaw = float(self.get_parameter("workspace_yaw_rad").value)

        self._world_frame = str(self.get_parameter("world_frame").value)
        self._robot_frame = str(self.get_parameter("robot_frame").value)
        self._camera_link_frame = str(self.get_parameter("camera_link_frame").value)
        self._camera_optical_frame = str(self.get_parameter("camera_optical_frame").value)
        self._workspace_frame_id = str(self.get_parameter("workspace_frame").value)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))

        # T_base_workspace from parameters (constant).
        self._T_base_workspace: np.ndarray = euler_matrix(wr, wp, wyaw)
        self._T_base_workspace[0, 3] = wx
        self._T_base_workspace[1, 3] = wy
        self._T_base_workspace[2, 3] = wz

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._last_camera_tf: Optional[TransformStamped] = None
        self._board_seen = False

        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            "workspace_align_camera_tf started — "
            f"known workspace_frame in {self._robot_frame}: "
            f"xyz=({wx:.3f}, {wy:.3f}, {wz:.3f}) "
            f"rpy_deg=({math.degrees(wr):.1f}, {math.degrees(wp):.1f}, {math.degrees(wyaw):.1f}). "
            "Waiting for board AprilTags to be visible..."
        )

    def _on_timer(self) -> None:
        now = self.get_clock().now().to_msg()
        t = rclpy.time.Time()

        try:
            # world → base_link (robot RSP)
            ts_wb = self._tf_buffer.lookup_transform(self._world_frame, self._robot_frame, t)
            T_world_base = _ts_to_matrix(ts_wb)

            # camera_optical_frame → workspace_frame (workspace_board_node, dynamic)
            ts_cw = self._tf_buffer.lookup_transform(
                self._camera_optical_frame, self._workspace_frame_id, t
            )
            T_camopt_workspace = _ts_to_matrix(ts_cw)

            # camera_link → camera_optical_frame (RealSense driver, static)
            ts_lc = self._tf_buffer.lookup_transform(
                self._camera_link_frame, self._camera_optical_frame, t
            )
            T_camlink_camopt = _ts_to_matrix(ts_lc)

        except Exception:
            # Board not yet visible or TF not ready — republish last known TF.
            if self._last_camera_tf is not None:
                self._last_camera_tf.header.stamp = now
                self._tf_broadcaster.sendTransform(self._last_camera_tf)
            return

        # T_world_camlink = T_world_base × T_base_workspace
        #                 × inv(T_camopt_workspace) × inv(T_camlink_camopt)
        T_world_workspace = T_world_base @ self._T_base_workspace
        T_world_camlink = (
            T_world_workspace
            @ np.linalg.inv(T_camopt_workspace)
            @ np.linalg.inv(T_camlink_camopt)
        )

        ts = _matrix_to_ts(T_world_camlink, self._world_frame, self._camera_link_frame, now)
        self._last_camera_tf = ts
        self._tf_broadcaster.sendTransform(ts)

        if not self._board_seen:
            self._board_seen = True
            self.get_logger().info(
                "Board detected — camera TF locked. "
                f"world→{self._camera_link_frame} computed and publishing."
            )


def main() -> None:
    rclpy.init()
    node = WorkspaceAlignCameraTfNode()
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
