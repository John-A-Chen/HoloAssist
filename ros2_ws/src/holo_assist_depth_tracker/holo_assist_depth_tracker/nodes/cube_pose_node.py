#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker

from holo_assist_depth_tracker.utils.apriltag import candidate_tag_frame_names
from holo_assist_depth_tracker.utils.math3d import (
    normalize,
    quaternion_from_rotation_matrix,
    rotation_matrix_from_quaternion,
)


class CubePoseNode(Node):
    """Estimate 4 independent cube poses from grouped AprilTag IDs."""

    AXIS_SEQUENCE = [
        np.array([1.0, 0.0, 0.0], dtype=np.float64),
        np.array([-1.0, 0.0, 0.0], dtype=np.float64),
        np.array([0.0, 1.0, 0.0], dtype=np.float64),
        np.array([0.0, -1.0, 0.0], dtype=np.float64),
        np.array([0.0, 0.0, 1.0], dtype=np.float64),
        np.array([0.0, 0.0, -1.0], dtype=np.float64),
    ]

    def __init__(self) -> None:
        super().__init__("holoassist_cube_pose")

        self.declare_parameter("detections_topic", "/detections_all")
        self.declare_parameter("workspace_frame", "workspace_frame")
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("cube_size_m", 0.045)
        self.declare_parameter("detections_timeout_s", 1.0)
        self.declare_parameter("tf_lookup_timeout_s", 0.05)
        self.declare_parameter("timer_hz", 20.0)
        self.declare_parameter("publish_cube_tf", True)
        self.declare_parameter("publish_cube_markers", True)
        self.declare_parameter("marker_alpha", 0.8)
        self.declare_parameter("legacy_cube_pose_topic", "/holoassist/perception/april_cube_pose")

        self.declare_parameter("cube_1_ids", [10, 11, 12, 13, 14, 15])
        self.declare_parameter("cube_2_ids", [16, 17, 18, 19, 20, 21])
        self.declare_parameter("cube_3_ids", [22, 23, 24, 25, 26, 27])
        self.declare_parameter("cube_4_ids", [28, 29, 30, 31, 32, 33])

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.workspace_frame = str(self.get_parameter("workspace_frame").value)
        self.tag_family = str(self.get_parameter("tag_family").value)
        self.cube_size_m = max(0.01, float(self.get_parameter("cube_size_m").value))
        self.detections_timeout_s = max(0.05, float(self.get_parameter("detections_timeout_s").value))
        self.tf_lookup_timeout_s = max(0.0, float(self.get_parameter("tf_lookup_timeout_s").value))
        self.timer_hz = max(1.0, float(self.get_parameter("timer_hz").value))
        self.publish_cube_tf = bool(self.get_parameter("publish_cube_tf").value)
        self.publish_cube_markers = bool(self.get_parameter("publish_cube_markers").value)
        self.marker_alpha = max(0.05, min(1.0, float(self.get_parameter("marker_alpha").value)))
        self.legacy_cube_pose_topic = str(self.get_parameter("legacy_cube_pose_topic").value)

        self.cube_groups: List[List[int]] = [
            [int(v) for v in self.get_parameter("cube_1_ids").value],
            [int(v) for v in self.get_parameter("cube_2_ids").value],
            [int(v) for v in self.get_parameter("cube_3_ids").value],
            [int(v) for v in self.get_parameter("cube_4_ids").value],
        ]

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pose_pubs = []
        self.marker_pubs = []
        self.status_pubs = []
        self.cube_tf_frames = []
        for idx in range(4):
            n = idx + 1
            self.pose_pubs.append(
                self.create_publisher(
                    PoseStamped,
                    f"/holoassist/perception/april_cube_{n}_pose",
                    reliable_qos,
                )
            )
            self.marker_pubs.append(
                self.create_publisher(
                    Marker,
                    f"/holoassist/perception/april_cube_{n}_marker",
                    reliable_qos,
                )
            )
            self.status_pubs.append(
                self.create_publisher(
                    String,
                    f"/holoassist/perception/april_cube_{n}_status",
                    reliable_qos,
                )
            )
            self.cube_tf_frames.append(f"apriltag_cube_{n}")

        self.legacy_pose_pub = self.create_publisher(PoseStamped, self.legacy_cube_pose_topic, reliable_qos)

        self._detections_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._on_detections,
            reliable_qos,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._latest_visible_ids: set[int] = set()
        self._latest_detection_stamp: Optional[rclpy.time.Time] = None

        self._last_normal_sign: List[float] = [1.0, 1.0, 1.0, 1.0]
        self._marker_active: List[bool] = [False, False, False, False]

        self._timer = self.create_timer(1.0 / self.timer_hz, self._on_timer)

        self.get_logger().info(
            "cube pose node started detections=%s workspace_frame=%s cube_size=%.3f groups=%s"
            % (self.detections_topic, self.workspace_frame, self.cube_size_m, self.cube_groups)
        )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self._latest_visible_ids = {int(det.id) for det in msg.detections}
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        self._latest_detection_stamp = stamp

    def _on_timer(self) -> None:
        if self._latest_detection_stamp is None:
            for idx in range(4):
                self._publish_status(idx, "waiting for detections")
            return

        age_s = (self.get_clock().now() - self._latest_detection_stamp).nanoseconds / 1e9
        if age_s > self.detections_timeout_s:
            for idx in range(4):
                self._publish_status(idx, f"stale detections age={age_s:.2f}s")
                self._delete_marker(idx)
            return

        first_cube_pose: Optional[PoseStamped] = None
        for idx, tag_ids in enumerate(self.cube_groups):
            observations = self._collect_observations(tag_ids)
            if not observations:
                self._publish_status(idx, "no visible cube tags")
                self._delete_marker(idx)
                continue

            solved = self._solve_pose(observations, idx)
            if solved is None:
                self._publish_status(idx, "failed to solve cube pose")
                self._delete_marker(idx)
                continue

            center, rot_w_c, pos_resid_m, ori_resid_rad, used_ids = solved
            qx, qy, qz, qw = quaternion_from_rotation_matrix(rot_w_c)
            pose = self._publish_cube_outputs(idx, center, (qx, qy, qz, qw))

            if idx == 0:
                first_cube_pose = pose

            self._publish_status(
                idx,
                "ok tags=%s pos_resid=%.1fmm ori_resid=%.1fdeg"
                % (used_ids, 1000.0 * pos_resid_m, math.degrees(ori_resid_rad)),
            )

        if first_cube_pose is not None:
            self.legacy_pose_pub.publish(first_cube_pose)

    def _collect_observations(self, tag_ids: List[int]) -> List[Dict[str, object]]:
        observations: List[Dict[str, object]] = []
        visible_ordered = [tag_id for tag_id in tag_ids if tag_id in self._latest_visible_ids]

        for tag_id in visible_ordered:
            tf_msg = self._lookup_tag_transform(tag_id)
            if tf_msg is None:
                continue

            t = tf_msg.transform.translation
            r = tf_msg.transform.rotation
            point = np.array([float(t.x), float(t.y), float(t.z)], dtype=np.float64)
            rot_w_t = rotation_matrix_from_quaternion(float(r.x), float(r.y), float(r.z), float(r.w))

            raw_normal = normalize(rot_w_t[:, 2].astype(np.float64))
            if raw_normal is None:
                continue

            observations.append(
                {
                    "id": int(tag_id),
                    "point": point,
                    "raw_normal": raw_normal,
                    "cube_axis": self._cube_axis_for_id(tag_ids, tag_id),
                }
            )

        return observations

    def _lookup_tag_transform(self, tag_id: int) -> Optional[TransformStamped]:
        for candidate in candidate_tag_frame_names(self.tag_family, tag_id):
            try:
                return self._tf_buffer.lookup_transform(
                    self.workspace_frame,
                    candidate,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_lookup_timeout_s),
                )
            except TransformException:
                continue
        return None

    def _cube_axis_for_id(self, group_ids: List[int], tag_id: int) -> np.ndarray:
        try:
            idx = group_ids.index(tag_id)
        except ValueError:
            idx = 0
        idx = max(0, min(5, idx))
        return np.asarray(self.AXIS_SEQUENCE[idx], dtype=np.float64)

    def _solve_pose(
        self,
        observations: List[Dict[str, object]],
        cube_idx: int,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, float, float, List[int]]]:
        if not observations:
            return None

        half = 0.5 * float(self.cube_size_m)
        preferred = 1.0 if self._last_normal_sign[cube_idx] >= 0.0 else -1.0
        sign_candidates = [preferred, -preferred]

        best = None
        for sign in sign_candidates:
            normals = []
            centers = []
            for obs in observations:
                raw = np.asarray(obs["raw_normal"], dtype=np.float64)
                normal = normalize(sign * raw)
                if normal is None:
                    normal = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                normals.append(normal)

                point = np.asarray(obs["point"], dtype=np.float64)
                centers.append(point - half * normal)

            center = np.mean(np.asarray(centers, dtype=np.float64), axis=0)
            pos_resid = float(np.mean([np.linalg.norm(c - center) for c in centers]))
            rot_w_c, ori_resid = self._estimate_rotation(observations, normals)
            score = pos_resid + 0.05 * ori_resid

            if best is None or score < best["score"]:
                best = {
                    "center": center,
                    "rotation": rot_w_c,
                    "pos_resid": pos_resid,
                    "ori_resid": ori_resid,
                    "score": score,
                    "sign": sign,
                }

        if best is None:
            return None

        self._last_normal_sign[cube_idx] = float(best["sign"])
        return (
            np.asarray(best["center"], dtype=np.float64),
            np.asarray(best["rotation"], dtype=np.float64),
            float(best["pos_resid"]),
            float(best["ori_resid"]),
            sorted(int(obs["id"]) for obs in observations),
        )

    def _estimate_rotation(
        self,
        observations: List[Dict[str, object]],
        normals: List[np.ndarray],
    ) -> Tuple[np.ndarray, float]:
        if len(observations) >= 2:
            m = np.zeros((3, 3), dtype=np.float64)
            for obs, normal in zip(observations, normals):
                cube_axis = np.asarray(obs["cube_axis"], dtype=np.float64)
                m += np.outer(normal, cube_axis)

            u, _, vt = np.linalg.svd(m)
            rot = u @ vt
            if float(np.linalg.det(rot)) < 0.0:
                u[:, -1] *= -1.0
                rot = u @ vt

            ori_err = 0.0
            for obs, normal in zip(observations, normals):
                cube_axis = np.asarray(obs["cube_axis"], dtype=np.float64)
                projected = normalize(rot @ cube_axis)
                if projected is None:
                    continue
                dot = float(np.clip(np.dot(projected, normal), -1.0, 1.0))
                ori_err += math.acos(dot)
            ori_err /= max(1, len(normals))
            return rot, ori_err

        obs = observations[0]
        normal = normals[0]
        cube_axis = normalize(np.asarray(obs["cube_axis"], dtype=np.float64))
        if cube_axis is None:
            cube_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        cross = np.cross(cube_axis, normal)
        cross_norm = float(np.linalg.norm(cross))
        dot = float(np.clip(np.dot(cube_axis, normal), -1.0, 1.0))
        if cross_norm < 1e-9:
            if dot >= 0.0:
                rot_align = np.eye(3, dtype=np.float64)
            else:
                axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
                if abs(float(cube_axis[0])) > 0.9:
                    axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
                axis = normalize(axis - float(np.dot(axis, cube_axis)) * cube_axis)
                if axis is None:
                    axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
                rot_align = self._axis_angle(axis, math.pi)
        else:
            axis = cross / cross_norm
            angle = math.atan2(cross_norm, dot)
            rot_align = self._axis_angle(axis, angle)

        helper_ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(np.dot(helper_ref, normal))) > 0.9:
            helper_ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        helper_tangent = normalize(helper_ref - float(np.dot(helper_ref, normal)) * normal)
        if helper_tangent is None:
            helper_tangent = np.array([0.0, 1.0, 0.0], dtype=np.float64)

        tangent_proj = normalize(rot_align @ np.array([1.0, 0.0, 0.0], dtype=np.float64))
        if tangent_proj is None:
            tangent_proj = helper_tangent

        dot_t = float(np.clip(np.dot(tangent_proj, helper_tangent), -1.0, 1.0))
        sign = float(np.dot(np.cross(tangent_proj, helper_tangent), normal))
        yaw = math.atan2(sign, dot_t)
        rot_yaw = self._axis_angle(normal, yaw)
        rot = rot_yaw @ rot_align
        return rot, 0.0

    def _axis_angle(self, axis: np.ndarray, angle: float) -> np.ndarray:
        x, y, z = axis
        c = math.cos(angle)
        s = math.sin(angle)
        t = 1.0 - c
        return np.array(
            [
                [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
            ],
            dtype=np.float64,
        )

    def _publish_cube_outputs(
        self,
        cube_idx: int,
        center: np.ndarray,
        quat_xyzw: Tuple[float, float, float, float],
    ) -> PoseStamped:
        qx, qy, qz, qw = quat_xyzw
        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.workspace_frame
        pose_msg.header.stamp = stamp
        pose_msg.pose.position.x = float(center[0])
        pose_msg.pose.position.y = float(center[1])
        pose_msg.pose.position.z = float(center[2])
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        self.pose_pubs[cube_idx].publish(pose_msg)

        if self.publish_cube_tf:
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = self.workspace_frame
            tf_msg.header.stamp = stamp
            tf_msg.child_frame_id = self.cube_tf_frames[cube_idx]
            tf_msg.transform.translation.x = float(center[0])
            tf_msg.transform.translation.y = float(center[1])
            tf_msg.transform.translation.z = float(center[2])
            tf_msg.transform.rotation.x = float(qx)
            tf_msg.transform.rotation.y = float(qy)
            tf_msg.transform.rotation.z = float(qz)
            tf_msg.transform.rotation.w = float(qw)
            self._tf_broadcaster.sendTransform(tf_msg)

        if self.publish_cube_markers:
            marker = Marker()
            marker.header.frame_id = self.workspace_frame
            marker.header.stamp = stamp
            marker.ns = f"holoassist_april_cube_{cube_idx + 1}"
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose_msg.pose
            marker.scale.x = float(self.cube_size_m)
            marker.scale.y = float(self.cube_size_m)
            marker.scale.z = float(self.cube_size_m)
            marker.color.r = 0.10
            marker.color.g = 0.85
            marker.color.b = 0.35
            marker.color.a = float(self.marker_alpha)
            self.marker_pubs[cube_idx].publish(marker)
            self._marker_active[cube_idx] = True

        return pose_msg

    def _publish_status(self, cube_idx: int, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pubs[cube_idx].publish(msg)

    def _delete_marker(self, cube_idx: int) -> None:
        if not self._marker_active[cube_idx]:
            return
        marker = Marker()
        marker.header.frame_id = self.workspace_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"holoassist_april_cube_{cube_idx + 1}"
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pubs[cube_idx].publish(marker)
        self._marker_active[cube_idx] = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CubePoseNode()
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
