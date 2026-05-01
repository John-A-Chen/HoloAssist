#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from holo_assist_depth_tracker.utils.apriltag import default_tag_frame_name
from holo_assist_depth_tracker.utils.math3d import quaternion_from_rotation_matrix


def _rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _transform_point(rot_parent_child: np.ndarray, trans_parent_child: np.ndarray, point_child: np.ndarray) -> np.ndarray:
    return trans_parent_child + rot_parent_child @ point_child


def _parse_vec(name: str, values: Sequence[float], expected_len: int) -> np.ndarray:
    if len(values) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} values, got {len(values)}")
    return np.asarray([float(v) for v in values], dtype=np.float64)


def _parse_cube_centers(values: Sequence[float]) -> List[np.ndarray]:
    if len(values) % 3 != 0 or len(values) == 0:
        raise ValueError("cube_centers_board_xyz must contain 3*N values")
    out: List[np.ndarray] = []
    for i in range(0, len(values), 3):
        out.append(np.asarray([float(values[i]), float(values[i + 1]), float(values[i + 2])], dtype=np.float64))
    return out


@dataclass
class TagPose:
    tag_id: int
    center_c: np.ndarray
    rot_c_t: np.ndarray


class SimApriltagPublisherNode(Node):
    """Publish synthetic AprilTag detections + TF for full-stack simulation."""

    def __init__(self) -> None:
        super().__init__("holoassist_sim_apriltag_publisher")

        self.declare_parameter("detections_topic", "/detections_all")
        self.declare_parameter("detections_fallback_topic", "/detections")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("tag_family", "36h11")

        self.declare_parameter("board_tag_ids", [0, 1, 2, 3])
        self.declare_parameter("board_width_m", 0.700)
        self.declare_parameter("board_depth_m", 0.500)
        self.declare_parameter("board_tag_size_m", 0.032)
        self.declare_parameter("board_tag_center_edge_offset_m", 0.016)
        self.declare_parameter("board_center_in_camera_m", [0.0, 0.06, 0.95])
        self.declare_parameter("board_rpy_deg", [70.0, 0.0, 0.0])

        self.declare_parameter("cube_tag_size_m", 0.032)
        self.declare_parameter("cube_face_offset_m", 0.020)
        self.declare_parameter("april_cube_1_tag_ids", [10, 11, 12, 13, 14, 15])
        self.declare_parameter("april_cube_2_tag_ids", [16, 17, 18, 19, 20, 21])
        self.declare_parameter("april_cube_3_tag_ids", [22, 23, 24, 25, 26, 27])
        self.declare_parameter("april_cube_4_tag_ids", [28, 29, 30, 31, 32, 33])
        self.declare_parameter("cube_centers_board_xyz", [
            0.20, 0.14, 0.020,
            0.50, 0.16, 0.020,
            0.23, 0.34, 0.020,
            0.48, 0.33, 0.020,
        ])

        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("fx", 462.0)
        self.declare_parameter("fy", 462.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("corner_scale_px_m", 900.0)
        self.declare_parameter("corner_min_px", 6.0)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.detections_fallback_topic = str(self.get_parameter("detections_fallback_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.tag_family = str(self.get_parameter("tag_family").value)

        self.board_tag_ids = [int(v) for v in self.get_parameter("board_tag_ids").value]
        if len(self.board_tag_ids) != 4:
            self.get_logger().warn("board_tag_ids must have 4 entries; falling back to [0, 1, 2, 3]")
            self.board_tag_ids = [0, 1, 2, 3]

        self.board_width_m = max(0.10, float(self.get_parameter("board_width_m").value))
        self.board_depth_m = max(0.10, float(self.get_parameter("board_depth_m").value))
        self.board_tag_size_m = max(0.005, float(self.get_parameter("board_tag_size_m").value))
        self.board_tag_center_edge_offset_m = max(0.0, float(self.get_parameter("board_tag_center_edge_offset_m").value))

        self.board_center_c = _parse_vec(
            "board_center_in_camera_m",
            self.get_parameter("board_center_in_camera_m").value,
            3,
        )
        board_rpy_deg = _parse_vec("board_rpy_deg", self.get_parameter("board_rpy_deg").value, 3)
        board_rpy_rad = np.deg2rad(board_rpy_deg)
        self.rot_c_b = _rotation_matrix_from_rpy(
            float(board_rpy_rad[0]),
            float(board_rpy_rad[1]),
            float(board_rpy_rad[2]),
        )

        self.cube_tag_size_m = max(0.005, float(self.get_parameter("cube_tag_size_m").value))
        self.cube_face_offset_m = max(0.001, float(self.get_parameter("cube_face_offset_m").value))

        self.cube_groups: List[List[int]] = [
            [int(v) for v in self.get_parameter("april_cube_1_tag_ids").value],
            [int(v) for v in self.get_parameter("april_cube_2_tag_ids").value],
            [int(v) for v in self.get_parameter("april_cube_3_tag_ids").value],
            [int(v) for v in self.get_parameter("april_cube_4_tag_ids").value],
        ]
        self.cube_centers_b = _parse_cube_centers(self.get_parameter("cube_centers_board_xyz").value)
        if len(self.cube_centers_b) != len(self.cube_groups):
            raise ValueError(
                "cube_centers_board_xyz group count must match cube_groups length "
                f"({len(self.cube_centers_b)} != {len(self.cube_groups)})"
            )

        self.image_width = max(1, int(self.get_parameter("image_width").value))
        self.image_height = max(1, int(self.get_parameter("image_height").value))
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.corner_scale_px_m = max(1.0, float(self.get_parameter("corner_scale_px_m").value))
        self.corner_min_px = max(1.0, float(self.get_parameter("corner_min_px").value))

        self._detections_pub = self.create_publisher(AprilTagDetectionArray, self.detections_topic, 10)
        self._detections_fallback_pub = self.create_publisher(AprilTagDetectionArray, self.detections_fallback_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._tag_poses = self._build_tag_poses()
        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            "sim apriltag publisher started: frame=%s tags=%d detections_topic=%s"
            % (self.camera_frame, len(self._tag_poses), self.detections_topic)
        )

    def _build_tag_poses(self) -> List[TagPose]:
        poses: List[TagPose] = []

        edge = self.board_tag_center_edge_offset_m
        width = self.board_width_m
        depth = self.board_depth_m
        board_tag_layout = {
            self.board_tag_ids[0]: np.array([edge, edge, 0.0], dtype=np.float64),
            self.board_tag_ids[1]: np.array([width - edge, edge, 0.0], dtype=np.float64),
            self.board_tag_ids[2]: np.array([edge, depth - edge, 0.0], dtype=np.float64),
            self.board_tag_ids[3]: np.array([width - edge, depth - edge, 0.0], dtype=np.float64),
        }

        for tag_id, point_b in board_tag_layout.items():
            center_c = _transform_point(self.rot_c_b, self.board_center_c, point_b)
            poses.append(
                TagPose(
                    tag_id=int(tag_id),
                    center_c=center_c,
                    rot_c_t=self.rot_c_b.copy(),
                )
            )

        face_axes = [
            np.array([1.0, 0.0, 0.0], dtype=np.float64),
            np.array([-1.0, 0.0, 0.0], dtype=np.float64),
            np.array([0.0, 1.0, 0.0], dtype=np.float64),
            np.array([0.0, -1.0, 0.0], dtype=np.float64),
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
            np.array([0.0, 0.0, -1.0], dtype=np.float64),
        ]

        for group_idx, tag_ids in enumerate(self.cube_groups):
            if len(tag_ids) != 6:
                self.get_logger().warn(
                    "cube_groups[%d] has %d IDs; expected 6. Skipping this group."
                    % (group_idx, len(tag_ids))
                )
                continue

            cube_center_b = self.cube_centers_b[group_idx]
            cube_rot_c = self.rot_c_b.copy()
            for face_idx, tag_id in enumerate(tag_ids):
                axis_b = face_axes[face_idx]
                tag_center_b = cube_center_b + axis_b * self.cube_face_offset_m
                tag_center_c = _transform_point(self.rot_c_b, self.board_center_c, tag_center_b)
                poses.append(TagPose(tag_id=int(tag_id), center_c=tag_center_c, rot_c_t=cube_rot_c))

        return poses

    def _project_point(self, point_c: np.ndarray) -> Tuple[float, float] | None:
        z = float(point_c[2])
        if z <= 1e-6:
            return None
        u = self.fx * float(point_c[0]) / z + self.cx
        v = self.fy * float(point_c[1]) / z + self.cy
        return u, v

    def _build_detection(self, pose: TagPose) -> AprilTagDetection | None:
        center_uv = self._project_point(pose.center_c)
        if center_uv is None:
            return None

        detection = AprilTagDetection()
        detection.family = self.tag_family
        detection.id = int(pose.tag_id)
        detection.hamming = 0
        detection.goodness = 1.0
        detection.decision_margin = 80.0

        detection.centre.x = float(center_uv[0])
        detection.centre.y = float(center_uv[1])

        half = max(self.corner_min_px, self.corner_scale_px_m * self.board_tag_size_m / max(0.1, pose.center_c[2])) * 0.5
        corners = [
            (center_uv[0] - half, center_uv[1] - half),
            (center_uv[0] + half, center_uv[1] - half),
            (center_uv[0] + half, center_uv[1] + half),
            (center_uv[0] - half, center_uv[1] + half),
        ]
        for i, (u, v) in enumerate(corners):
            detection.corners[i].x = float(min(max(0.0, u), float(self.image_width - 1)))
            detection.corners[i].y = float(min(max(0.0, v), float(self.image_height - 1)))

        detection.homography[:] = [1.0, 0.0, detection.centre.x, 0.0, 1.0, detection.centre.y, 0.0, 0.0, 1.0]
        return detection

    def _broadcast_tf(self, stamp_msg, pose: TagPose) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp_msg
        tf_msg.header.frame_id = self.camera_frame
        tf_msg.child_frame_id = default_tag_frame_name(self.tag_family, pose.tag_id)
        tf_msg.transform.translation.x = float(pose.center_c[0])
        tf_msg.transform.translation.y = float(pose.center_c[1])
        tf_msg.transform.translation.z = float(pose.center_c[2])
        qx, qy, qz, qw = quaternion_from_rotation_matrix(pose.rot_c_t)
        tf_msg.transform.rotation.x = float(qx)
        tf_msg.transform.rotation.y = float(qy)
        tf_msg.transform.rotation.z = float(qz)
        tf_msg.transform.rotation.w = float(qw)
        self._tf_broadcaster.sendTransform(tf_msg)

    def _on_timer(self) -> None:
        stamp_msg = self.get_clock().now().to_msg()

        out = AprilTagDetectionArray()
        out.header.stamp = stamp_msg
        out.header.frame_id = self.camera_frame

        for tag_pose in self._tag_poses:
            detection = self._build_detection(tag_pose)
            if detection is None:
                continue
            out.detections.append(detection)
            self._broadcast_tf(stamp_msg, tag_pose)

        self._detections_pub.publish(out)
        if self.detections_fallback_topic and self.detections_fallback_topic != self.detections_topic:
            self._detections_fallback_pub.publish(out)


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimApriltagPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
