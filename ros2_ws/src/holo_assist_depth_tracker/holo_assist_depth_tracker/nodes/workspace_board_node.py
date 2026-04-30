#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker

from holo_assist_depth_tracker.utils.apriltag import default_tag_frame_name
from holo_assist_depth_tracker.utils.math3d import (
    quaternion_from_rotation_matrix,
    rotation_matrix_from_quaternion,
)


def solve_workspace_pose(
    observed_points: Dict[int, np.ndarray],
    model_points: Dict[int, np.ndarray],
    expected_dists: Dict[Tuple[int, int], float],
    rms_tolerance_m: float,
    dimension_tolerance_m: float,
) -> Optional[Tuple[np.ndarray, np.ndarray, float, float]]:
    ordered_ids = sorted(model_points.keys())
    if any(tag_id not in observed_points for tag_id in ordered_ids):
        return None

    src = np.stack([model_points[tag_id] for tag_id in ordered_ids], axis=0)
    dst = np.stack([observed_points[tag_id] for tag_id in ordered_ids], axis=0)

    src_center = np.mean(src, axis=0)
    dst_center = np.mean(dst, axis=0)
    src_zero = src - src_center
    dst_zero = dst - dst_center

    h = src_zero.T @ dst_zero
    u, _, vt = np.linalg.svd(h)
    rot_c_w = vt.T @ u.T
    if float(np.linalg.det(rot_c_w)) < 0.0:
        vt[-1, :] *= -1.0
        rot_c_w = vt.T @ u.T

    origin_c = dst_center - rot_c_w @ src_center
    predicted = (rot_c_w @ src.T).T + origin_c
    residuals = np.linalg.norm(predicted - dst, axis=1)
    rms_residual_m = float(np.sqrt(np.mean(np.square(residuals))))

    max_dimension_error = 0.0
    for (a, b), expected_dist in expected_dists.items():
        dist = float(np.linalg.norm(observed_points[a] - observed_points[b]))
        max_dimension_error = max(max_dimension_error, abs(dist - expected_dist))

    if rms_residual_m > rms_tolerance_m:
        return None
    if max_dimension_error > dimension_tolerance_m:
        return None

    return origin_c, rot_c_w, rms_residual_m, max_dimension_error


@dataclass
class WorkspaceSolution:
    parent_frame: str
    origin_c: np.ndarray
    rotation_c_w: np.ndarray
    rms_residual_m: float
    max_dimension_error_m: float


class WorkspaceBoardNode(Node):
    """Derive and lock workspace frame from a deterministic 4-tag board."""

    TAG_LAYOUT_W = {
        0: np.array([0.075, 0.075, 0.0], dtype=np.float64),
        1: np.array([0.625, 0.075, 0.0], dtype=np.float64),
        2: np.array([0.075, 0.425, 0.0], dtype=np.float64),
        3: np.array([0.625, 0.425, 0.0], dtype=np.float64),
    }

    EXPECTED_CENTER_DISTS = {
        (0, 1): 0.55,
        (0, 2): 0.35,
        (1, 3): 0.35,
        (2, 3): 0.55,
        (0, 3): math.sqrt(0.55 * 0.55 + 0.35 * 0.35),
        (1, 2): math.sqrt(0.55 * 0.55 + 0.35 * 0.35),
    }

    def __init__(self) -> None:
        super().__init__("holoassist_workspace_board")

        self.declare_parameter("detections_topic", "/detections_board")
        self.declare_parameter("workspace_frame", "workspace_frame")
        self.declare_parameter("tag_family", "36h11")
        self.declare_parameter("board_tag_ids", [0, 1, 2, 3])
        self.declare_parameter("tag_lookup_timeout_s", 0.05)
        self.declare_parameter("detections_timeout_s", 1.0)
        self.declare_parameter("timer_hz", 20.0)
        self.declare_parameter("geometry_rms_tolerance_m", 0.02)
        self.declare_parameter("dimension_tolerance_m", 0.04)
        self.declare_parameter("workspace_realign_service_name", "/holoassist/perception/realign_workspace")

        self.declare_parameter("robot_tf_child_frame", "ur3e_base_link0")
        self.declare_parameter("publish_robot_tf", True)
        self.declare_parameter("robot_pose_topic", "/holoassist/perception/ur3e_base_link0_pose")
        self.declare_parameter("robot_marker_topic", "/holoassist/perception/ur3e_base_link0_marker")
        self.declare_parameter("robot_x_m", 0.450)
        self.declare_parameter("robot_y_m", 0.564)
        self.declare_parameter("robot_z_m", 0.0)
        self.declare_parameter("robot_base_diameter_m", 0.128)
        self.declare_parameter("robot_marker_height_m", 0.01)

        self.declare_parameter("workspace_mode_topic", "/holoassist/perception/workspace_mode")
        self.declare_parameter("workspace_diag_topic", "/holoassist/perception/workspace_diagnostics")
        self.declare_parameter("plane_coeff_topic", "/holoassist/perception/bench_plane_coefficients")

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.workspace_frame = str(self.get_parameter("workspace_frame").value)
        self.tag_family = str(self.get_parameter("tag_family").value)
        self.board_tag_ids = [int(v) for v in self.get_parameter("board_tag_ids").value]
        self.tag_lookup_timeout_s = max(0.0, float(self.get_parameter("tag_lookup_timeout_s").value))
        self.detections_timeout_s = max(0.05, float(self.get_parameter("detections_timeout_s").value))
        self.timer_hz = max(1.0, float(self.get_parameter("timer_hz").value))
        self.geometry_rms_tolerance_m = max(0.001, float(self.get_parameter("geometry_rms_tolerance_m").value))
        self.dimension_tolerance_m = max(0.001, float(self.get_parameter("dimension_tolerance_m").value))
        self.workspace_realign_service_name = str(self.get_parameter("workspace_realign_service_name").value)

        self.robot_tf_child_frame = str(self.get_parameter("robot_tf_child_frame").value)
        self.publish_robot_tf = bool(self.get_parameter("publish_robot_tf").value)
        self.robot_pose_topic = str(self.get_parameter("robot_pose_topic").value)
        self.robot_marker_topic = str(self.get_parameter("robot_marker_topic").value)
        self.robot_x_m = float(self.get_parameter("robot_x_m").value)
        self.robot_y_m = float(self.get_parameter("robot_y_m").value)
        self.robot_z_m = float(self.get_parameter("robot_z_m").value)
        self.robot_base_diameter_m = max(0.01, float(self.get_parameter("robot_base_diameter_m").value))
        self.robot_marker_height_m = max(0.001, float(self.get_parameter("robot_marker_height_m").value))

        self.workspace_mode_topic = str(self.get_parameter("workspace_mode_topic").value)
        self.workspace_diag_topic = str(self.get_parameter("workspace_diag_topic").value)
        self.plane_coeff_topic = str(self.get_parameter("plane_coeff_topic").value)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._mode_pub = self.create_publisher(String, self.workspace_mode_topic, reliable_qos)
        self._diag_pub = self.create_publisher(DiagnosticArray, self.workspace_diag_topic, reliable_qos)
        self._plane_pub = self.create_publisher(Float32MultiArray, self.plane_coeff_topic, reliable_qos)
        self._robot_pose_pub = self.create_publisher(PoseStamped, self.robot_pose_topic, reliable_qos)
        self._robot_marker_pub = self.create_publisher(Marker, self.robot_marker_topic, reliable_qos)

        self._det_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._on_detections,
            reliable_qos,
        )
        self._realign_srv = self.create_service(
            Trigger,
            self.workspace_realign_service_name,
            self._on_realign,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._latest_detections_msg: Optional[AprilTagDetectionArray] = None
        self._latest_detection_stamp: Optional[rclpy.time.Time] = None

        self._locked = False
        self._locked_solution: Optional[WorkspaceSolution] = None

        self._timer = self.create_timer(1.0 / self.timer_hz, self._on_timer)

        self.get_logger().info(
            "workspace board node started detections=%s workspace_frame=%s tags=%s realign_service=%s"
            % (
                self.detections_topic,
                self.workspace_frame,
                self.board_tag_ids,
                self.workspace_realign_service_name,
            )
        )

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self._latest_detections_msg = msg
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        self._latest_detection_stamp = stamp

    def _on_realign(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._locked = False
        self._locked_solution = None
        response.success = True
        response.message = "workspace lock cleared; waiting for first valid full board detection"
        self.get_logger().info("workspace realign requested: lock cleared")
        return response

    def _on_timer(self) -> None:
        visible_ids, frame_id, fresh = self._latest_detection_view()
        have_all = all(tag_id in visible_ids for tag_id in self.board_tag_ids)

        if not fresh:
            self._publish_state(
                mode="INVALID",
                level=DiagnosticStatus.ERROR,
                message="stale or missing board detections",
                extra={"visible_ids": sorted(visible_ids), "locked": self._locked},
            )
            if self._locked and self._locked_solution is not None:
                self._publish_locked_solution(self._locked_solution)
            return

        if not have_all:
            self._publish_state(
                mode="INVALID",
                level=DiagnosticStatus.ERROR,
                message="insufficient board tags",
                extra={"visible_ids": sorted(visible_ids), "required_ids": self.board_tag_ids, "locked": self._locked},
            )
            if self._locked and self._locked_solution is not None:
                self._publish_locked_solution(self._locked_solution)
            return

        if self._locked and self._locked_solution is not None:
            self._publish_locked_solution(self._locked_solution)
            self._publish_state(
                mode="LOCKED",
                level=DiagnosticStatus.OK,
                message="workspace locked; drift updates disabled",
                extra={"visible_ids": sorted(visible_ids), "locked": True},
            )
            return

        solution = self._estimate_workspace(frame_id=frame_id)
        if solution is None:
            self._publish_state(
                mode="INVALID",
                level=DiagnosticStatus.ERROR,
                message="board geometry invalid",
                extra={"visible_ids": sorted(visible_ids), "locked": False},
            )
            return

        self._locked = True
        self._locked_solution = solution
        self.get_logger().info("workspace locked from first valid full 4-tag board detection")
        self._publish_locked_solution(solution)
        self._publish_state(
            mode="LOCKED",
            level=DiagnosticStatus.OK,
            message="workspace locked",
            extra={
                "visible_ids": sorted(visible_ids),
                "rms_residual_m": round(solution.rms_residual_m, 4),
                "max_dimension_error_m": round(solution.max_dimension_error_m, 4),
                "locked": True,
            },
        )

    def _latest_detection_view(self) -> Tuple[set[int], str, bool]:
        if self._latest_detections_msg is None or self._latest_detection_stamp is None:
            return set(), "", False

        now = self.get_clock().now()
        age_s = (now - self._latest_detection_stamp).nanoseconds / 1e9
        fresh = age_s <= self.detections_timeout_s
        visible_ids = {int(det.id) for det in self._latest_detections_msg.detections}
        frame_id = str(self._latest_detections_msg.header.frame_id)
        return visible_ids, frame_id, fresh

    def _estimate_workspace(self, frame_id: str) -> Optional[WorkspaceSolution]:
        if not frame_id:
            return None

        observed_points: Dict[int, np.ndarray] = {}
        rotations: Dict[int, np.ndarray] = {}

        for tag_id in self.board_tag_ids:
            tag_frame = default_tag_frame_name(self.tag_family, tag_id)
            try:
                tf_msg = self._tf_buffer.lookup_transform(
                    frame_id,
                    tag_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.tag_lookup_timeout_s),
                )
            except TransformException:
                return None

            t = tf_msg.transform.translation
            r = tf_msg.transform.rotation
            observed_points[int(tag_id)] = np.array([float(t.x), float(t.y), float(t.z)], dtype=np.float64)
            rotations[int(tag_id)] = rotation_matrix_from_quaternion(float(r.x), float(r.y), float(r.z), float(r.w))

        solved = solve_workspace_pose(
            observed_points=observed_points,
            model_points={k: self.TAG_LAYOUT_W[k] for k in self.board_tag_ids},
            expected_dists=self.EXPECTED_CENTER_DISTS,
            rms_tolerance_m=self.geometry_rms_tolerance_m,
            dimension_tolerance_m=self.dimension_tolerance_m,
        )
        if solved is None:
            return None

        origin_c, rot_c_w, rms_residual_m, max_dimension_error = solved

        normals = [rot[:, 2] for rot in rotations.values()]
        if normals:
            avg_normal = np.mean(np.stack(normals, axis=0), axis=0)
            if float(np.dot(rot_c_w[:, 2], avg_normal)) < 0.0:
                rot_c_w[:, 2] *= -1.0
                rot_c_w[:, 1] *= -1.0

        return WorkspaceSolution(
            parent_frame=frame_id,
            origin_c=origin_c,
            rotation_c_w=rot_c_w,
            rms_residual_m=rms_residual_m,
            max_dimension_error_m=max_dimension_error,
        )

    def _publish_locked_solution(self, solution: WorkspaceSolution) -> None:
        self._publish_workspace_tf(solution)
        self._publish_plane(solution)
        self._publish_robot_pose_and_tf()

    def _publish_workspace_tf(self, solution: WorkspaceSolution) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = solution.parent_frame
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.child_frame_id = self.workspace_frame
        tf_msg.transform.translation.x = float(solution.origin_c[0])
        tf_msg.transform.translation.y = float(solution.origin_c[1])
        tf_msg.transform.translation.z = float(solution.origin_c[2])
        qx, qy, qz, qw = quaternion_from_rotation_matrix(solution.rotation_c_w)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf_msg)

    def _publish_plane(self, solution: WorkspaceSolution) -> None:
        normal = solution.rotation_c_w[:, 2]
        d = -float(np.dot(normal, solution.origin_c))
        msg = Float32MultiArray()
        msg.data = [float(normal[0]), float(normal[1]), float(normal[2]), float(d)]
        self._plane_pub.publish(msg)

    def _publish_robot_pose_and_tf(self) -> None:
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.frame_id = self.workspace_frame
        pose.header.stamp = stamp
        pose.pose.position.x = float(self.robot_x_m)
        pose.pose.position.y = float(self.robot_y_m)
        pose.pose.position.z = float(self.robot_z_m)
        pose.pose.orientation.w = 1.0
        self._robot_pose_pub.publish(pose)

        if self.publish_robot_tf:
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = self.workspace_frame
            tf_msg.header.stamp = stamp
            tf_msg.child_frame_id = self.robot_tf_child_frame
            tf_msg.transform.translation.x = float(self.robot_x_m)
            tf_msg.transform.translation.y = float(self.robot_y_m)
            tf_msg.transform.translation.z = float(self.robot_z_m)
            tf_msg.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(tf_msg)

        marker = Marker()
        marker.header.frame_id = self.workspace_frame
        marker.header.stamp = stamp
        marker.ns = "holoassist_robot_pose"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.robot_x_m)
        marker.pose.position.y = float(self.robot_y_m)
        marker.pose.position.z = float(self.robot_z_m + 0.5 * self.robot_marker_height_m)
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(self.robot_base_diameter_m)
        marker.scale.y = float(self.robot_base_diameter_m)
        marker.scale.z = float(self.robot_marker_height_m)
        marker.color.r = 0.95
        marker.color.g = 0.65
        marker.color.b = 0.15
        marker.color.a = 0.85
        self._robot_marker_pub.publish(marker)

    def _publish_state(self, mode: str, level: int, message: str, extra: Optional[Dict[str, object]] = None) -> None:
        mode_msg = String()
        mode_msg.data = mode
        self._mode_pub.publish(mode_msg)

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "holoassist/workspace_frame"
        status.level = int(level)
        status.hardware_id = "holoassist_workspace_board"
        status.message = message

        values = [
            KeyValue(key="mode", value=mode),
            KeyValue(key="workspace_frame", value=self.workspace_frame),
            KeyValue(key="board_tag_ids", value=",".join(str(v) for v in self.board_tag_ids)),
            KeyValue(key="locked", value=str(bool(self._locked))),
        ]
        if extra:
            for key, value in extra.items():
                values.append(KeyValue(key=str(key), value=str(value)))

        status.values = values
        diag.status = [status]
        self._diag_pub.publish(diag)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorkspaceBoardNode()
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
