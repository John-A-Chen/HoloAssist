#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class SelectedCubeToMoveItTargetNode(Node):
    """Publish selected-cube hover point and pre-grasp pose targets for MoveIt."""

    def __init__(self) -> None:
        super().__init__("holoassist_selected_cube_to_moveit_target")

        self.declare_parameter("workspace_frame", "workspace_frame")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("selected_cube_topic", "/holoassist/teleop/selected_cube")
        self.declare_parameter("input_pose_topic", "/holoassist/teleop/selected_cube_pose")
        self.declare_parameter("output_point_topic", "/moveit_robot_control/target_point")
        self.declare_parameter("output_pose_topic", "/moveit_robot_control/target_pose")
        self.declare_parameter("output_point_topic_legacy", "")
        self.declare_parameter("target_x_offset_m", 0.0)
        self.declare_parameter("target_y_offset_m", 0.0)
        self.declare_parameter("target_z_offset_m", 0.10)
        self.declare_parameter("target_roll_rad", math.pi)
        self.declare_parameter("target_pitch_rad", 0.0)
        self.declare_parameter("target_yaw_rad", 0.0)
        self.declare_parameter("min_position_delta_m", 0.02)
        self.declare_parameter("min_republish_period_sec", 1.0)

        self.workspace_frame = str(self.get_parameter("workspace_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.selected_cube_topic = str(self.get_parameter("selected_cube_topic").value)
        self.input_pose_topic = str(self.get_parameter("input_pose_topic").value)
        self.output_point_topic = str(self.get_parameter("output_point_topic").value)
        self.output_pose_topic = str(self.get_parameter("output_pose_topic").value)
        self.output_point_topic_legacy = str(
            self.get_parameter("output_point_topic_legacy").value
        ).strip()
        self.target_x_offset_m = float(self.get_parameter("target_x_offset_m").value)
        self.target_y_offset_m = float(self.get_parameter("target_y_offset_m").value)
        self.target_z_offset_m = float(self.get_parameter("target_z_offset_m").value)
        self.target_roll_rad = float(self.get_parameter("target_roll_rad").value)
        self.target_pitch_rad = float(self.get_parameter("target_pitch_rad").value)
        self.target_yaw_rad = float(self.get_parameter("target_yaw_rad").value)
        self.min_position_delta_m = max(0.0, float(self.get_parameter("min_position_delta_m").value))
        self.min_republish_period_sec = max(
            0.0, float(self.get_parameter("min_republish_period_sec").value)
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._selected_cube_name: str = ""
        self._force_publish_next = True
        self._last_published_point: Optional[Tuple[float, float, float]] = None
        self._last_publish_time = self.get_clock().now() - Duration(seconds=3600.0)

        self._point_pub = self.create_publisher(Point, self.output_point_topic, 10)
        self._pose_pub = self.create_publisher(Pose, self.output_pose_topic, 10)
        self._legacy_point_pub = None
        if (
            self.output_point_topic_legacy
            and self.output_point_topic_legacy != self.output_point_topic
        ):
            self._legacy_point_pub = self.create_publisher(
                Point, self.output_point_topic_legacy, 10
            )
        self.create_subscription(String, self.selected_cube_topic, self._on_selected_cube, 10)
        self.create_subscription(PoseStamped, self.input_pose_topic, self._on_selected_cube_pose, 10)

        self._fixed_orientation = self._quaternion_from_rpy(
            self.target_roll_rad,
            self.target_pitch_rad,
            self.target_yaw_rad,
        )

        output_topics = [self.output_point_topic, self.output_pose_topic]
        if self._legacy_point_pub is not None:
            output_topics.append(self.output_point_topic_legacy)

        self.get_logger().info(
            "selected cube adapter started input=%s outputs=%s target_frame=%s offsets=(%.3f,%.3f,%.3f) rpy_rad=(%.3f,%.3f,%.3f)"
            % (
                self.input_pose_topic,
                ",".join(output_topics),
                self.target_frame,
                self.target_x_offset_m,
                self.target_y_offset_m,
                self.target_z_offset_m,
                self.target_roll_rad,
                self.target_pitch_rad,
                self.target_yaw_rad,
            )
        )

    def _on_selected_cube(self, msg: String) -> None:
        name = msg.data.strip().lower()
        if name != self._selected_cube_name:
            self.get_logger().info(
                "selected cube changed from %s to %s"
                % (self._selected_cube_name or "<none>", name or "<none>")
            )
            self._selected_cube_name = name
            self._force_publish_next = True

    def _on_selected_cube_pose(self, msg: PoseStamped) -> None:
        frame_id = msg.header.frame_id or self.workspace_frame
        transformed_xyz = self._transform_point_to_target_frame(
            frame_id,
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.header.stamp,
        )
        if transformed_xyz is None:
            return

        target_xyz = self._apply_position_offsets(transformed_xyz)

        if not self._should_publish(target_xyz):
            return

        point_msg = Point()
        point_msg.x = float(target_xyz[0])
        point_msg.y = float(target_xyz[1])
        point_msg.z = float(target_xyz[2])

        pose_msg = Pose()
        pose_msg.position.x = point_msg.x
        pose_msg.position.y = point_msg.y
        pose_msg.position.z = point_msg.z
        pose_msg.orientation = self._fixed_orientation

        self._point_pub.publish(point_msg)
        self._pose_pub.publish(pose_msg)
        if self._legacy_point_pub is not None:
            self._legacy_point_pub.publish(point_msg)

        self._last_published_point = target_xyz
        self._last_publish_time = self.get_clock().now()
        self._force_publish_next = False

        self.get_logger().info(
            "published pre-grasp hover target cube=%s input_frame=%s target_frame=%s transformed_xyz=(%.3f,%.3f,%.3f) point=(%.3f,%.3f,%.3f) pose_rpy_rad=(%.3f,%.3f,%.3f)"
            % (
                self._selected_cube_name or "<unknown>",
                frame_id,
                self.target_frame,
                transformed_xyz[0],
                transformed_xyz[1],
                transformed_xyz[2],
                point_msg.x,
                point_msg.y,
                point_msg.z,
                self.target_roll_rad,
                self.target_pitch_rad,
                self.target_yaw_rad,
            ),
            throttle_duration_sec=1.0,
        )

    def _apply_position_offsets(
        self, transformed_xyz: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        return (
            transformed_xyz[0] + self.target_x_offset_m,
            transformed_xyz[1] + self.target_y_offset_m,
            transformed_xyz[2] + self.target_z_offset_m,
        )

    def _should_publish(self, xyz: Tuple[float, float, float]) -> bool:
        now = self.get_clock().now()

        if self._force_publish_next or self._last_published_point is None:
            return True

        elapsed = (now - self._last_publish_time).nanoseconds / 1e9
        if elapsed < self.min_republish_period_sec:
            return False

        dx = xyz[0] - self._last_published_point[0]
        dy = xyz[1] - self._last_published_point[1]
        dz = xyz[2] - self._last_published_point[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz) >= self.min_position_delta_m

    def _transform_point_to_target_frame(
        self, source_frame: str, x: float, y: float, z: float, source_stamp
    ) -> Optional[Tuple[float, float, float]]:
        if source_frame == self.target_frame:
            return (x, y, z)

        lookup_time = Time()
        if source_stamp.sec != 0 or source_stamp.nanosec != 0:
            lookup_time = Time.from_msg(source_stamp)

        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                lookup_time,
            )
        except TransformException as exc:
            self.get_logger().warn(
                "waiting for TF %s->%s: %s" % (source_frame, self.target_frame, exc),
                throttle_duration_sec=2.0,
            )
            return None

        qx = tf_msg.transform.rotation.x
        qy = tf_msg.transform.rotation.y
        qz = tf_msg.transform.rotation.z
        qw = tf_msg.transform.rotation.w

        rx, ry, rz = self._rotate_vector_by_quaternion((x, y, z), (qx, qy, qz, qw))

        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        tz = tf_msg.transform.translation.z
        return (rx + tx, ry + ty, rz + tz)

    @staticmethod
    def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        out = Quaternion()
        out.w = cr * cp * cy + sr * sp * sy
        out.x = sr * cp * cy - cr * sp * sy
        out.y = cr * sp * cy + sr * cp * sy
        out.z = cr * cp * sy - sr * sp * cy
        return out

    @staticmethod
    def _rotate_vector_by_quaternion(
        vec: Tuple[float, float, float], quat: Tuple[float, float, float, float]
    ) -> Tuple[float, float, float]:
        vx, vy, vz = vec
        qx, qy, qz, qw = quat

        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-12:
            return vec

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)

        rx = vx + qw * tx + (qy * tz - qz * ty)
        ry = vy + qw * ty + (qz * tx - qx * tz)
        rz = vz + qw * tz + (qx * ty - qy * tx)
        return (rx, ry, rz)


def main() -> None:
    rclpy.init()
    node = SelectedCubeToMoveItTargetNode()
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
