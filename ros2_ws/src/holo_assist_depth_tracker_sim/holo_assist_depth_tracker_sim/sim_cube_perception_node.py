#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf_transformations import quaternion_matrix
from visualization_msgs.msg import Marker

from holo_assist_depth_tracker_sim.common import CUBE_COLORS, CUBE_NAMES
from holo_assist_depth_tracker_sim_interfaces.msg import CubePerceptionStatus


@dataclass
class CubeMemory:
    truth_pose: Optional[PoseStamped] = None
    perceived_pose: Optional[PoseStamped] = None
    last_seen_pose: Optional[PoseStamped] = None
    last_seen_time_ns: Optional[int] = None


class SimCubePerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("holoassist_sim_cube_perception")

        self.declare_parameter("workspace_frame", "workspace_frame")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("cube_size_m", 0.040)
        self.declare_parameter("horizontal_fov_deg", 69.0)
        self.declare_parameter("vertical_fov_deg", 42.0)
        self.declare_parameter("max_range_m", 2.0)
        self.declare_parameter("publish_last_seen_when_hidden", True)
        self.declare_parameter("near_clip_m", 0.05)
        self.declare_parameter("far_clip_m", 1.2)
        self.declare_parameter("frustum_color_rgba", [0.0, 1.0, 1.0, 0.95])
        self.declare_parameter("frustum_line_width", 0.006)

        self.workspace_frame = str(self.get_parameter("workspace_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.cube_size_m = float(self.get_parameter("cube_size_m").value)
        self.horizontal_fov_rad = math.radians(float(self.get_parameter("horizontal_fov_deg").value))
        self.vertical_fov_rad = math.radians(float(self.get_parameter("vertical_fov_deg").value))
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.publish_last_seen = bool(self.get_parameter("publish_last_seen_when_hidden").value)
        self.near_clip_m = max(0.001, float(self.get_parameter("near_clip_m").value))
        self.far_clip_m = max(self.near_clip_m + 0.001, float(self.get_parameter("far_clip_m").value))

        color = [float(v) for v in self.get_parameter("frustum_color_rgba").value]
        if len(color) != 4:
            raise ValueError("frustum_color_rgba must have 4 values")
        self.frustum_color_rgba = color
        self.frustum_line_width = max(0.0005, float(self.get_parameter("frustum_line_width").value))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._state: Dict[str, CubeMemory] = {name: CubeMemory() for name in CUBE_NAMES}

        self._perceived_pose_pubs = {
            name: self.create_publisher(PoseStamped, f"/holoassist/sim/perception/{name}_pose", 10)
            for name in CUBE_NAMES
        }
        self._status_pubs = {
            name: self.create_publisher(CubePerceptionStatus, f"/holoassist/sim/perception/{name}_status", 10)
            for name in CUBE_NAMES
        }
        self._marker_pubs = {
            name: self.create_publisher(Marker, f"/holoassist/sim/perception/{name}_marker", 10)
            for name in CUBE_NAMES
        }
        self._frustum_marker_pub = self.create_publisher(Marker, "/holoassist/sim/camera/frustum_marker", 10)
        self._camera_debug_pub = self.create_publisher(String, "/holoassist/sim/camera/debug_status", 10)

        for name in CUBE_NAMES:
            self.create_subscription(
                PoseStamped,
                f"/holoassist/sim/truth/{name}_pose",
                self._make_truth_cb(name),
                10,
            )

        self._reset_memory_srv = self.create_service(
            Trigger,
            "/holoassist/sim/reset_perception_memory",
            self._on_reset_memory,
        )

        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            "sim perception node started workspace=%s camera=%s fov=(%.1f, %.1f) near=%.2f far=%.2f max_range=%.2f"
            % (
                self.workspace_frame,
                self.camera_frame,
                math.degrees(self.horizontal_fov_rad),
                math.degrees(self.vertical_fov_rad),
                self.near_clip_m,
                self.far_clip_m,
                self.max_range_m,
            )
        )

    def _make_truth_cb(self, cube_name: str):
        def _cb(msg: PoseStamped) -> None:
            self._state[cube_name].truth_pose = msg

        return _cb

    def _on_reset_memory(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        for cube_name in CUBE_NAMES:
            state = self._state[cube_name]
            state.perceived_pose = None
            state.last_seen_pose = None
            state.last_seen_time_ns = None
        response.success = True
        response.message = "cleared last-seen memory"
        return response

    def _on_timer(self) -> None:
        tf_ws_to_cam = self._lookup_workspace_to_camera_transform()
        now = self.get_clock().now()
        stamp = now.to_msg()

        visible_count = 0
        seen_count = 0

        for idx, name in enumerate(CUBE_NAMES, start=1):
            state = self._state[name]
            truth_pose = state.truth_pose
            if truth_pose is None:
                continue

            pos_in_cam = self._point_in_camera(tf_ws_to_cam, truth_pose)
            visible, in_front, within_range, within_fov = self._is_visible(pos_in_cam)

            if visible:
                visible_count += 1
                state.perceived_pose = self._copy_pose(truth_pose, stamp)
                state.last_seen_pose = self._copy_pose(truth_pose, stamp)
                state.last_seen_time_ns = now.nanoseconds
            elif self.publish_last_seen and state.last_seen_pose is not None:
                state.perceived_pose = self._copy_pose(state.last_seen_pose, stamp)

            if state.last_seen_pose is not None:
                seen_count += 1

            perceived_pose = state.perceived_pose
            if perceived_pose is not None:
                self._perceived_pose_pubs[name].publish(perceived_pose)
                self._publish_cube_marker(name, idx, perceived_pose, stamp, visible)
                self._publish_cube_tf(idx, perceived_pose, stamp)

            self._publish_status(
                cube_name=name,
                truth_pose=truth_pose,
                perceived_pose=perceived_pose,
                pos_in_cam=pos_in_cam,
                visible=visible,
                in_front=in_front,
                within_range=within_range,
                within_fov=within_fov,
                now_ns=now.nanoseconds,
                stamp=stamp,
            )

        self._publish_frustum_marker(stamp)
        self._publish_camera_debug(stamp, visible_count, seen_count)

    def _lookup_workspace_to_camera_transform(self) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                self.camera_frame,
                self.workspace_frame,
                rclpy.time.Time(),
                timeout=RclpyDuration(seconds=0.05),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"waiting for TF {self.workspace_frame}->{self.camera_frame}: {exc}")
            return None

    def _point_in_camera(self, tf_ws_to_cam: Optional[TransformStamped], truth_pose: PoseStamped) -> Point:
        p = Point()
        if tf_ws_to_cam is None:
            p.x = float("nan")
            p.y = float("nan")
            p.z = float("nan")
            return p

        q = tf_ws_to_cam.transform.rotation
        t = tf_ws_to_cam.transform.translation
        mat = quaternion_matrix([q.x, q.y, q.z, q.w])

        pw = [
            float(truth_pose.pose.position.x),
            float(truth_pose.pose.position.y),
            float(truth_pose.pose.position.z),
            1.0,
        ]

        pc = [0.0, 0.0, 0.0]
        pc[0] = mat[0][0] * pw[0] + mat[0][1] * pw[1] + mat[0][2] * pw[2] + float(t.x)
        pc[1] = mat[1][0] * pw[0] + mat[1][1] * pw[1] + mat[1][2] * pw[2] + float(t.y)
        pc[2] = mat[2][0] * pw[0] + mat[2][1] * pw[1] + mat[2][2] * pw[2] + float(t.z)

        p.x = pc[0]
        p.y = pc[1]
        p.z = pc[2]
        return p

    def _is_visible(self, p: Point) -> tuple[bool, bool, bool, bool]:
        if math.isnan(p.z):
            return False, False, False, False

        in_front = bool(p.z > 0.0)
        within_range = bool(p.z < self.max_range_m)
        if not in_front:
            return False, in_front, within_range, False

        horizontal_angle = math.atan2(p.x, p.z)
        vertical_angle = math.atan2(p.y, p.z)

        within_fov = bool(
            abs(horizontal_angle) <= (self.horizontal_fov_rad / 2.0)
            and abs(vertical_angle) <= (self.vertical_fov_rad / 2.0)
        )

        visible = bool(in_front and within_range and within_fov)
        return visible, in_front, within_range, within_fov

    def _copy_pose(self, source: PoseStamped, stamp) -> PoseStamped:
        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.workspace_frame
        out.pose = source.pose
        return out

    def _publish_cube_tf(self, idx: int, pose: PoseStamped, stamp) -> None:
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.workspace_frame
        tf_msg.child_frame_id = f"apriltag_cube_{idx}_sim"
        tf_msg.transform.translation.x = pose.pose.position.x
        tf_msg.transform.translation.y = pose.pose.position.y
        tf_msg.transform.translation.z = pose.pose.position.z
        tf_msg.transform.rotation = pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf_msg)

    def _publish_cube_marker(self, cube_name: str, idx: int, pose: PoseStamped, stamp, visible: bool) -> None:
        marker = Marker()
        marker.header.frame_id = self.workspace_frame
        marker.header.stamp = stamp
        marker.ns = "holoassist_sim_perception"
        marker.id = idx
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = self.cube_size_m
        marker.scale.y = self.cube_size_m
        marker.scale.z = self.cube_size_m

        rgba = CUBE_COLORS[cube_name]
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = 0.95 if visible else 0.45

        self._marker_pubs[cube_name].publish(marker)

    def _publish_frustum_marker(self, stamp) -> None:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.camera_frame
        marker.ns = "holoassist_sim_camera"
        marker.id = 200
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = self.frustum_line_width
        marker.color.r = self.frustum_color_rgba[0]
        marker.color.g = self.frustum_color_rgba[1]
        marker.color.b = self.frustum_color_rgba[2]
        marker.color.a = self.frustum_color_rgba[3]

        def corners(d: float) -> list[Point]:
            half_w = math.tan(self.horizontal_fov_rad / 2.0) * d
            half_h = math.tan(self.vertical_fov_rad / 2.0) * d
            # Optical frame convention: +Z forward, +X right, +Y down.
            return [
                Point(x=-half_w, y=-half_h, z=d),
                Point(x=half_w, y=-half_h, z=d),
                Point(x=half_w, y=half_h, z=d),
                Point(x=-half_w, y=half_h, z=d),
            ]

        near = corners(self.near_clip_m)
        far = corners(self.far_clip_m)
        origin = Point(x=0.0, y=0.0, z=0.0)

        lines: list[Point] = []
        for i in range(4):
            j = (i + 1) % 4
            lines.extend([near[i], near[j]])
            lines.extend([far[i], far[j]])
            lines.extend([near[i], far[i]])
            lines.extend([origin, far[i]])

        marker.points = lines
        self._frustum_marker_pub.publish(marker)

    def _publish_camera_debug(self, stamp, visible_count: int, seen_count: int) -> None:
        debug_msg = String()
        debug_msg.data = (
            f"frame={self.camera_frame} visible_now={visible_count}/{len(CUBE_NAMES)} "
            f"last_seen_available={seen_count}/{len(CUBE_NAMES)} "
            f"fov_deg=({math.degrees(self.horizontal_fov_rad):.1f},{math.degrees(self.vertical_fov_rad):.1f}) "
            f"near={self.near_clip_m:.2f} far={self.far_clip_m:.2f} max_range={self.max_range_m:.2f}"
        )
        self._camera_debug_pub.publish(debug_msg)

    def _publish_status(
        self,
        cube_name: str,
        truth_pose: PoseStamped,
        perceived_pose: Optional[PoseStamped],
        pos_in_cam: Point,
        visible: bool,
        in_front: bool,
        within_range: bool,
        within_fov: bool,
        now_ns: int,
        stamp,
    ) -> None:
        state = self._state[cube_name]
        status = CubePerceptionStatus()
        status.header.stamp = stamp
        status.header.frame_id = self.workspace_frame
        status.cube_name = cube_name
        status.visible_now = bool(visible)
        status.within_fov = bool(within_fov)
        status.within_range = bool(within_range)
        status.in_front = bool(in_front)
        status.last_seen_available = bool(state.last_seen_pose is not None)
        status.truth_pose = self._copy_pose(truth_pose, stamp)

        if perceived_pose is None:
            blank = PoseStamped()
            blank.header.stamp = stamp
            blank.header.frame_id = self.workspace_frame
            status.perceived_pose = blank
        else:
            status.perceived_pose = self._copy_pose(perceived_pose, stamp)

        status.position_in_camera = pos_in_cam

        if state.last_seen_time_ns is None:
            status.last_seen_age = Duration(sec=0, nanosec=0)
        else:
            age_ns = max(0, now_ns - state.last_seen_time_ns)
            status.last_seen_age = Duration(
                sec=int(age_ns // 1_000_000_000),
                nanosec=int(age_ns % 1_000_000_000),
            )

        self._status_pubs[cube_name].publish(status)


def main() -> None:
    rclpy.init()
    node = SimCubePerceptionNode()
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
