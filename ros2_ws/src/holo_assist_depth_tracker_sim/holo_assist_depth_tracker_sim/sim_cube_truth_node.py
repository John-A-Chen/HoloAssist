#!/usr/bin/env python3

from __future__ import annotations

import math
import random
from dataclasses import replace
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

from holo_assist_depth_tracker_sim.common import (
    CUBE_COLORS,
    CUBE_NAMES,
    CubeState,
    cube_state_to_pose,
    normalize_angle,
    yaw_from_quaternion,
)
from holo_assist_depth_tracker_sim_interfaces.srv import SetAprilCubePose, SetCameraPose


class SimCubeTruthNode(Node):
    def __init__(self) -> None:
        super().__init__("holoassist_sim_cube_truth")

        self.declare_parameter("workspace_frame", "workspace_frame")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("board_width_m", 0.700)
        self.declare_parameter("board_depth_m", 0.500)
        self.declare_parameter("board_thickness_m", 0.010)
        self.declare_parameter("board_tag_size_m", 0.032)
        self.declare_parameter("board_tag_center_edge_offset_m", 0.016)
        self.declare_parameter("cube_size_m", 0.040)
        self.declare_parameter("min_cube_spacing_m", 0.080)
        self.declare_parameter("randomise_yaw", True)
        self.declare_parameter("random_seed", 7)

        self.declare_parameter(
            "default_cube_centers_xyz_yaw",
            [
                -0.150,
                -0.100,
                0.020,
                0.0,
                0.150,
                -0.100,
                0.020,
                0.0,
                -0.120,
                0.100,
                0.020,
                0.0,
                0.140,
                0.090,
                0.020,
                0.0,
            ],
        )

        self.declare_parameter("camera_link_frame", "camera_link")
        self.declare_parameter("camera_color_frame", "camera_color_frame")
        self.declare_parameter("camera_color_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("camera_depth_frame", "camera_depth_frame")
        self.declare_parameter("camera_depth_optical_frame", "camera_depth_optical_frame")
        self.declare_parameter("camera_imu_frame", "camera_imu_frame")
        self.declare_parameter("camera_accel_frame", "camera_accel_frame")
        self.declare_parameter("camera_gyro_frame", "camera_gyro_frame")
        self.declare_parameter("camera_default_xyzrpy", [0.0, -0.600, 0.700, 0.0, 0.0, 0.0])
        self.declare_parameter("camera_body_size_xyz", [0.090, 0.025, 0.025])
        self.declare_parameter("publish_workspace_realign_service", True)

        self.workspace_frame = str(self.get_parameter("workspace_frame").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.board_width_m = float(self.get_parameter("board_width_m").value)
        self.board_depth_m = float(self.get_parameter("board_depth_m").value)
        self.board_thickness_m = float(self.get_parameter("board_thickness_m").value)
        self.board_tag_size_m = float(self.get_parameter("board_tag_size_m").value)
        self.board_tag_center_edge_offset_m = float(
            self.get_parameter("board_tag_center_edge_offset_m").value
        )
        self.cube_size_m = float(self.get_parameter("cube_size_m").value)
        self.min_cube_spacing_m = float(self.get_parameter("min_cube_spacing_m").value)
        self.randomise_yaw = bool(self.get_parameter("randomise_yaw").value)
        random_seed = int(self.get_parameter("random_seed").value)
        self._rng = random.Random(random_seed)

        self.camera_link_frame = str(self.get_parameter("camera_link_frame").value)
        self.camera_color_frame = str(self.get_parameter("camera_color_frame").value)
        self.camera_color_optical_frame = str(self.get_parameter("camera_color_optical_frame").value)
        self.camera_depth_frame = str(self.get_parameter("camera_depth_frame").value)
        self.camera_depth_optical_frame = str(self.get_parameter("camera_depth_optical_frame").value)
        self.camera_imu_frame = str(self.get_parameter("camera_imu_frame").value)
        self.camera_accel_frame = str(self.get_parameter("camera_accel_frame").value)
        self.camera_gyro_frame = str(self.get_parameter("camera_gyro_frame").value)
        self.camera_pose_xyzrpy = self._parse_camera_pose(self.get_parameter("camera_default_xyzrpy").value)
        self.camera_default_pose_xyzrpy = self.camera_pose_xyzrpy

        camera_body_size = [float(v) for v in self.get_parameter("camera_body_size_xyz").value]
        if len(camera_body_size) != 3:
            raise ValueError("camera_body_size_xyz must have 3 values")
        self.camera_body_size_xyz = camera_body_size

        default_cube_values = [float(v) for v in self.get_parameter("default_cube_centers_xyz_yaw").value]
        self.default_cubes = self._parse_default_cubes(default_cube_values)
        self.cubes: Dict[str, CubeState] = {name: replace(cube) for name, cube in self.default_cubes.items()}

        self._truth_pose_pubs = {
            name: self.create_publisher(PoseStamped, f"/holoassist/sim/truth/{name}_pose", 10)
            for name in CUBE_NAMES
        }
        self._truth_marker_pubs = {
            name: self.create_publisher(Marker, f"/holoassist/sim/truth/{name}_marker", 10)
            for name in CUBE_NAMES
        }
        self._camera_pose_pub = self.create_publisher(PoseStamped, "/holoassist/sim/truth/camera_pose", 10)
        self._camera_body_marker_pub = self.create_publisher(Marker, "/holoassist/sim/camera/body_marker", 10)
        self._camera_axis_marker_pub = self.create_publisher(Marker, "/holoassist/sim/camera/optical_axis_marker", 10)
        self._board_marker_pub = self.create_publisher(Marker, "/holoassist/sim/workspace/board_marker", 10)
        self._tags_marker_pub = self.create_publisher(Marker, "/holoassist/sim/workspace/tags_marker", 10)
        self._camera_info_pub = self.create_publisher(String, "/holoassist/sim/camera/info", 10)

        self._tf_broadcaster = TransformBroadcaster(self)
        self._im_server = InteractiveMarkerServer(self, "holoassist_sim_cube_truth_controls")

        for name in CUBE_NAMES:
            self._insert_cube_marker(name)

        self._insert_camera_marker()

        self._randomise_srv = self.create_service(
            Trigger,
            "/holoassist/sim/randomise_april_cubes",
            self._on_randomise,
        )
        self._reset_srv = self.create_service(
            Trigger,
            "/holoassist/sim/reset_april_cubes",
            self._on_reset,
        )
        self._set_cube_srv = self.create_service(
            SetAprilCubePose,
            "/holoassist/sim/set_april_cube_pose",
            self._on_set_cube_pose,
        )
        self._set_camera_srv = self.create_service(
            SetCameraPose,
            "/holoassist/sim/set_camera_pose",
            self._on_set_camera_pose,
        )
        self._reset_camera_srv = self.create_service(
            Trigger,
            "/holoassist/sim/reset_camera_pose",
            self._on_reset_camera_pose,
        )

        if bool(self.get_parameter("publish_workspace_realign_service").value):
            self._workspace_realign_srv = self.create_service(
                Trigger,
                "/holoassist/perception/realign_workspace",
                self._on_workspace_realign,
            )
        else:
            self._workspace_realign_srv = None

        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_all)
        self._publish_all()

        self.get_logger().info(
            "sim truth node started workspace_frame=%s camera_link=%s color_optical=%s cubes=%d"
            % (
                self.workspace_frame,
                self.camera_link_frame,
                self.camera_color_optical_frame,
                len(self.cubes),
            )
        )

    def _parse_camera_pose(self, values: List[float]) -> Tuple[float, float, float, float, float, float]:
        if len(values) != 6:
            raise ValueError("camera_default_xyzrpy must have 6 values")
        return tuple(float(v) for v in values)  # type: ignore[return-value]

    def _parse_default_cubes(self, values: List[float]) -> Dict[str, CubeState]:
        if len(values) != 16:
            raise ValueError("default_cube_centers_xyz_yaw must contain 16 values (4 cubes x x,y,z,yaw)")
        parsed: Dict[str, CubeState] = {}
        idx = 0
        for name in CUBE_NAMES:
            parsed[name] = CubeState(
                name=name,
                x=float(values[idx]),
                y=float(values[idx + 1]),
                z=float(values[idx + 2]),
                yaw=float(values[idx + 3]),
            )
            idx += 4
        return parsed

    def _cube_bounds(self) -> Tuple[float, float, float, float]:
        margin = self.cube_size_m / 2.0
        return (
            -self.board_width_m / 2.0 + margin,
            self.board_width_m / 2.0 - margin,
            -self.board_depth_m / 2.0 + margin,
            self.board_depth_m / 2.0 - margin,
        )

    def _resolve_cube_name(self, cube_name: str, cube_id: int) -> Optional[str]:
        if cube_name:
            clean = cube_name.strip().lower()
            for name in CUBE_NAMES:
                if name == clean:
                    return name
            return None
        if 1 <= cube_id <= len(CUBE_NAMES):
            return CUBE_NAMES[cube_id - 1]
        return None

    def _insert_cube_marker(self, name: str) -> None:
        cube = self.cubes[name]
        marker = InteractiveMarker()
        marker.header.frame_id = self.workspace_frame
        marker.name = name
        marker.description = name
        marker.scale = 0.10
        marker.pose = cube_state_to_pose(cube)

        cube_visual = Marker()
        cube_visual.type = Marker.CUBE
        cube_visual.scale.x = self.cube_size_m
        cube_visual.scale.y = self.cube_size_m
        cube_visual.scale.z = self.cube_size_m
        rgba = CUBE_COLORS[name]
        cube_visual.color.r = rgba[0]
        cube_visual.color.g = rgba[1]
        cube_visual.color.b = rgba[2]
        cube_visual.color.a = 1.0

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(cube_visual)
        marker.controls.append(visual_control)

        move_control = InteractiveMarkerControl()
        move_control.name = "move_xy"
        move_control.orientation.w = 1.0
        move_control.orientation.x = 0.0
        move_control.orientation.y = 1.0
        move_control.orientation.z = 0.0
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        marker.controls.append(move_control)

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "yaw"
        rotate_control.orientation.w = 1.0
        rotate_control.orientation.x = 0.0
        rotate_control.orientation.y = 0.0
        rotate_control.orientation.z = 1.0
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(rotate_control)

        self._im_server.insert(marker, feedback_callback=self._on_marker_feedback)
        self._im_server.applyChanges()

    def _insert_camera_marker(self) -> None:
        x, y, z, roll, pitch, yaw = self.camera_pose_xyzrpy
        q = quaternion_from_euler(roll, pitch, yaw)

        marker = InteractiveMarker()
        marker.header.frame_id = self.workspace_frame
        marker.name = "camera"
        marker.description = "sim camera"
        marker.scale = 0.15
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = float(q[0])
        marker.pose.orientation.y = float(q[1])
        marker.pose.orientation.z = float(q[2])
        marker.pose.orientation.w = float(q[3])

        cam_visual = Marker()
        cam_visual.type = Marker.CUBE
        cam_visual.scale.x = self.camera_body_size_xyz[0]
        cam_visual.scale.y = self.camera_body_size_xyz[1]
        cam_visual.scale.z = self.camera_body_size_xyz[2]
        cam_visual.color.r = 0.15
        cam_visual.color.g = 0.15
        cam_visual.color.b = 0.85
        cam_visual.color.a = 0.9

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(cam_visual)
        marker.controls.append(visual_control)

        # XY horizontal movement — same ring as cubes (plane perpendicular to world Z)
        move_xy = InteractiveMarkerControl()
        move_xy.name = "move_xy"
        move_xy.orientation.w = 1.0
        move_xy.orientation.y = 1.0
        move_xy.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        marker.controls.append(move_xy)

        # Z height adjustment — arrow along world Z
        move_z = InteractiveMarkerControl()
        move_z.name = "move_z"
        move_z.orientation.w = 1.0
        move_z.orientation.y = 1.0
        move_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(move_z)

        # Pitch (tilt up/down to re-aim after moving)
        rotate_pitch = InteractiveMarkerControl()
        rotate_pitch.name = "pitch"
        rotate_pitch.orientation.w = 1.0
        rotate_pitch.orientation.x = 1.0
        rotate_pitch.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(rotate_pitch)

        self._im_server.insert(marker, feedback_callback=self._on_camera_marker_feedback)
        self._im_server.applyChanges()

    def _update_camera_interactive_marker(self) -> None:
        x, y, z, roll, pitch, yaw = self.camera_pose_xyzrpy
        q = quaternion_from_euler(roll, pitch, yaw)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = float(q[0])
        pose.orientation.y = float(q[1])
        pose.orientation.z = float(q[2])
        pose.orientation.w = float(q[3])
        self._im_server.setPose("camera", pose)
        self._im_server.applyChanges()

    def _on_camera_marker_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        x = float(feedback.pose.position.x)
        y = float(feedback.pose.position.y)
        z = max(0.10, float(feedback.pose.position.z))
        q = feedback.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.camera_pose_xyzrpy = (x, y, z, float(roll), float(pitch), float(yaw))
        if z != float(feedback.pose.position.z):
            self._update_camera_interactive_marker()

    def _on_marker_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        name = feedback.marker_name
        if name not in self.cubes:
            return
        cube = self.cubes[name]
        x_min, x_max, y_min, y_max = self._cube_bounds()

        cube.x = max(x_min, min(x_max, float(feedback.pose.position.x)))
        cube.y = max(y_min, min(y_max, float(feedback.pose.position.y)))
        cube.z = self.cube_size_m / 2.0
        cube.yaw = normalize_angle(yaw_from_quaternion(feedback.pose.orientation))

        clamped_pose = cube_state_to_pose(cube)
        self._im_server.setPose(name, clamped_pose)
        self._im_server.applyChanges()

    def _publish_all(self) -> None:
        now = self.get_clock().now().to_msg()

        for idx, name in enumerate(CUBE_NAMES, start=1):
            cube = self.cubes[name]
            pose = cube_state_to_pose(cube)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = self.workspace_frame
            pose_msg.pose = pose
            self._truth_pose_pubs[name].publish(pose_msg)

            marker = Marker()
            marker.header = pose_msg.header
            marker.ns = "holoassist_sim_truth"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = self.cube_size_m
            marker.scale.y = self.cube_size_m
            marker.scale.z = self.cube_size_m
            rgba = CUBE_COLORS[name]
            marker.color.r = rgba[0]
            marker.color.g = rgba[1]
            marker.color.b = rgba[2]
            marker.color.a = 0.35
            self._truth_marker_pubs[name].publish(marker)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.workspace_frame
            tf_msg.child_frame_id = f"apriltag_cube_{idx}_sim_truth"
            tf_msg.transform.translation.x = cube.x
            tf_msg.transform.translation.y = cube.y
            tf_msg.transform.translation.z = cube.z
            tf_msg.transform.rotation = pose.orientation
            self._tf_broadcaster.sendTransform(tf_msg)

        self._publish_camera_tf(now)
        self._publish_camera_markers(now)
        self._publish_workspace_markers(now)

    def _publish_camera_tf(self, stamp) -> None:
        x, y, z, roll, pitch, yaw = self.camera_pose_xyzrpy
        q_cam = quaternion_from_euler(roll, pitch, yaw)

        camera_tf = TransformStamped()
        camera_tf.header.stamp = stamp
        camera_tf.header.frame_id = self.workspace_frame
        camera_tf.child_frame_id = self.camera_link_frame
        camera_tf.transform.translation.x = x
        camera_tf.transform.translation.y = y
        camera_tf.transform.translation.z = z
        camera_tf.transform.rotation.x = float(q_cam[0])
        camera_tf.transform.rotation.y = float(q_cam[1])
        camera_tf.transform.rotation.z = float(q_cam[2])
        camera_tf.transform.rotation.w = float(q_cam[3])

        optical_q = quaternion_from_euler(-math.pi / 2.0, 0.0, -math.pi / 2.0)

        color_tf = TransformStamped()
        color_tf.header.stamp = stamp
        color_tf.header.frame_id = self.camera_link_frame
        color_tf.child_frame_id = self.camera_color_frame
        color_tf.transform.translation.x = 0.0
        color_tf.transform.translation.y = 0.0
        color_tf.transform.translation.z = 0.0
        color_tf.transform.rotation.w = 1.0

        color_optical_tf = TransformStamped()
        color_optical_tf.header.stamp = stamp
        color_optical_tf.header.frame_id = self.camera_color_frame
        color_optical_tf.child_frame_id = self.camera_color_optical_frame
        color_optical_tf.transform.rotation.x = float(optical_q[0])
        color_optical_tf.transform.rotation.y = float(optical_q[1])
        color_optical_tf.transform.rotation.z = float(optical_q[2])
        color_optical_tf.transform.rotation.w = float(optical_q[3])

        depth_tf = TransformStamped()
        depth_tf.header.stamp = stamp
        depth_tf.header.frame_id = self.camera_link_frame
        depth_tf.child_frame_id = self.camera_depth_frame
        depth_tf.transform.translation.x = -0.015
        depth_tf.transform.translation.y = 0.0
        depth_tf.transform.translation.z = 0.0
        depth_tf.transform.rotation.w = 1.0

        depth_optical_tf = TransformStamped()
        depth_optical_tf.header.stamp = stamp
        depth_optical_tf.header.frame_id = self.camera_depth_frame
        depth_optical_tf.child_frame_id = self.camera_depth_optical_frame
        depth_optical_tf.transform.rotation.x = float(optical_q[0])
        depth_optical_tf.transform.rotation.y = float(optical_q[1])
        depth_optical_tf.transform.rotation.z = float(optical_q[2])
        depth_optical_tf.transform.rotation.w = float(optical_q[3])

        imu_tf = TransformStamped()
        imu_tf.header.stamp = stamp
        imu_tf.header.frame_id = self.camera_link_frame
        imu_tf.child_frame_id = self.camera_imu_frame
        imu_tf.transform.translation.x = 0.0
        imu_tf.transform.translation.y = 0.0
        imu_tf.transform.translation.z = -0.010
        imu_tf.transform.rotation.w = 1.0

        accel_tf = TransformStamped()
        accel_tf.header.stamp = stamp
        accel_tf.header.frame_id = self.camera_imu_frame
        accel_tf.child_frame_id = self.camera_accel_frame
        accel_tf.transform.rotation.w = 1.0

        gyro_tf = TransformStamped()
        gyro_tf.header.stamp = stamp
        gyro_tf.header.frame_id = self.camera_imu_frame
        gyro_tf.child_frame_id = self.camera_gyro_frame
        gyro_tf.transform.rotation.w = 1.0

        self._tf_broadcaster.sendTransform(
            [
                camera_tf,
                color_tf,
                color_optical_tf,
                depth_tf,
                depth_optical_tf,
                imu_tf,
                accel_tf,
                gyro_tf,
            ]
        )

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.workspace_frame
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = float(q_cam[0])
        pose_msg.pose.orientation.y = float(q_cam[1])
        pose_msg.pose.orientation.z = float(q_cam[2])
        pose_msg.pose.orientation.w = float(q_cam[3])
        self._camera_pose_pub.publish(pose_msg)

        info = String()
        info.data = (
            f"camera_link=({x:.3f},{y:.3f},{z:.3f}) rpy=({roll:.3f},{pitch:.3f},{yaw:.3f}) "
            f"color_optical={self.camera_color_optical_frame} depth_optical={self.camera_depth_optical_frame}"
        )
        self._camera_info_pub.publish(info)

    def _publish_camera_markers(self, stamp) -> None:
        body = Marker()
        body.header.stamp = stamp
        body.header.frame_id = self.camera_link_frame
        body.ns = "holoassist_sim_camera"
        body.id = 100
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.scale.x = self.camera_body_size_xyz[0]
        body.scale.y = self.camera_body_size_xyz[1]
        body.scale.z = self.camera_body_size_xyz[2]
        body.color.r = 0.12
        body.color.g = 0.12
        body.color.b = 0.12
        body.color.a = 0.95
        self._camera_body_marker_pub.publish(body)

        axis = Marker()
        axis.header.stamp = stamp
        axis.header.frame_id = self.camera_color_optical_frame
        axis.ns = "holoassist_sim_camera"
        axis.id = 101
        axis.type = Marker.ARROW
        axis.action = Marker.ADD
        axis.scale.x = 0.12
        axis.scale.y = 0.008
        axis.scale.z = 0.012
        axis.color.r = 1.0
        axis.color.g = 0.1
        axis.color.b = 0.9
        axis.color.a = 0.95
        self._camera_axis_marker_pub.publish(axis)

    def _publish_workspace_markers(self, stamp) -> None:
        board = Marker()
        board.header.stamp = stamp
        board.header.frame_id = self.workspace_frame
        board.ns = "holoassist_sim_workspace"
        board.id = 300
        board.type = Marker.CUBE
        board.action = Marker.ADD
        board.pose.position.z = -0.5 * self.board_thickness_m
        board.pose.orientation.w = 1.0
        board.scale.x = self.board_width_m
        board.scale.y = self.board_depth_m
        board.scale.z = self.board_thickness_m
        board.color.r = 0.05
        board.color.g = 0.05
        board.color.b = 0.05
        board.color.a = 0.95
        self._board_marker_pub.publish(board)

        edge = self.board_tag_center_edge_offset_m
        x_min = -self.board_width_m * 0.5 + edge
        x_max = self.board_width_m * 0.5 - edge
        y_min = -self.board_depth_m * 0.5 + edge
        y_max = self.board_depth_m * 0.5 - edge

        tags = Marker()
        tags.header.stamp = stamp
        tags.header.frame_id = self.workspace_frame
        tags.ns = "holoassist_sim_workspace"
        tags.id = 301
        tags.type = Marker.CUBE_LIST
        tags.action = Marker.ADD
        tags.pose.orientation.w = 1.0
        tags.scale.x = self.board_tag_size_m
        tags.scale.y = self.board_tag_size_m
        tags.scale.z = max(0.001, self.board_thickness_m * 0.15)
        tags.color.r = 1.0
        tags.color.g = 1.0
        tags.color.b = 1.0
        tags.color.a = 0.98
        tags.points = [
            Point(x=x_min, y=y_min, z=0.0005),
            Point(x=x_max, y=y_min, z=0.0005),
            Point(x=x_min, y=y_max, z=0.0005),
            Point(x=x_max, y=y_max, z=0.0005),
        ]
        self._tags_marker_pub.publish(tags)

    def _update_interactive_marker_pose(self, name: str) -> None:
        self._im_server.setPose(name, cube_state_to_pose(self.cubes[name]))
        self._im_server.applyChanges()

    def _on_randomise(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        success, msg = self._randomise_all_cubes()
        response.success = success
        response.message = msg
        if success:
            for name in CUBE_NAMES:
                self._update_interactive_marker_pose(name)
            self._publish_all()
        return response

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.cubes = {name: replace(cube) for name, cube in self.default_cubes.items()}
        for name in CUBE_NAMES:
            self._update_interactive_marker_pose(name)
        self._publish_all()
        response.success = True
        response.message = "reset to default cube poses"
        return response

    def _on_workspace_realign(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._publish_all()
        response.success = True
        response.message = "sim workspace realign acknowledged"
        return response

    def _on_set_cube_pose(self, request: SetAprilCubePose.Request, response: SetAprilCubePose.Response) -> SetAprilCubePose.Response:
        name = self._resolve_cube_name(str(request.cube_name), int(request.cube_id))
        if name is None:
            response.success = False
            response.message = "cube not found; use cube_name=april_cube_1..4 or cube_id=1..4"
            return response

        x_min, x_max, y_min, y_max = self._cube_bounds()
        if not (x_min <= request.x <= x_max and y_min <= request.y <= y_max):
            response.success = False
            response.message = "requested pose is outside board bounds for full cube footprint"
            return response

        cube = self.cubes[name]
        cube.x = float(request.x)
        cube.y = float(request.y)
        cube.z = float(request.z)
        cube.yaw = normalize_angle(float(request.yaw))

        self._update_interactive_marker_pose(name)
        self._publish_all()

        response.success = True
        response.message = f"updated {name}"
        response.pose.header.frame_id = self.workspace_frame
        response.pose.header.stamp = self.get_clock().now().to_msg()
        response.pose.pose = cube_state_to_pose(cube)
        return response

    def _on_set_camera_pose(self, request: SetCameraPose.Request, response: SetCameraPose.Response) -> SetCameraPose.Response:
        self.camera_pose_xyzrpy = (
            float(request.x),
            float(request.y),
            float(request.z),
            float(request.roll),
            float(request.pitch),
            float(request.yaw),
        )

        now = self.get_clock().now().to_msg()
        self._publish_camera_tf(now)
        self._publish_camera_markers(now)
        self._update_camera_interactive_marker()

        q_cam = quaternion_from_euler(request.roll, request.pitch, request.yaw)
        response.success = True
        response.message = "camera pose updated"
        response.pose.header.stamp = now
        response.pose.header.frame_id = self.workspace_frame
        response.pose.pose.position.x = float(request.x)
        response.pose.pose.position.y = float(request.y)
        response.pose.pose.position.z = float(request.z)
        response.pose.pose.orientation.x = float(q_cam[0])
        response.pose.pose.orientation.y = float(q_cam[1])
        response.pose.pose.orientation.z = float(q_cam[2])
        response.pose.pose.orientation.w = float(q_cam[3])
        return response

    def _on_reset_camera_pose(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.camera_pose_xyzrpy = self.camera_default_pose_xyzrpy
        now = self.get_clock().now().to_msg()
        self._publish_camera_tf(now)
        self._publish_camera_markers(now)
        self._update_camera_interactive_marker()
        response.success = True
        response.message = (
            "camera pose reset to default " f"{tuple(round(v, 3) for v in self.camera_default_pose_xyzrpy)}"
        )
        return response

    def _randomise_all_cubes(self) -> Tuple[bool, str]:
        x_min, x_max, y_min, y_max = self._cube_bounds()
        names = list(CUBE_NAMES)
        sampled: Dict[str, CubeState] = {}

        for name in names:
            placed = False
            for _ in range(200):
                x = self._rng.uniform(x_min, x_max)
                y = self._rng.uniform(y_min, y_max)
                if all(
                    math.hypot(x - other.x, y - other.y) >= self.min_cube_spacing_m
                    for other in sampled.values()
                ):
                    yaw = self._rng.uniform(-math.pi, math.pi) if self.randomise_yaw else 0.0
                    sampled[name] = CubeState(name=name, x=x, y=y, z=self.cube_size_m / 2.0, yaw=yaw)
                    placed = True
                    break
            if not placed:
                return False, "failed to randomise without overlap; lower min_cube_spacing_m"

        self.cubes = sampled
        return True, "randomised cube poses"


def main() -> None:
    rclpy.init()
    node = SimCubeTruthNode()
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
