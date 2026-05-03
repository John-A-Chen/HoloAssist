#!/usr/bin/env python3

from __future__ import annotations

from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

from holo_assist_depth_tracker_sim.common import CUBE_NAMES
from holo_assist_depth_tracker_sim_interfaces.msg import CubePerceptionStatus


class SimCubeMoveItBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("holoassist_sim_cube_moveit_bridge")

        self.declare_parameter("workspace_frame", "workspace_frame")
        self.declare_parameter("cube_size_m", 0.040)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("planning_scene_topic", "/planning_scene")
        self.declare_parameter("selected_cube_topic", "/holoassist/teleop/selected_cube")
        self.declare_parameter("selected_cube_pose_topic", "/holoassist/teleop/selected_cube_pose")

        self.workspace_frame = str(self.get_parameter("workspace_frame").value)
        self.cube_size_m = float(self.get_parameter("cube_size_m").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.planning_scene_topic = str(self.get_parameter("planning_scene_topic").value)
        self.selected_cube_topic = str(self.get_parameter("selected_cube_topic").value)
        self.selected_cube_pose_topic = str(self.get_parameter("selected_cube_pose_topic").value)

        self._perceived_pose: Dict[str, Optional[PoseStamped]] = {name: None for name in CUBE_NAMES}
        self._last_status: Dict[str, Optional[CubePerceptionStatus]] = {name: None for name in CUBE_NAMES}
        self._selected_cube: Optional[str] = None

        self._planning_scene_pub = self.create_publisher(PlanningScene, self.planning_scene_topic, 10)
        self._selected_pose_pub = self.create_publisher(PoseStamped, self.selected_cube_pose_topic, 10)

        for name in CUBE_NAMES:
            self.create_subscription(
                PoseStamped,
                f"/holoassist/sim/perception/{name}_pose",
                self._make_pose_cb(name),
                10,
            )
            self.create_subscription(
                CubePerceptionStatus,
                f"/holoassist/sim/perception/{name}_status",
                self._make_status_cb(name),
                10,
            )

        self.create_subscription(String, self.selected_cube_topic, self._on_selected_cube, 10)

        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            "sim moveit bridge started planning_scene_topic=%s selected_cube_topic=%s"
            % (self.planning_scene_topic, self.selected_cube_topic)
        )

    def _make_pose_cb(self, cube_name: str):
        def _cb(msg: PoseStamped) -> None:
            self._perceived_pose[cube_name] = msg

        return _cb

    def _make_status_cb(self, cube_name: str):
        def _cb(msg: CubePerceptionStatus) -> None:
            self._last_status[cube_name] = msg

        return _cb

    def _on_selected_cube(self, msg: String) -> None:
        name = msg.data.strip().lower()
        if name not in CUBE_NAMES:
            self.get_logger().warn(
                "selected cube '%s' invalid; expected one of %s" % (msg.data, ", ".join(CUBE_NAMES))
            )
            return
        self._selected_cube = name

    def _on_timer(self) -> None:
        self._publish_planning_scene_update()
        self._publish_selected_cube_pose()

    def _publish_planning_scene_update(self) -> None:
        scene = PlanningScene()
        scene.is_diff = True

        for name in CUBE_NAMES:
            pose_msg = self._perceived_pose[name]
            if pose_msg is None:
                continue

            obj = CollisionObject()
            obj.header.frame_id = self.workspace_frame
            obj.header.stamp = self.get_clock().now().to_msg()
            obj.id = name

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [self.cube_size_m, self.cube_size_m, self.cube_size_m]
            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose_msg.pose)
            obj.operation = CollisionObject.ADD

            scene.world.collision_objects.append(obj)

        if scene.world.collision_objects:
            self._planning_scene_pub.publish(scene)

    def _publish_selected_cube_pose(self) -> None:
        if self._selected_cube is None:
            return

        pose_msg = self._perceived_pose.get(self._selected_cube)
        if pose_msg is None:
            status = self._last_status.get(self._selected_cube)
            if status is not None and not status.last_seen_available:
                self.get_logger().warn(
                    "selected cube %s has never been seen yet" % self._selected_cube,
                    throttle_duration_sec=3.0,
                )
            return

        status = self._last_status.get(self._selected_cube)
        if status is not None and not status.visible_now:
            self.get_logger().warn(
                "selected cube %s currently hidden; publishing last seen pose" % self._selected_cube,
                throttle_duration_sec=5.0,
            )

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.workspace_frame
        out.pose = pose_msg.pose
        self._selected_pose_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = SimCubeMoveItBridgeNode()
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
