#!/usr/bin/env python3

from __future__ import annotations

import copy
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

# Registers geometry message conversions for tf2 Buffer.transform.
import tf2_geometry_msgs  # noqa: F401


class ObstacleToObjectPoseAdapter(Node):
    """Adapter that projects depth-tracker obstacle markers into object-pose topics."""

    def __init__(self) -> None:
        super().__init__("obstacle_to_object_pose_adapter")

        self.declare_parameter(
            "obstacle_marker_topic", "/holo_assist_depth_tracker/obstacle_marker"
        )
        self.declare_parameter(
            "object_pose_topic", "/holoassist/perception/object_pose"
        )
        self.declare_parameter(
            "object_marker_topic", "/holoassist/perception/object_marker"
        )
        self.declare_parameter(
            "object_pose_workspace_topic", "/holoassist/perception/object_pose_workspace"
        )
        self.declare_parameter("workspace_frame", "base_link")
        self.declare_parameter("publish_workspace_pose", True)
        self.declare_parameter("workspace_tf_timeout_s", 0.15)
        self.declare_parameter("output_marker_ns", "holoassist_object")
        self.declare_parameter("output_marker_id", 0)

        self.obstacle_marker_topic = str(
            self.get_parameter("obstacle_marker_topic").value
        )
        self.object_pose_topic = str(self.get_parameter("object_pose_topic").value)
        self.object_marker_topic = str(self.get_parameter("object_marker_topic").value)
        self.object_pose_workspace_topic = str(
            self.get_parameter("object_pose_workspace_topic").value
        )
        self.workspace_frame = str(self.get_parameter("workspace_frame").value)
        self.publish_workspace_pose = bool(
            self.get_parameter("publish_workspace_pose").value
        )
        self.workspace_tf_timeout_s = float(
            self.get_parameter("workspace_tf_timeout_s").value
        )
        self.output_marker_ns = str(self.get_parameter("output_marker_ns").value)
        self.output_marker_id = int(self.get_parameter("output_marker_id").value)

        if self.workspace_tf_timeout_s <= 0.0:
            self.workspace_tf_timeout_s = 0.15

        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.VOLATILE

        self.object_pose_pub = self.create_publisher(
            PoseStamped,
            self.object_pose_topic,
            pub_qos,
        )
        self.object_marker_pub = self.create_publisher(
            Marker,
            self.object_marker_topic,
            pub_qos,
        )

        self.workspace_pose_pub: Optional[Publisher] = None
        if self.publish_workspace_pose:
            self.workspace_pose_pub = self.create_publisher(
                PoseStamped,
                self.object_pose_workspace_topic,
                pub_qos,
            )

        self.tf_buffer: Optional[Buffer] = None
        self.tf_listener: Optional[TransformListener] = None
        if self.publish_workspace_pose:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.obstacle_sub = self.create_subscription(
            Marker,
            self.obstacle_marker_topic,
            self._on_obstacle_marker,
            pub_qos,
        )

        self.get_logger().info(
            "object pose adapter started: obstacle=%s pose=%s marker=%s workspace_pose=%s"
            % (
                self.obstacle_marker_topic,
                self.object_pose_topic,
                self.object_marker_topic,
                self.object_pose_workspace_topic,
            )
        )

    def _on_obstacle_marker(self, msg: Marker) -> None:
        output_marker = copy.deepcopy(msg)
        output_marker.ns = self.output_marker_ns
        output_marker.id = self.output_marker_id
        self.object_marker_pub.publish(output_marker)

        if msg.action in (Marker.DELETE, Marker.DELETEALL):
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose
        self.object_pose_pub.publish(pose)

        if (
            not self.publish_workspace_pose
            or self.workspace_pose_pub is None
            or self.tf_buffer is None
        ):
            return

        if not pose.header.frame_id:
            return

        if pose.header.frame_id == self.workspace_frame:
            self.workspace_pose_pub.publish(pose)
            return

        try:
            transformed = self.tf_buffer.transform(
                pose,
                self.workspace_frame,
                timeout=Duration(seconds=self.workspace_tf_timeout_s),
            )
            self.workspace_pose_pub.publish(transformed)
        except TransformException as exc:
            self.get_logger().warn(
                "workspace transform unavailable (%s -> %s): %s"
                % (pose.header.frame_id, self.workspace_frame, exc),
                throttle_duration_sec=2.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleToObjectPoseAdapter()
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
