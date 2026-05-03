#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler


class WorkspaceFrameStaticTfNode(Node):
    """Publish a tunable static TF from robot planning frame to workspace frame."""

    def __init__(self) -> None:
        super().__init__("holoassist_workspace_frame_tf")

        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("child_frame", "workspace_frame")
        self.declare_parameter("x_m", -0.10)
        self.declare_parameter("y_m", -0.314)
        self.declare_parameter("z_m", 0.015)
        self.declare_parameter("roll_rad", 0.0)
        self.declare_parameter("pitch_rad", 0.0)
        self.declare_parameter("yaw_rad", 0.0)

        parent_frame = str(self.get_parameter("parent_frame").value)
        child_frame = str(self.get_parameter("child_frame").value)
        x_m = float(self.get_parameter("x_m").value)
        y_m = float(self.get_parameter("y_m").value)
        z_m = float(self.get_parameter("z_m").value)
        roll_rad = float(self.get_parameter("roll_rad").value)
        pitch_rad = float(self.get_parameter("pitch_rad").value)
        yaw_rad = float(self.get_parameter("yaw_rad").value)

        qx, qy, qz, qw = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = x_m
        tf_msg.transform.translation.y = y_m
        tf_msg.transform.translation.z = z_m
        tf_msg.transform.rotation.x = float(qx)
        tf_msg.transform.rotation.y = float(qy)
        tf_msg.transform.rotation.z = float(qz)
        tf_msg.transform.rotation.w = float(qw)

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(tf_msg)

        self.get_logger().info(
            (
                "Published static TF %s -> %s xyz=(%.3f, %.3f, %.3f) "
                "rpy_deg=(%.1f, %.1f, %.1f)"
            )
            % (
                parent_frame,
                child_frame,
                x_m,
                y_m,
                z_m,
                math.degrees(roll_rad),
                math.degrees(pitch_rad),
                math.degrees(yaw_rad),
            )
        )


def main() -> None:
    rclpy.init()
    node = WorkspaceFrameStaticTfNode()
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
