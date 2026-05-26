#!/usr/bin/env python3
"""Publish synthetic tag-1 observations for eye-on-base calibration in simulation."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf_transformations import euler_matrix, quaternion_from_matrix, quaternion_matrix


def transform_matrix(xyz, quaternion=None, rpy_deg=None):
    if quaternion is not None:
        matrix = quaternion_matrix(quaternion)
    else:
        rpy = [math.radians(value) for value in rpy_deg]
        matrix = euler_matrix(*rpy)
    matrix[:3, 3] = xyz
    return matrix


class SyntheticMarkerNode(Node):
    def __init__(self):
        super().__init__("holoassist_calibration_sim_marker")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("effector_frame", "tool0")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("marker_frame", "tag36h11:1")
        self.declare_parameter("camera_xyz", [0.10, 0.68, 0.80])
        self.declare_parameter("camera_rpy_deg", [180.0, 0.0, 180.0])
        self.declare_parameter("tool_marker_xyz", [0.0, 0.0, 0.08])
        self.declare_parameter("tool_marker_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("publish_hz", 20.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.effector_frame = self.get_parameter("effector_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.marker_frame = self.get_parameter("marker_frame").value
        self.base_to_camera = transform_matrix(
            self.get_parameter("camera_xyz").value,
            rpy_deg=self.get_parameter("camera_rpy_deg").value,
        )
        self.tool_to_marker = transform_matrix(
            self.get_parameter("tool_marker_xyz").value,
            rpy_deg=self.get_parameter("tool_marker_rpy_deg").value,
        )

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self._waiting_logged = False
        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / max(hz, 1.0), self.publish_observation)
        self.get_logger().info(
            f"Generating simulated observations for {self.camera_frame} -> "
            f"{self.marker_frame}; no base-to-camera calibration TF is published"
        )

    def publish_observation(self):
        try:
            robot_tf = self.buffer.lookup_transform(
                self.base_frame, self.effector_frame, Time()
            )
        except tf2_ros.TransformException:
            if not self._waiting_logged:
                self.get_logger().info("Waiting for robot TF before publishing simulated tag")
                self._waiting_logged = True
            return

        t = robot_tf.transform.translation
        q = robot_tf.transform.rotation
        base_to_tool = transform_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
        camera_to_marker = (
            np.linalg.inv(self.base_to_camera) @ base_to_tool @ self.tool_to_marker
        )
        marker_q = quaternion_from_matrix(camera_to_marker)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.child_frame_id = self.marker_frame
        msg.transform.translation.x = float(camera_to_marker[0, 3])
        msg.transform.translation.y = float(camera_to_marker[1, 3])
        msg.transform.translation.z = float(camera_to_marker[2, 3])
        msg.transform.rotation.x = float(marker_q[0])
        msg.transform.rotation.y = float(marker_q[1])
        msg.transform.rotation.z = float(marker_q[2])
        msg.transform.rotation.w = float(marker_q[3])
        self.broadcaster.sendTransform(msg)


def main():
    rclpy.init()
    node = SyntheticMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
