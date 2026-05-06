#!/usr/bin/env python3
"""
Relays cube poses from workspace_frame to base_link for Unity consumption.

Subscribes to /holoassist/perception/april_cube_{1-4}_pose (PoseStamped in workspace_frame)
and republishes as /holoassist/unity/cube_{1-4}_pose (PoseStamped in base_link) using TF.

Unity's CubePoseSubscriber.cs subscribes to the relayed topics.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import numpy as np


class CubePoseRelay(Node):
    def __init__(self):
        super().__init__("cube_pose_relay")

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("cube_count", 4)
        self.declare_parameter("input_prefix", "/holoassist/perception")

        self.target_frame = str(self.get_parameter("target_frame").value)
        cube_count = int(self.get_parameter("cube_count").value)
        input_prefix = str(self.get_parameter("input_prefix").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pubs = {}
        for i in range(1, cube_count + 1):
            name = f"april_cube_{i}"
            in_topic = f"{input_prefix}/{name}_pose"
            out_topic = f"/holoassist/unity/cube_{i}_pose"

            self.pubs[name] = self.create_publisher(PoseStamped, out_topic, 10)
            self.create_subscription(
                PoseStamped, in_topic,
                lambda msg, n=name: self._on_pose(n, msg), 10
            )
            self.get_logger().info(f"Relaying {in_topic} -> {out_topic}")

    def _on_pose(self, name: str, msg: PoseStamped):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame

        t = tf.transform.translation
        r = tf.transform.rotation

        rot = self._quat_to_matrix(r.x, r.y, r.z, r.w)
        p_in = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        p_out = rot @ p_in + np.array([t.x, t.y, t.z])

        out.pose.position.x = p_out[0]
        out.pose.position.y = p_out[1]
        out.pose.position.z = p_out[2]

        q_tf = np.array([r.x, r.y, r.z, r.w])
        q_in = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])
        q_out = self._quat_multiply(q_tf, q_in)
        out.pose.orientation.x = q_out[0]
        out.pose.orientation.y = q_out[1]
        out.pose.orientation.z = q_out[2]
        out.pose.orientation.w = q_out[3]

        self.pubs[name].publish(out)

    @staticmethod
    def _quat_to_matrix(x, y, z, w):
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])

    @staticmethod
    def _quat_multiply(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
        ])


def main():
    rclpy.init()
    node = CubePoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
