#!/usr/bin/env python3
"""
Publishes fake cube poses directly to the Unity relay topics.
No camera, no AprilTags, no workspace_frame needed.
Cubes orbit slowly in front of the robot so you can see them in Unity.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time


class TestCubePublisher(Node):
    def __init__(self):
        super().__init__("test_cube_publisher")
        self.cube_count = 4
        self.pubs = {}

        for i in range(1, self.cube_count + 1):
            topic = f"/holoassist/unity/cube_{i}_pose"
            self.pubs[i] = self.create_publisher(PoseStamped, topic, 10)
            self.get_logger().info(f"Publishing fake cube on {topic}")

        self.timer = self.create_timer(0.1, self.publish_cubes)
        self.t0 = time.time()

    def publish_cubes(self):
        t = time.time() - self.t0

        positions = [
            (0.3, 0.15, 0.05),   # cube 1: front-left
            (0.3, -0.15, 0.05),  # cube 2: front-right
            (0.4, 0.0, 0.05),    # cube 3: centre-far
            (0.25, 0.0, 0.10),   # cube 4: centre-near, raised
        ]

        for i in range(1, self.cube_count + 1):
            bx, by, bz = positions[i - 1]
            # gentle orbit so cubes visibly move
            x = bx + 0.03 * math.sin(t * 0.5 + i)
            y = by + 0.03 * math.cos(t * 0.5 + i)
            z = bz

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.w = 1.0

            self.pubs[i].publish(msg)


def main():
    rclpy.init()
    node = TestCubePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
