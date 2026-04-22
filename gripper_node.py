#!/usr/bin/env python3
"""
ROS 2 node for OnRobot RG2 gripper control.

Subscribes to /gripper/command (Float32, 0.0=open → 1.0=closed) from Unity
and sends width commands to the RG2 via URScript on the UR driver.

The RG2 width range is 0–110 mm. This node maps:
  0.0 (trigger released) → 110 mm (fully open)
  1.0 (trigger fully pressed) → 0 mm (fully closed)

Requires: OnRobot RG2 URCap installed on teach pendant.
The UR driver must be running with /urscript_interface/script_command topic available.

Usage:
  ros2 run ros_tcp_endpoint default_server_endpoint  # (already running)
  python3 gripper_node.py
  python3 gripper_node.py --force 40    # custom grip force (N, default 40)
  python3 gripper_node.py --dry-run     # print commands without sending
"""

import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


RG2_WIDTH_MIN = 0.0    # mm (fully closed)
RG2_WIDTH_MAX = 110.0  # mm (fully open)
PUBLISH_RATE = 10.0    # Hz — rate-limit URScript commands


class GripperNode(Node):
    def __init__(self, force: float, dry_run: bool):
        super().__init__('gripper_controller')
        self.force = force
        self.dry_run = dry_run
        self.last_width_mm = -1.0
        self.target_width_mm = RG2_WIDTH_MAX

        self.sub = self.create_subscription(
            Float32, '/gripper/command', self.on_gripper_command, 10)

        if not dry_run:
            self.script_pub = self.create_publisher(
                String, '/urscript_interface/script_command', 10)

        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.publish_command)
        self.get_logger().info(
            f'Gripper node ready (force={force}N, dry_run={dry_run})')

    def on_gripper_command(self, msg: Float32):
        value = max(0.0, min(1.0, msg.data))
        self.target_width_mm = RG2_WIDTH_MAX - value * (RG2_WIDTH_MAX - RG2_WIDTH_MIN)

    def publish_command(self):
        width = self.target_width_mm
        # Only send if width changed by more than 0.5mm
        if abs(width - self.last_width_mm) < 0.5:
            return
        self.last_width_mm = width

        script = f'rg_grip({width:.1f}, {self.force:.0f})'

        if self.dry_run:
            self.get_logger().info(f'[dry-run] {script}')
            return

        msg = String()
        msg.data = script
        self.script_pub.publish(msg)
        self.get_logger().info(f'Gripper → {width:.1f} mm')


def main():
    parser = argparse.ArgumentParser(description='OnRobot RG2 gripper ROS 2 node')
    parser.add_argument('--force', type=float, default=40.0,
                        help='Grip force in Newtons (default: 40)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Log commands without publishing URScript')
    args = parser.parse_args()

    rclpy.init()
    node = GripperNode(force=args.force, dry_run=args.dry_run)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
