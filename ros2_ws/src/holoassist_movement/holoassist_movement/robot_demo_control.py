#!/usr/bin/env python3
import argparse
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from holoassist_movement.movement_sequence import DEFAULT_MOVE_DURATION


JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


class RobotDemoControl(Node):
    def __init__(self):
        super().__init__('robot_demo_control')

        self.current_pos = None
        self.create_subscription(
            JointState,
            '/ur/joint_states',
            self._joint_state_cb,
            10,
        )
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/ur/scaled_pos_joint_traj_controller/command',
            10,
        )
        self.open_client = self.create_client(Trigger, '/onrobot/open')
        self.close_client = self.create_client(Trigger, '/onrobot/close')

    def _joint_state_cb(self, message):
        self.current_pos = list(message.position)

    def wait_for_joint_state(self, timeout=5.0):
        """Wait for and return the current joint positions."""
        self.current_pos = None
        self.get_logger().info('Waiting for current joint state...')
        start_time = time.time()
        while self.current_pos is None and time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_pos is None:
            raise RuntimeError('No joint state received from /ur/joint_states')

        self.get_logger().info(f'Current joint positions: {self.current_pos}')
        return list(self.current_pos)

    def call_service(self, service_name, timeout=5.0):
        """Call a std_srvs/Trigger gripper service."""
        if service_name == '/onrobot/open':
            client = self.open_client
        elif service_name == '/onrobot/close':
            client = self.close_client
        else:
            raise ValueError(f'Unsupported service name: {service_name}')

        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Service not available: {service_name}')

        self.get_logger().info(f'Calling service: {service_name}')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        result = future.result()
        if result is None:
            raise RuntimeError(f'Service call failed: {service_name}')

        self.get_logger().info(
            f"Response: success={result.success}, message='{result.message}'"
        )
        return result

    def move_ur_joint_positions(self, joint_positions, duration=5.0):
        current_pos = self.wait_for_joint_state()

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = JOINT_NAMES

        start_point = JointTrajectoryPoint()
        start_point.positions = current_pos
        start_point.time_from_start = Duration(sec=0, nanosec=0)

        end_point = JointTrajectoryPoint()
        end_point.positions = joint_positions
        end_point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration - int(duration)) * 1e9),
        )

        trajectory_msg.points = [start_point, end_point]
        self.trajectory_pub.publish(trajectory_msg)
        self.get_logger().info('Trajectory published.')

        time.sleep(duration + 1.0)


def build_argument_parser():
    parser = argparse.ArgumentParser(description='Run the HoloAssist robot motion demo.')
    parser.add_argument(
        '--duration',
        type=float,
        default=DEFAULT_MOVE_DURATION,
        help='Default seconds to reach each joint waypoint.',
    )
    return parser


def main(argv=None):
    from holoassist_movement.demo_runner import run_demo

    rclpy.init(args=argv)
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    node = RobotDemoControl()
    try:
        run_demo(node, args.duration)
    except KeyboardInterrupt:
        print('\n[APP] Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
