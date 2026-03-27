#!/usr/bin/env python3
import argparse

import numpy as np
import rclpy

from holoassist_movement.movement_sequence import DEFAULT_MOVE_DURATION
from holoassist_movement.robot_demo_control import JOINT_NAMES, RobotDemoControl


def build_argument_parser():
    parser = argparse.ArgumentParser(description='Move the robot to a single joint target.')
    parser.add_argument(
        '--duration',
        type=float,
        default=DEFAULT_MOVE_DURATION,
        help='Seconds to reach the target joint angles.',
    )
    parser.add_argument(
        '--angles-deg',
        nargs=6,
        type=float,
        metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6'),
        required=True,
        help='Six joint angles in degrees.',
    )
    return parser


def main(argv=None):
    rclpy.init(args=argv)
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    node = RobotDemoControl()
    try:
        target_joint_positions = np.deg2rad(args.angles_deg).tolist()
        print(f'[APP] Moving {JOINT_NAMES} to {args.angles_deg} deg')
        node.move_ur_joint_positions(target_joint_positions, duration=args.duration)
    except KeyboardInterrupt:
        print('\n[APP] Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
