#!/usr/bin/env python3
import argparse

import numpy as np
import rclpy

from holoassist_movement.robot_demo_control import JOINT_NAMES, RobotDemoControl


def build_argument_parser():
    parser = argparse.ArgumentParser(description='Print the current UR joint state.')
    parser.add_argument(
        '--timeout',
        type=float,
        default=5.0,
        help='Seconds to wait for a joint state message.',
    )
    return parser


def main(argv=None):
    rclpy.init(args=argv)
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    node = RobotDemoControl()
    try:
        joint_positions_rad = node.wait_for_joint_state(timeout=args.timeout)
        joint_positions_deg = np.rad2deg(joint_positions_rad).tolist()

        print('[APP] Current joint positions')
        for name, rad, deg in zip(JOINT_NAMES, joint_positions_rad, joint_positions_deg):
            print(f'  {name}: {rad:.6f} rad | {deg:.2f} deg')

        print(f'[APP] Angles for movement_sequence.py: {joint_positions_deg}')
    except KeyboardInterrupt:
        print('\n[APP] Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
