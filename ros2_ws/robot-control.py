#!/usr/bin/env python3
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_STATES_TOPIC = "/joint_states"
TRAJECTORY_TOPIC = "/scaled_joint_trajectory_controller/joint_trajectory"
UR_JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class RobotControl(Node):
    def __init__(self) -> None:
        super().__init__("robot_control")
        self.current_pos: Optional[List[float]] = None

        self.trajectory_pub = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self.joint_state_cb, 10)

    def joint_state_cb(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            return

        joint_map = {
            joint_name: joint_position
            for joint_name, joint_position in zip(msg.name, msg.position)
        }

        if not all(joint_name in joint_map for joint_name in UR_JOINT_ORDER):
            return

        self.current_pos = [joint_map[joint_name] for joint_name in UR_JOINT_ORDER]

    def wait_for_joint_state(self, timeout_sec: float = 5.0) -> List[float]:
        self.current_pos = None
        self.get_logger().info("Waiting for current joint state...")
        deadline = time.time() + timeout_sec

        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_pos is not None:
                return list(self.current_pos)

        raise RuntimeError(f"No joint state received from {JOINT_STATES_TOPIC}")

    def move_ur_joint_positions(self, joint_positions: List[float], duration: float = 5.0) -> None:
        current_pos = self.wait_for_joint_state()
        self.get_logger().info(f"Current joint positions: {current_pos}")

        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.joint_names = list(UR_JOINT_ORDER)

        start_point = JointTrajectoryPoint()
        start_point.positions = list(current_pos)
        start_point.time_from_start.sec = 0
        start_point.time_from_start.nanosec = 0

        target_point = JointTrajectoryPoint()
        target_point.positions = list(joint_positions)
        target_point.time_from_start.sec = int(duration)
        target_point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        trajectory_msg.points = [start_point, target_point]
        self.trajectory_pub.publish(trajectory_msg)
        self.get_logger().info(f"Trajectory published to {TRAJECTORY_TOPIC}")

        end_time = time.time() + duration + 1.0
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def call_trigger_service(self, service_name: str, timeout_sec: float = 5.0) -> None:
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service {service_name} not available")

        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done() or future.result() is None:
            raise RuntimeError(f"Failed to call service {service_name}")

        result = future.result()
        self.get_logger().info(
            f"Service {service_name} response: success={result.success}, message='{result.message}'"
        )


def main() -> None:
    rclpy.init()
    node = RobotControl()

    try:
        target_joint_positions = np.deg2rad(
            [36.10, -75.63, 68.76, -84.23, -88.24, 0.11]
        ).tolist()
        node.move_ur_joint_positions(target_joint_positions, duration=7.0)

        # node.call_trigger_service("/onrobot/open")
        time.sleep(2)

        target_joint_positions = np.deg2rad(
            [-49.57, -92.18, -79.95, -93.72, 89.42, 0.0]
        ).tolist()
        node.move_ur_joint_positions(target_joint_positions, duration=7.0)

        # node.call_trigger_service("/onrobot/close")
        time.sleep(2)

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
