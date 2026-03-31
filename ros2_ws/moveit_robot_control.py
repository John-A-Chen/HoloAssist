#!/usr/bin/env python3
import argparse
import subprocess
import time
from typing import List, Optional

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.srv import GetMotionPlan
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.msg import SafetyMode


JOINT_STATES_TOPIC = "/joint_states"
TRAJECTORY_TOPIC = "/scaled_joint_trajectory_controller/joint_trajectory"
MOVEIT_PLANNING_SERVICE = "/plan_kinematic_path"
MOVEIT_LAUNCH_FILE = "/home/ollie/ros2_ws/ur_moveit.launch.py"
MOVE_GROUP_NAME = "ur_manipulator"
SCALED_CONTROLLER_NAME = "scaled_joint_trajectory_controller"
RUNNING_ROBOT_MODE = RobotMode.RUNNING
SAFE_SAFETY_MODES = {SafetyMode.NORMAL, SafetyMode.REDUCED}
MOVEIT_ERROR_CODES = {
    1: "SUCCESS",
    99999: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -25: "COMMUNICATION_FAILURE",
    -26: "START_STATE_INVALID",
    -27: "GOAL_STATE_INVALID",
    -28: "UNRECOGNIZED_GOAL_TYPE",
    -29: "CRASH",
    -30: "ABORT",
    -31: "NO_IK_SOLUTION",
}
UR_JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class MoveItRobotControl(Node):
    def __init__(self) -> None:
        super().__init__("moveit_robot_control")
        self.current_pos: Optional[List[float]] = None
        self.robot_program_running: Optional[bool] = None
        self.robot_mode: Optional[int] = None
        self.safety_mode: Optional[int] = None
        self.controller_active: Optional[bool] = None

        transient_status_qos = QoSProfile(depth=1)
        transient_status_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.trajectory_pub = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self.joint_state_cb, 10)
        self.create_subscription(
            Bool,
            "/io_and_status_controller/robot_program_running",
            self.robot_program_running_cb,
            transient_status_qos,
        )
        self.create_subscription(
            RobotMode,
            "/io_and_status_controller/robot_mode",
            self.robot_mode_cb,
            transient_status_qos,
        )
        self.create_subscription(
            SafetyMode,
            "/io_and_status_controller/safety_mode",
            self.safety_mode_cb,
            transient_status_qos,
        )

        self.plan_client = self.create_client(GetMotionPlan, MOVEIT_PLANNING_SERVICE)
        self.controller_manager_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switch_controller_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

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

    def robot_program_running_cb(self, msg: Bool) -> None:
        self.robot_program_running = msg.data

    def robot_mode_cb(self, msg: RobotMode) -> None:
        self.robot_mode = msg.mode

    def safety_mode_cb(self, msg: SafetyMode) -> None:
        self.safety_mode = msg.mode

    def wait_for_joint_state(self, timeout_sec: float = 5.0) -> List[float]:
        self.current_pos = None
        deadline = time.time() + timeout_sec
        self.get_logger().info("Waiting for current joint state...")

        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_pos is not None:
                return list(self.current_pos)

        raise RuntimeError(f"No joint state received from {JOINT_STATES_TOPIC}")

    def wait_for_moveit(self, timeout_sec: float = 20.0) -> None:
        self.get_logger().info("Waiting for MoveIt planning service...")
        if not self.plan_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"MoveIt planning service {MOVEIT_PLANNING_SERVICE} is not available")

    def refresh_controller_status(self, timeout_sec: float = 5.0) -> None:
        if not self.controller_manager_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/controller_manager/list_controllers is not available")

        future = self.controller_manager_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None:
            raise RuntimeError("Failed to query controller_manager")

        self.controller_active = False
        for controller in response.controller:
            if controller.name == SCALED_CONTROLLER_NAME:
                self.controller_active = controller.state == "active"
                return

    def wait_for_robot_status(self, timeout_sec: float = 5.0) -> None:
        deadline = time.time() + timeout_sec
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.robot_program_running is not None
                and self.robot_mode is not None
                and self.safety_mode is not None
            ):
                return
        raise RuntimeError("Timed out waiting for robot status topics")

    def activate_scaled_controller(self, timeout_sec: float = 5.0) -> None:
        if not self.switch_controller_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/controller_manager/switch_controller is not available")

        request = SwitchController.Request()
        request.activate_controllers = [SCALED_CONTROLLER_NAME]
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = int(timeout_sec)
        request.timeout.nanosec = 0

        future = self.switch_controller_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec + 2.0)
        response = future.result()
        if response is None or not response.ok:
            raise RuntimeError(
                "Failed to activate scaled_joint_trajectory_controller through controller_manager"
            )

    def assert_robot_ready(self) -> None:
        self.refresh_controller_status()
        self.wait_for_robot_status()

        if not self.controller_active:
            self.get_logger().info(
                "scaled_joint_trajectory_controller is inactive, trying to activate it"
            )
            self.activate_scaled_controller()
            self.refresh_controller_status()
            if not self.controller_active:
                raise RuntimeError("scaled_joint_trajectory_controller is not active")
        if self.robot_program_running is not True:
            raise RuntimeError("External Control program is not running")
        if self.robot_mode != RUNNING_ROBOT_MODE:
            raise RuntimeError(f"robot_mode is {self.robot_mode}, expected RUNNING")
        if self.safety_mode not in SAFE_SAFETY_MODES:
            raise RuntimeError(
                f"safety_mode is {self.safety_mode}, expected NORMAL or REDUCED"
            )

    def build_joint_goal_constraints(
        self, joint_positions: List[float], tolerance: float = 0.001
    ) -> Constraints:
        if len(joint_positions) != len(UR_JOINT_ORDER):
            raise ValueError("Expected exactly 6 joint positions for the UR robot")

        constraints = Constraints()
        constraints.name = "ur_joint_goal"

        for joint_name, joint_position in zip(UR_JOINT_ORDER, joint_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(joint_position)
            joint_constraint.tolerance_above = tolerance
            joint_constraint.tolerance_below = tolerance
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        return constraints

    def moveit_error_name(self, code: int) -> str:
        return MOVEIT_ERROR_CODES.get(code, f"UNKNOWN_ERROR_{code}")

    def plan_joint_motion(
        self,
        joint_positions: List[float],
        allowed_planning_time: float = 5.0,
        velocity_scale: float = 0.2,
        acceleration_scale: float = 0.2,
    ) -> JointTrajectory:
        current_pos = self.wait_for_joint_state()
        self.get_logger().info(f"Current joint positions: {current_pos}")

        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = MOVE_GROUP_NAME
        request.motion_plan_request.pipeline_id = ""
        request.motion_plan_request.planner_id = ""
        request.motion_plan_request.num_planning_attempts = 5
        request.motion_plan_request.allowed_planning_time = allowed_planning_time
        request.motion_plan_request.max_velocity_scaling_factor = velocity_scale
        request.motion_plan_request.max_acceleration_scaling_factor = acceleration_scale
        request.motion_plan_request.goal_constraints = [
            self.build_joint_goal_constraints(joint_positions)
        ]
        request.motion_plan_request.start_state.joint_state.header.stamp = (
            self.get_clock().now().to_msg()
        )
        request.motion_plan_request.start_state.joint_state.name = list(UR_JOINT_ORDER)
        request.motion_plan_request.start_state.joint_state.position = list(current_pos)
        request.motion_plan_request.start_state.is_diff = False

        self.get_logger().info(f"Requesting MoveIt plan for group {MOVE_GROUP_NAME}")
        future = self.plan_client.call_async(request)
        completed = rclpy.spin_until_future_complete(
            self, future, timeout_sec=allowed_planning_time + 10.0
        )

        if not future.done():
            raise RuntimeError(
                "MoveIt planning service timed out before returning a response"
            )

        if future.exception() is not None:
            raise RuntimeError(
                f"MoveIt planning service raised an exception: {future.exception()}"
            )

        response = future.result()

        if response is None:
            raise RuntimeError("MoveIt planning service did not return a response")

        error_code = response.motion_plan_response.error_code.val
        error_name = self.moveit_error_name(error_code)
        if error_code != 1:
            raise RuntimeError(f"MoveIt planning failed: {error_name} ({error_code})")

        trajectory = response.motion_plan_response.trajectory.joint_trajectory
        if not trajectory.points:
            raise RuntimeError("MoveIt returned an empty joint trajectory")

        self.get_logger().info(
            f"MoveIt planning succeeded in {response.motion_plan_response.planning_time:.3f} s "
            f"with {len(trajectory.points)} trajectory points"
        )
        return trajectory

    def execute_trajectory(self, trajectory: JointTrajectory) -> None:
        self.assert_robot_ready()
        self.trajectory_pub.publish(trajectory)

        final_point = trajectory.points[-1]
        duration_sec = float(final_point.time_from_start.sec) + (
            float(final_point.time_from_start.nanosec) / 1e9
        )
        self.get_logger().info(
            f"Published planned trajectory to {TRAJECTORY_TOPIC}; waiting {duration_sec:.2f} s"
        )

        deadline = time.time() + duration_sec + 2.0
        target_positions = list(final_point.positions)
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_pos is None:
                continue
            max_error = max(
                abs(current - target)
                for current, target in zip(self.current_pos, target_positions)
            )
            if max_error < 0.02:
                self.get_logger().info(
                    f"Robot reached target with max joint error {max_error:.4f} rad"
                )
                return

        if self.current_pos is None:
            raise RuntimeError("Lost joint state feedback during execution")

        max_error = max(
            abs(current - target)
            for current, target in zip(self.current_pos, target_positions)
        )
        raise RuntimeError(
            f"Trajectory execution did not settle within tolerance; max joint error is {max_error:.4f} rad"
        )

    def move_to_joint_positions(
        self,
        joint_positions: List[float],
        allowed_planning_time: float = 5.0,
        velocity_scale: float = 0.2,
        acceleration_scale: float = 0.2,
    ) -> None:
        trajectory = self.plan_joint_motion(
            joint_positions,
            allowed_planning_time=allowed_planning_time,
            velocity_scale=velocity_scale,
            acceleration_scale=acceleration_scale,
        )
        self.execute_trajectory(trajectory)


def maybe_launch_moveit(ur_type: str, launch_rviz: bool) -> subprocess.Popen:
    command = [
        "ros2",
        "launch",
        MOVEIT_LAUNCH_FILE,
        f"ur_type:={ur_type}",
        f"launch_rviz:={'true' if launch_rviz else 'false'}",
        "launch_servo:=false",
    ]
    return subprocess.Popen(command)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Launch MoveIt optionally and move a UR robot using MoveIt planning in code."
    )
    parser.add_argument(
        "--launch-moveit",
        action="store_true",
        help="Launch /home/ollie/ros2_ws/ur_moveit.launch.py before sending goals.",
    )
    parser.add_argument(
        "--ur-type",
        default="ur3e",
        help="UR type passed to ur_moveit.launch.py when --launch-moveit is used.",
    )
    parser.add_argument(
        "--launch-rviz",
        action="store_true",
        help="Launch RViz together with MoveIt when --launch-moveit is used.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    moveit_process = None

    if args.launch_moveit:
        moveit_process = maybe_launch_moveit(args.ur_type, args.launch_rviz)
        time.sleep(8.0)

    rclpy.init()
    node = MoveItRobotControl()

    try:
        node.wait_for_moveit()

        first_goal = np.deg2rad([36.10, -75.63, 68.76, -84.23, -88.24, 0.11]).tolist()
        second_goal = np.deg2rad([-49.57, -92.18, -79.95, -93.72, 89.42, 0.0]).tolist()

        node.move_to_joint_positions(first_goal)
        time.sleep(2.0)
        node.move_to_joint_positions(second_goal)

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

        if moveit_process is not None:
            moveit_process.terminate()
            moveit_process.wait(timeout=5)


if __name__ == "__main__":
    main()
