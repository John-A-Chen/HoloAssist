#!/usr/bin/env python3
import argparse
import json
import math
import subprocess
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, List, Optional

import numpy as np
import rclpy
from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from moveit_msgs.msg import BoundingVolume
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import OrientationConstraint
from moveit_msgs.msg import PlanningScene
from moveit_msgs.msg import PositionConstraint
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
from trajectory_msgs.msg import JointTrajectory
from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.msg import SafetyMode


JOINT_STATES_TOPIC = "/joint_states"
TRAJECTORY_TOPIC = "/scaled_joint_trajectory_controller/joint_trajectory"
MOVEIT_PLANNING_SERVICE = "/plan_kinematic_path"
MOVEIT_CARTESIAN_PATH_SERVICE = "/compute_cartesian_path"
STATE_VALIDITY_SERVICE = "/check_state_validity"
APPLY_PLANNING_SCENE_SERVICE = "/apply_planning_scene"
MOVE_GROUP_NAME = "ur_manipulator"
BASE_FRAME = "base_link"
END_EFFECTOR_LINK = "tool0"
ROUTE_CANDIDATES = 5
MOVEIT_PLANNING_ATTEMPTS = 1
COLLISION_CHECK_STRIDE = 1
CARTESIAN_MAX_STEP = 0.005
CARTESIAN_AUTO_WAYPOINT_DISTANCE = 0.10
CARTESIAN_JUMP_THRESHOLD = 5.0
CARTESIAN_MIN_FRACTION = 0.999
POSE_GOAL_POSITION_TOLERANCE = 0.005
POSE_GOAL_ORIENTATION_TOLERANCE = 0.05
POSE_GOAL_PLANNING_TIME = 5.0
FLOOR_OBJECT_ID = "workspace_floor"
FLOOR_SIZE_X = 4.0
FLOOR_SIZE_Y = 4.0
FLOOR_THICKNESS = 0.02
FLOOR_CLEARANCE = 0.001
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


def default_moveit_launch_file() -> str:
    try:
        package_launch = (
            Path(get_package_share_directory("moveit_robot_control"))
            / "launch"
            / "ur_moveit.launch.py"
        )
        if package_launch.exists():
            return str(package_launch)
    except PackageNotFoundError:
        pass

    source_launch = (
        Path(__file__).resolve().parents[1] / "launch" / "ur_moveit.launch.py"
    )
    if source_launch.exists():
        return str(source_launch)

    workspace_launch = Path.cwd() / "ur_moveit.launch.py"
    if workspace_launch.exists():
        return str(workspace_launch)

    return str(source_launch)


MOVEIT_LAUNCH_FILE = default_moveit_launch_file()


@dataclass(frozen=True)
class PlanScore:
    trajectory: JointTrajectory
    path_length: float
    duration_sec: float
    checked_states: int


def trajectory_duration_sec(trajectory: JointTrajectory) -> float:
    if not trajectory.points:
        return 0.0
    final_point = trajectory.points[-1]
    return float(final_point.time_from_start.sec) + (
        float(final_point.time_from_start.nanosec) / 1e9
    )


def set_duration_seconds(duration, seconds: float) -> None:
    duration.sec = int(seconds)
    duration.nanosec = int(round((seconds - duration.sec) * 1e9))
    if duration.nanosec >= 1_000_000_000:
        duration.sec += 1
        duration.nanosec -= 1_000_000_000


def trajectory_has_strict_timing(trajectory: JointTrajectory) -> bool:
    if not trajectory.points:
        return False

    previous_time = -1.0
    for point in trajectory.points:
        point_time = float(point.time_from_start.sec) + (
            float(point.time_from_start.nanosec) / 1e9
        )
        if point_time <= previous_time:
            return False
        previous_time = point_time

    return previous_time > 0.0


def retime_trajectory_if_needed(
    trajectory: JointTrajectory,
    start_positions: List[float],
    velocity_scale: float,
    minimum_segment_duration: float = 0.05,
) -> None:
    if trajectory_has_strict_timing(trajectory):
        return

    start_by_joint = {
        joint_name: joint_position
        for joint_name, joint_position in zip(UR_JOINT_ORDER, start_positions)
    }
    previous_positions = [start_by_joint[name] for name in trajectory.joint_names]

    scale = max(0.01, min(1.0, velocity_scale))
    max_joint_speed = math.pi * scale
    elapsed = 0.0

    for point in trajectory.points:
        current_positions = list(point.positions)
        max_joint_delta = max(
            abs(current - previous)
            for current, previous in zip(current_positions, previous_positions)
        )
        segment_duration = max(
            minimum_segment_duration, max_joint_delta / max_joint_speed
        )
        elapsed += segment_duration
        set_duration_seconds(point.time_from_start, elapsed)
        point.velocities = [
            (current - previous) / segment_duration
            for current, previous in zip(current_positions, previous_positions)
        ]
        previous_positions = current_positions


def joint_space_path_length(
    trajectory: JointTrajectory,
    start_positions: List[float],
) -> float:
    if not trajectory.points:
        return 0.0

    start_by_joint = {
        joint_name: joint_position
        for joint_name, joint_position in zip(UR_JOINT_ORDER, start_positions)
    }
    previous_positions = [start_by_joint[name] for name in trajectory.joint_names]

    path_length = 0.0
    for point in trajectory.points:
        if len(point.positions) != len(trajectory.joint_names):
            raise RuntimeError("MoveIt returned a trajectory point with the wrong joint count")

        current_positions = list(point.positions)
        squared_step = sum(
            (current - previous) ** 2
            for current, previous in zip(current_positions, previous_positions)
        )
        path_length += float(np.sqrt(squared_step))
        previous_positions = current_positions

    return path_length


def joint_positions_for_order(
    joint_names: List[str],
    positions: List[float],
    target_order: List[str],
) -> List[float]:
    if len(joint_names) != len(positions):
        raise RuntimeError("Joint name and position counts do not match")

    positions_by_joint = {
        joint_name: joint_position
        for joint_name, joint_position in zip(joint_names, positions)
    }
    missing_joints = [
        joint_name for joint_name in target_order if joint_name not in positions_by_joint
    ]
    if missing_joints:
        raise RuntimeError(
            "Trajectory is missing expected joint(s): " + ", ".join(missing_joints)
        )

    return [positions_by_joint[joint_name] for joint_name in target_order]


def normalize_quaternion(quat: Quaternion) -> Quaternion:
    length = math.sqrt(quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2)
    if length == 0.0:
        raise ValueError("Quaternion length cannot be zero")

    normalized = Quaternion()
    normalized.x = quat.x / length
    normalized.y = quat.y / length
    normalized.z = quat.z / length
    normalized.w = quat.w / length
    return normalized


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    quat = Quaternion()
    quat.w = cr * cp * cy + sr * sp * sy
    quat.x = sr * cp * cy - cr * sp * sy
    quat.y = cr * sp * cy + sr * cp * sy
    quat.z = cr * cp * sy - sr * sp * cy
    return normalize_quaternion(quat)


def make_pose(
    x: float,
    y: float,
    z: float,
    orientation: Quaternion,
) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation = orientation
    return pose


def identity_pose() -> Pose:
    pose = Pose()
    pose.orientation.w = 1.0
    return pose


def copy_pose(pose: Pose) -> Pose:
    copied = Pose()
    copied.position.x = pose.position.x
    copied.position.y = pose.position.y
    copied.position.z = pose.position.z
    copied.orientation.x = pose.orientation.x
    copied.orientation.y = pose.orientation.y
    copied.orientation.z = pose.orientation.z
    copied.orientation.w = pose.orientation.w
    return copied


def cartesian_distance(first: Pose, second: Pose) -> float:
    dx = second.position.x - first.position.x
    dy = second.position.y - first.position.y
    dz = second.position.z - first.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def cartesian_path_length(start_pose: Pose, waypoints: List[Pose]) -> float:
    previous_pose = start_pose
    path_length = 0.0
    for waypoint in waypoints:
        path_length += cartesian_distance(previous_pose, waypoint)
        previous_pose = waypoint
    return path_length


def densify_cartesian_waypoints(
    start_pose: Pose,
    coordinates: List[tuple[float, float, float]],
    orientation: Quaternion,
    max_segment_distance: float,
) -> List[Pose]:
    if max_segment_distance <= 0.0:
        return [make_pose(x, y, z, orientation) for x, y, z in coordinates]

    dense_waypoints: List[Pose] = []
    previous_pose = start_pose

    for x, y, z in coordinates:
        target_pose = make_pose(x, y, z, orientation)
        distance = cartesian_distance(previous_pose, target_pose)
        segment_count = max(1, math.ceil(distance / max_segment_distance))

        for segment_index in range(1, segment_count + 1):
            t = segment_index / segment_count
            dense_waypoints.append(
                make_pose(
                    previous_pose.position.x
                    + (target_pose.position.x - previous_pose.position.x) * t,
                    previous_pose.position.y
                    + (target_pose.position.y - previous_pose.position.y) * t,
                    previous_pose.position.z
                    + (target_pose.position.z - previous_pose.position.z) * t,
                    orientation,
                )
            )

        previous_pose = target_pose

    return dense_waypoints


def parse_coordinate_triples(values: List[float]) -> List[tuple[float, float, float]]:
    if not values:
        return []
    if len(values) % 3 != 0:
        raise ValueError("Cartesian coordinates must be supplied as x y z triples")

    return [
        (values[index], values[index + 1], values[index + 2])
        for index in range(0, len(values), 3)
    ]


def format_contacts(contacts, max_contacts: int = 3) -> str:
    if not contacts:
        return ""

    pairs = [
        f"{contact.contact_body_1}<->{contact.contact_body_2}"
        for contact in contacts[:max_contacts]
    ]
    if len(contacts) > max_contacts:
        pairs.append(f"+{len(contacts) - max_contacts} more")

    return "; contacts: " + ", ".join(pairs)


class MoveItRobotControl(Node):
    def __init__(self) -> None:
        super().__init__("moveit_robot_control")
        self.current_pos: Optional[List[float]] = None
        self.robot_program_running: Optional[bool] = None
        self.robot_mode: Optional[int] = None
        self.safety_mode: Optional[int] = None
        self.controller_active: Optional[bool] = None
        self.floor_collision_object_applied = False
        self.floor_collision_object_frame: Optional[str] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        self.cartesian_path_client = self.create_client(
            GetCartesianPath, MOVEIT_CARTESIAN_PATH_SERVICE
        )
        self.state_validity_client = self.create_client(
            GetStateValidity, STATE_VALIDITY_SERVICE
        )
        self.apply_planning_scene_client = self.create_client(
            ApplyPlanningScene, APPLY_PLANNING_SCENE_SERVICE
        )
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
            raise RuntimeError(
                f"MoveIt planning service {MOVEIT_PLANNING_SERVICE} is not available"
            )

    def wait_for_cartesian_path_service(self, timeout_sec: float = 20.0) -> None:
        self.get_logger().info("Waiting for MoveIt Cartesian path service...")
        if not self.cartesian_path_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(
                "MoveIt Cartesian path service "
                f"{MOVEIT_CARTESIAN_PATH_SERVICE} is not available"
            )

    def wait_for_state_validity(self, timeout_sec: float = 10.0) -> None:
        self.get_logger().info("Waiting for MoveIt state validity service...")
        if not self.state_validity_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(
                f"MoveIt state validity service {STATE_VALIDITY_SERVICE} is not available"
            )

    def floor_top_z(self) -> float:
        return -FLOOR_CLEARANCE

    def build_floor_collision_object(self, frame_id: str = BASE_FRAME) -> CollisionObject:
        floor = CollisionObject()
        floor.header.stamp = self.get_clock().now().to_msg()
        floor.header.frame_id = frame_id
        floor.id = FLOOR_OBJECT_ID

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [FLOOR_SIZE_X, FLOOR_SIZE_Y, FLOOR_THICKNESS]

        primitive_pose = identity_pose()
        primitive_pose.position.z = self.floor_top_z() - (FLOOR_THICKNESS / 2.0)

        floor.primitives.append(primitive)
        floor.primitive_poses.append(primitive_pose)
        floor.operation = CollisionObject.ADD
        return floor

    def apply_floor_collision_object(
        self,
        frame_id: str = BASE_FRAME,
        timeout_sec: float = 10.0,
    ) -> None:
        if (
            self.floor_collision_object_applied
            and self.floor_collision_object_frame == frame_id
        ):
            return

        self.get_logger().info(
            f"Adding floor collision object to MoveIt scene in frame {frame_id}"
        )
        if not self.apply_planning_scene_client.wait_for_service(
            timeout_sec=timeout_sec
        ):
            raise RuntimeError(
                "MoveIt apply planning scene service "
                f"{APPLY_PLANNING_SCENE_SERVICE} is not available; cannot add the "
                "floor collision object"
            )

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(
            self.build_floor_collision_object(frame_id)
        )

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            raise RuntimeError(
                "Timed out while applying the floor collision object to MoveIt"
            )
        if future.exception() is not None:
            raise RuntimeError(
                "MoveIt apply planning scene service raised an exception while "
                f"adding the floor collision object: {future.exception()}"
            )

        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(
                "MoveIt rejected the floor collision object; refusing to plan "
                "without floor collision checking"
            )

        self.floor_collision_object_applied = True
        self.floor_collision_object_frame = frame_id
        self.get_logger().info(
            f"Floor collision object active: {FLOOR_SIZE_X:.1f} m x "
            f"{FLOOR_SIZE_Y:.1f} m, top at z={self.floor_top_z():.4f} m"
        )

    def wait_for_current_pose(
        self,
        base_frame: str,
        end_effector_link: str,
        timeout_sec: float = 5.0,
    ) -> Pose:
        deadline = time.time() + timeout_sec
        last_error: Optional[Exception] = None
        self.get_logger().info(
            f"Waiting for TF from {base_frame} to {end_effector_link}..."
        )

        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    base_frame,
                    end_effector_link,
                    Time(),
                )
            except TransformException as exc:
                last_error = exc
                continue

            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose

        raise RuntimeError(
            f"Could not read current pose for {end_effector_link} in "
            f"{base_frame}: {last_error}"
        )

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

    def build_pose_goal_constraints(
        self,
        target_pose: Pose,
        base_frame: str,
        end_effector_link: str,
        position_tolerance: float = POSE_GOAL_POSITION_TOLERANCE,
        orientation_tolerance: float = POSE_GOAL_ORIENTATION_TOLERANCE,
    ) -> Constraints:
        if position_tolerance <= 0.0:
            raise ValueError("pose goal position tolerance must be greater than 0")
        if orientation_tolerance <= 0.0:
            raise ValueError("pose goal orientation tolerance must be greater than 0")

        now = self.get_clock().now().to_msg()
        constraints = Constraints()
        constraints.name = "ur_pose_goal"

        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tolerance]
        region.primitives.append(sphere)

        region_pose = identity_pose()
        region_pose.position.x = target_pose.position.x
        region_pose.position.y = target_pose.position.y
        region_pose.position.z = target_pose.position.z
        region.primitive_poses.append(region_pose)

        position_constraint = PositionConstraint()
        position_constraint.header.stamp = now
        position_constraint.header.frame_id = base_frame
        position_constraint.link_name = end_effector_link
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.stamp = now
        orientation_constraint.header.frame_id = base_frame
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.orientation = normalize_quaternion(
            target_pose.orientation
        )
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance
        orientation_constraint.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def moveit_error_name(self, code: int) -> str:
        return MOVEIT_ERROR_CODES.get(code, f"UNKNOWN_ERROR_{code}")

    def request_joint_motion_plan(
        self,
        joint_positions: List[float],
        current_pos: List[float],
        allowed_planning_time: float = 5.0,
        velocity_scale: float = 0.2,
        acceleration_scale: float = 0.2,
        planning_attempts: int = MOVEIT_PLANNING_ATTEMPTS,
        candidate_number: Optional[int] = None,
        candidate_count: Optional[int] = None,
    ) -> JointTrajectory:
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = MOVE_GROUP_NAME
        request.motion_plan_request.pipeline_id = ""
        request.motion_plan_request.planner_id = ""
        request.motion_plan_request.num_planning_attempts = planning_attempts
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

        candidate_text = ""
        if candidate_number is not None and candidate_count is not None:
            candidate_text = f" candidate {candidate_number}/{candidate_count}"
        self.get_logger().info(
            f"Requesting MoveIt plan{candidate_text} for group {MOVE_GROUP_NAME}"
        )
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(
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

    def request_pose_motion_plan(
        self,
        target_pose: Pose,
        current_pos: List[float],
        base_frame: str = BASE_FRAME,
        end_effector_link: str = END_EFFECTOR_LINK,
        allowed_planning_time: float = POSE_GOAL_PLANNING_TIME,
        velocity_scale: float = 0.2,
        acceleration_scale: float = 0.2,
        planning_attempts: int = MOVEIT_PLANNING_ATTEMPTS,
        position_tolerance: float = POSE_GOAL_POSITION_TOLERANCE,
        orientation_tolerance: float = POSE_GOAL_ORIENTATION_TOLERANCE,
        candidate_number: Optional[int] = None,
        candidate_count: Optional[int] = None,
    ) -> JointTrajectory:
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = MOVE_GROUP_NAME
        request.motion_plan_request.pipeline_id = ""
        request.motion_plan_request.planner_id = ""
        request.motion_plan_request.num_planning_attempts = planning_attempts
        request.motion_plan_request.allowed_planning_time = allowed_planning_time
        request.motion_plan_request.max_velocity_scaling_factor = velocity_scale
        request.motion_plan_request.max_acceleration_scaling_factor = acceleration_scale
        request.motion_plan_request.workspace_parameters.header.frame_id = base_frame
        request.motion_plan_request.workspace_parameters.min_corner.x = -FLOOR_SIZE_X / 2.0
        request.motion_plan_request.workspace_parameters.min_corner.y = -FLOOR_SIZE_Y / 2.0
        request.motion_plan_request.workspace_parameters.min_corner.z = self.floor_top_z()
        request.motion_plan_request.workspace_parameters.max_corner.x = FLOOR_SIZE_X / 2.0
        request.motion_plan_request.workspace_parameters.max_corner.y = FLOOR_SIZE_Y / 2.0
        request.motion_plan_request.workspace_parameters.max_corner.z = FLOOR_SIZE_X / 2.0
        request.motion_plan_request.goal_constraints = [
            self.build_pose_goal_constraints(
                target_pose,
                base_frame,
                end_effector_link,
                position_tolerance=position_tolerance,
                orientation_tolerance=orientation_tolerance,
            )
        ]
        request.motion_plan_request.start_state.joint_state.header.stamp = (
            self.get_clock().now().to_msg()
        )
        request.motion_plan_request.start_state.joint_state.name = list(UR_JOINT_ORDER)
        request.motion_plan_request.start_state.joint_state.position = list(current_pos)
        request.motion_plan_request.start_state.is_diff = False

        candidate_text = ""
        if candidate_number is not None and candidate_count is not None:
            candidate_text = f" candidate {candidate_number}/{candidate_count}"
        self.get_logger().info(
            f"Requesting MoveIt pose-goal plan{candidate_text} for "
            f"{end_effector_link} in {base_frame}: "
            f"x={target_pose.position.x:.4f}, y={target_pose.position.y:.4f}, "
            f"z={target_pose.position.z:.4f}"
        )
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=allowed_planning_time + 10.0
        )

        if not future.done():
            raise RuntimeError(
                "MoveIt pose-goal planning service timed out before returning a response"
            )

        if future.exception() is not None:
            raise RuntimeError(
                "MoveIt pose-goal planning service raised an exception: "
                f"{future.exception()}"
            )

        response = future.result()
        if response is None:
            raise RuntimeError("MoveIt pose-goal planning service did not return a response")

        error_code = response.motion_plan_response.error_code.val
        error_name = self.moveit_error_name(error_code)
        if error_code != 1:
            raise RuntimeError(
                f"MoveIt pose-goal planning failed: {error_name} ({error_code})"
            )

        trajectory = response.motion_plan_response.trajectory.joint_trajectory
        if not trajectory.points:
            raise RuntimeError("MoveIt returned an empty pose-goal trajectory")

        self.get_logger().info(
            "MoveIt pose-goal planning succeeded in "
            f"{response.motion_plan_response.planning_time:.3f} s with "
            f"{len(trajectory.points)} trajectory points"
        )
        return trajectory

    def validate_trajectory_collision_free(
        self,
        trajectory: JointTrajectory,
        stride: int = COLLISION_CHECK_STRIDE,
        timeout_sec: float = 2.0,
    ) -> int:
        if stride < 1:
            raise ValueError("Collision check stride must be at least 1")

        point_count = len(trajectory.points)
        if point_count == 0:
            raise RuntimeError("Cannot validate an empty trajectory")

        indexes = list(range(0, point_count, stride))
        if indexes[-1] != point_count - 1:
            indexes.append(point_count - 1)

        checked_states = 0
        for point_index in indexes:
            point = trajectory.points[point_index]
            request = GetStateValidity.Request()
            request.group_name = MOVE_GROUP_NAME
            request.robot_state.joint_state.header.stamp = (
                self.get_clock().now().to_msg()
            )
            request.robot_state.joint_state.name = list(trajectory.joint_names)
            request.robot_state.joint_state.position = list(point.positions)
            request.robot_state.is_diff = False

            future = self.state_validity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if not future.done():
                raise RuntimeError(
                    "MoveIt state validity service timed out while checking "
                    f"trajectory point {point_index + 1}/{point_count}"
                )

            if future.exception() is not None:
                raise RuntimeError(
                    "MoveIt state validity service raised an exception while checking "
                    f"trajectory point {point_index + 1}/{point_count}: "
                    f"{future.exception()}"
                )

            response = future.result()
            if response is None:
                raise RuntimeError(
                    "MoveIt state validity service did not return a response while "
                    f"checking trajectory point {point_index + 1}/{point_count}"
                )

            if not response.valid:
                raise RuntimeError(
                    f"trajectory point {point_index + 1}/{point_count} is invalid"
                    f"{format_contacts(response.contacts)}"
                )

            checked_states += 1

        return checked_states

    def plan_joint_motion(
        self,
        joint_positions: List[float],
        allowed_planning_time: float = 5.0,
        velocity_scale: float = 0.2,
        acceleration_scale: float = 0.2,
        route_candidates: int = ROUTE_CANDIDATES,
        moveit_planning_attempts: int = MOVEIT_PLANNING_ATTEMPTS,
        collision_check_stride: int = COLLISION_CHECK_STRIDE,
    ) -> JointTrajectory:
        if route_candidates < 1:
            raise ValueError("route_candidates must be at least 1")
        if moveit_planning_attempts < 1:
            raise ValueError("moveit_planning_attempts must be at least 1")
        if collision_check_stride < 1:
            raise ValueError("collision_check_stride must be at least 1")

        current_pos = self.wait_for_joint_state()
        self.get_logger().info(f"Current joint positions: {current_pos}")
        self.wait_for_moveit()
        self.wait_for_state_validity()
        self.apply_floor_collision_object(BASE_FRAME)

        best_score: Optional[PlanScore] = None
        successful_candidates = 0
        failed_candidates = 0

        for candidate_index in range(1, route_candidates + 1):
            try:
                trajectory = self.request_joint_motion_plan(
                    joint_positions,
                    current_pos,
                    allowed_planning_time=allowed_planning_time,
                    velocity_scale=velocity_scale,
                    acceleration_scale=acceleration_scale,
                    planning_attempts=moveit_planning_attempts,
                    candidate_number=candidate_index,
                    candidate_count=route_candidates,
                )
                checked_states = self.validate_trajectory_collision_free(
                    trajectory, stride=collision_check_stride
                )
                path_length = joint_space_path_length(trajectory, current_pos)
                duration_sec = trajectory_duration_sec(trajectory)
            except RuntimeError as exc:
                failed_candidates += 1
                self.get_logger().warning(
                    f"Candidate {candidate_index}/{route_candidates} failed: {exc}"
                )
                continue

            successful_candidates += 1
            self.get_logger().info(
                f"Candidate {candidate_index}/{route_candidates}: "
                f"joint path length {path_length:.3f} rad, duration "
                f"{duration_sec:.2f} s, checked {checked_states} collision-free state(s)"
            )

            route_score = PlanScore(
                trajectory=trajectory,
                path_length=path_length,
                duration_sec=duration_sec,
                checked_states=checked_states,
            )
            if best_score is None or route_score.path_length < best_score.path_length:
                best_score = route_score

        if best_score is None:
            raise RuntimeError(
                f"MoveIt could not produce a floor-collision-free trajectory after "
                f"{route_candidates} candidate route(s); {failed_candidates} "
                "candidate(s) failed. The location is invalid for the current "
                "scene; send a different location."
            )

        self.get_logger().info(
            f"Selected shortest floor-collision-free route from "
            f"{successful_candidates} successful candidate(s): joint path length "
            f"{best_score.path_length:.3f} rad, duration {best_score.duration_sec:.2f} s"
        )
        return best_score.trajectory

    def plan_pose_motion(
        self,
        target_pose: Pose,
        current_pos: Optional[List[float]] = None,
        base_frame: str = BASE_FRAME,
        end_effector_link: str = END_EFFECTOR_LINK,
        allowed_planning_time: float = POSE_GOAL_PLANNING_TIME,
        velocity_scale: float = 0.2,
        acceleration_scale: float = 0.2,
        route_candidates: int = ROUTE_CANDIDATES,
        moveit_planning_attempts: int = MOVEIT_PLANNING_ATTEMPTS,
        collision_check_stride: int = COLLISION_CHECK_STRIDE,
        position_tolerance: float = POSE_GOAL_POSITION_TOLERANCE,
        orientation_tolerance: float = POSE_GOAL_ORIENTATION_TOLERANCE,
    ) -> JointTrajectory:
        if route_candidates < 1:
            raise ValueError("route_candidates must be at least 1")
        if moveit_planning_attempts < 1:
            raise ValueError("moveit_planning_attempts must be at least 1")
        if collision_check_stride < 1:
            raise ValueError("collision_check_stride must be at least 1")

        if current_pos is None:
            current_pos = self.wait_for_joint_state()
            self.get_logger().info(f"Current joint positions: {current_pos}")

        self.wait_for_moveit()
        self.wait_for_state_validity()
        self.apply_floor_collision_object(base_frame)

        best_score: Optional[PlanScore] = None
        successful_candidates = 0
        failed_candidates = 0

        for candidate_index in range(1, route_candidates + 1):
            try:
                trajectory = self.request_pose_motion_plan(
                    target_pose,
                    current_pos,
                    base_frame=base_frame,
                    end_effector_link=end_effector_link,
                    allowed_planning_time=allowed_planning_time,
                    velocity_scale=velocity_scale,
                    acceleration_scale=acceleration_scale,
                    planning_attempts=moveit_planning_attempts,
                    position_tolerance=position_tolerance,
                    orientation_tolerance=orientation_tolerance,
                    candidate_number=candidate_index,
                    candidate_count=route_candidates,
                )
                checked_states = self.validate_trajectory_collision_free(
                    trajectory, stride=collision_check_stride
                )
                path_length = joint_space_path_length(trajectory, current_pos)
                duration_sec = trajectory_duration_sec(trajectory)
            except RuntimeError as exc:
                failed_candidates += 1
                self.get_logger().warning(
                    f"Pose-goal candidate {candidate_index}/{route_candidates} "
                    f"failed: {exc}"
                )
                continue

            successful_candidates += 1
            self.get_logger().info(
                f"Pose-goal candidate {candidate_index}/{route_candidates}: "
                f"joint path length {path_length:.3f} rad, duration "
                f"{duration_sec:.2f} s, checked {checked_states} "
                "collision-free state(s)"
            )

            route_score = PlanScore(
                trajectory=trajectory,
                path_length=path_length,
                duration_sec=duration_sec,
                checked_states=checked_states,
            )
            if best_score is None or route_score.path_length < best_score.path_length:
                best_score = route_score

        if best_score is None:
            raise RuntimeError(
                "MoveIt could not produce a floor-collision-free route to the "
                f"requested pose after {route_candidates} candidate route(s); "
                f"{failed_candidates} candidate(s) failed. The location is "
                "invalid for the current scene; send a different location."
            )

        self.get_logger().info(
            "Selected shortest floor-collision-free pose-goal route from "
            f"{successful_candidates} successful candidate(s): joint path length "
            f"{best_score.path_length:.3f} rad, duration {best_score.duration_sec:.2f} s"
        )
        return best_score.trajectory

    def plan_cartesian_motion(
        self,
        coordinates: List[tuple[float, float, float]],
        base_frame: str = BASE_FRAME,
        end_effector_link: str = END_EFFECTOR_LINK,
        orientation: Optional[Quaternion] = None,
        max_step: float = CARTESIAN_MAX_STEP,
        auto_waypoint_distance: float = CARTESIAN_AUTO_WAYPOINT_DISTANCE,
        jump_threshold: float = CARTESIAN_JUMP_THRESHOLD,
        min_fraction: float = CARTESIAN_MIN_FRACTION,
        velocity_scale: float = 0.2,
        collision_check_stride: int = COLLISION_CHECK_STRIDE,
        allow_pose_goal_fallback: bool = True,
        pose_goal_allowed_planning_time: float = POSE_GOAL_PLANNING_TIME,
        pose_goal_position_tolerance: float = POSE_GOAL_POSITION_TOLERANCE,
        pose_goal_orientation_tolerance: float = POSE_GOAL_ORIENTATION_TOLERANCE,
        pose_goal_route_candidates: int = ROUTE_CANDIDATES,
        pose_goal_planning_attempts: int = MOVEIT_PLANNING_ATTEMPTS,
    ) -> JointTrajectory:
        if not coordinates:
            raise ValueError("At least one Cartesian coordinate is required")
        if max_step <= 0.0:
            raise ValueError("Cartesian max step must be greater than 0")
        if auto_waypoint_distance < 0.0:
            raise ValueError("Automatic waypoint distance must be greater than or equal to 0")
        if min_fraction <= 0.0 or min_fraction > 1.0:
            raise ValueError("Minimum Cartesian fraction must be in the range (0, 1]")
        if collision_check_stride < 1:
            raise ValueError("collision_check_stride must be at least 1")

        current_pos = self.wait_for_joint_state()
        self.get_logger().info(f"Current joint positions: {current_pos}")
        current_pose = self.wait_for_current_pose(base_frame, end_effector_link)
        self.wait_for_moveit()
        self.wait_for_cartesian_path_service()
        self.wait_for_state_validity()
        self.apply_floor_collision_object(base_frame)

        waypoint_orientation = orientation or current_pose.orientation
        waypoints = densify_cartesian_waypoints(
            current_pose,
            coordinates,
            waypoint_orientation,
            auto_waypoint_distance,
        )
        requested_length = cartesian_path_length(current_pose, waypoints)
        target_pose = copy_pose(waypoints[-1])

        def pose_goal_fallback(reason: str) -> JointTrajectory:
            if not allow_pose_goal_fallback:
                raise RuntimeError(reason)
            if len(coordinates) != 1:
                raise RuntimeError(
                    f"{reason}. Alternate pose-goal planning is only available "
                    "for single target points; send a different waypoint set."
                )

            self.get_logger().warning(
                "Straight-line Cartesian path is blocked or invalid: "
                f"{reason}. Trying general MoveIt pose-goal planning so the "
                "planner can choose another floor-collision-free route."
            )
            try:
                return self.plan_pose_motion(
                    target_pose,
                    current_pos=current_pos,
                    base_frame=base_frame,
                    end_effector_link=end_effector_link,
                    allowed_planning_time=pose_goal_allowed_planning_time,
                    velocity_scale=velocity_scale,
                    acceleration_scale=velocity_scale,
                    route_candidates=pose_goal_route_candidates,
                    moveit_planning_attempts=pose_goal_planning_attempts,
                    collision_check_stride=collision_check_stride,
                    position_tolerance=pose_goal_position_tolerance,
                    orientation_tolerance=pose_goal_orientation_tolerance,
                )
            except RuntimeError as exc:
                raise RuntimeError(
                    "The requested location is invalid for the current floor "
                    f"collision scene. Straight-line planning failed: {reason}. "
                    f"Alternate route planning also failed: {exc}"
                ) from exc

        request = GetCartesianPath.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = base_frame
        request.start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        request.start_state.joint_state.name = list(UR_JOINT_ORDER)
        request.start_state.joint_state.position = list(current_pos)
        request.start_state.is_diff = False
        request.group_name = MOVE_GROUP_NAME
        request.link_name = end_effector_link
        request.waypoints = waypoints
        request.max_step = max_step
        request.jump_threshold = jump_threshold
        request.avoid_collisions = True

        self.get_logger().info(
            f"Requesting Cartesian path through {len(waypoints)} waypoint(s) "
            f"expanded from {len(coordinates)} target coordinate(s), straight-line "
            f"length {requested_length:.3f} m, max step {max_step:.3f} m"
        )
        future = self.cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)

        if not future.done():
            return pose_goal_fallback(
                "MoveIt Cartesian path service timed out before returning a response"
            )

        if future.exception() is not None:
            return pose_goal_fallback(
                "MoveIt Cartesian path service raised an exception: "
                f"{future.exception()}"
            )

        response = future.result()
        if response is None:
            return pose_goal_fallback(
                "MoveIt Cartesian path service did not return a response"
            )

        error_code = response.error_code.val
        error_name = self.moveit_error_name(error_code)
        if error_code != 1:
            return pose_goal_fallback(
                f"MoveIt Cartesian path failed: {error_name} ({error_code}), "
                f"fraction {response.fraction:.3f}"
            )

        if response.fraction + 1e-9 < min_fraction:
            return pose_goal_fallback(
                f"MoveIt could only compute {response.fraction:.3f} of the "
                f"straight Cartesian path; required at least {min_fraction:.3f}. "
                "Refusing to execute a partial path."
            )

        trajectory = response.solution.joint_trajectory
        if not trajectory.points:
            return pose_goal_fallback("MoveIt returned an empty Cartesian trajectory")

        try:
            checked_states = self.validate_trajectory_collision_free(
                trajectory, stride=collision_check_stride
            )
        except RuntimeError as exc:
            return pose_goal_fallback(str(exc))
        retime_trajectory_if_needed(trajectory, current_pos, velocity_scale)
        duration_sec = trajectory_duration_sec(trajectory)

        self.get_logger().info(
            f"Cartesian path succeeded: fraction {response.fraction:.3f}, "
            f"{len(trajectory.points)} trajectory points, checked "
            f"{checked_states} collision-free state(s), duration {duration_sec:.2f} s"
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
        target_positions = joint_positions_for_order(
            list(trajectory.joint_names), list(final_point.positions), UR_JOINT_ORDER
        )
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
            "Trajectory execution did not settle within tolerance; "
            f"max joint error is {max_error:.4f} rad"
        )

    def move_to_joint_positions(
        self,
        joint_positions: List[float],
        allowed_planning_time: float = 5.0,
        velocity_scale: float = 0.2,
        acceleration_scale: float = 0.2,
        route_candidates: int = ROUTE_CANDIDATES,
        moveit_planning_attempts: int = MOVEIT_PLANNING_ATTEMPTS,
        collision_check_stride: int = COLLISION_CHECK_STRIDE,
        dry_run: bool = False,
    ) -> None:
        trajectory = self.plan_joint_motion(
            joint_positions,
            allowed_planning_time=allowed_planning_time,
            velocity_scale=velocity_scale,
            acceleration_scale=acceleration_scale,
            route_candidates=route_candidates,
            moveit_planning_attempts=moveit_planning_attempts,
            collision_check_stride=collision_check_stride,
        )
        if dry_run:
            self.get_logger().info(
                "Dry run requested; selected trajectory was not executed"
            )
            return

        self.execute_trajectory(trajectory)

    def move_to_cartesian_coordinates(
        self,
        coordinates: List[tuple[float, float, float]],
        base_frame: str = BASE_FRAME,
        end_effector_link: str = END_EFFECTOR_LINK,
        orientation: Optional[Quaternion] = None,
        max_step: float = CARTESIAN_MAX_STEP,
        auto_waypoint_distance: float = CARTESIAN_AUTO_WAYPOINT_DISTANCE,
        jump_threshold: float = CARTESIAN_JUMP_THRESHOLD,
        min_fraction: float = CARTESIAN_MIN_FRACTION,
        velocity_scale: float = 0.2,
        collision_check_stride: int = COLLISION_CHECK_STRIDE,
        allow_pose_goal_fallback: bool = True,
        pose_goal_allowed_planning_time: float = POSE_GOAL_PLANNING_TIME,
        pose_goal_position_tolerance: float = POSE_GOAL_POSITION_TOLERANCE,
        pose_goal_orientation_tolerance: float = POSE_GOAL_ORIENTATION_TOLERANCE,
        pose_goal_route_candidates: int = ROUTE_CANDIDATES,
        pose_goal_planning_attempts: int = MOVEIT_PLANNING_ATTEMPTS,
        dry_run: bool = False,
    ) -> None:
        trajectory = self.plan_cartesian_motion(
            coordinates,
            base_frame=base_frame,
            end_effector_link=end_effector_link,
            orientation=orientation,
            max_step=max_step,
            auto_waypoint_distance=auto_waypoint_distance,
            jump_threshold=jump_threshold,
            min_fraction=min_fraction,
            velocity_scale=velocity_scale,
            collision_check_stride=collision_check_stride,
            allow_pose_goal_fallback=allow_pose_goal_fallback,
            pose_goal_allowed_planning_time=pose_goal_allowed_planning_time,
            pose_goal_position_tolerance=pose_goal_position_tolerance,
            pose_goal_orientation_tolerance=pose_goal_orientation_tolerance,
            pose_goal_route_candidates=pose_goal_route_candidates,
            pose_goal_planning_attempts=pose_goal_planning_attempts,
        )
        if dry_run:
            self.get_logger().info(
                "Dry run requested; selected Cartesian trajectory was not executed"
            )
            return

        self.execute_trajectory(trajectory)


class MoveItCoordinateTopicControl(MoveItRobotControl):
    def __init__(self) -> None:
        super().__init__()

        self.declare_parameter("coordinate_topic", "/moveit_robot_control/target")
        self.declare_parameter("complete_topic", "/moveit_robot_control/complete")
        self.declare_parameter("status_topic", "/moveit_robot_control/status")
        self.declare_parameter("state_topic", "/moveit_robot_control/state")
        self.declare_parameter("debug_topic", "/moveit_robot_control/debug")
        self.declare_parameter("complete_message", "complete")
        self.declare_parameter("frame", BASE_FRAME)
        self.declare_parameter("ee_link", END_EFFECTOR_LINK)
        self.declare_parameter("cartesian_max_step", CARTESIAN_MAX_STEP)
        self.declare_parameter(
            "auto_waypoint_distance", CARTESIAN_AUTO_WAYPOINT_DISTANCE
        )
        self.declare_parameter("cartesian_jump_threshold", CARTESIAN_JUMP_THRESHOLD)
        self.declare_parameter("min_cartesian_fraction", CARTESIAN_MIN_FRACTION)
        self.declare_parameter("velocity_scale", 0.2)
        self.declare_parameter("collision_check_stride", COLLISION_CHECK_STRIDE)
        self.declare_parameter("allow_pose_goal_fallback", True)
        self.declare_parameter("pose_goal_planning_time", POSE_GOAL_PLANNING_TIME)
        self.declare_parameter(
            "pose_goal_position_tolerance", POSE_GOAL_POSITION_TOLERANCE
        )
        self.declare_parameter(
            "pose_goal_orientation_tolerance", POSE_GOAL_ORIENTATION_TOLERANCE
        )
        self.declare_parameter("pose_goal_route_candidates", ROUTE_CANDIDATES)
        self.declare_parameter(
            "pose_goal_planning_attempts", MOVEIT_PLANNING_ATTEMPTS
        )
        self.declare_parameter("dry_run", False)
        self.declare_parameter("use_current_orientation", True)
        self.declare_parameter("roll_deg", 0.0)
        self.declare_parameter("pitch_deg", 0.0)
        self.declare_parameter("yaw_deg", 0.0)

        coordinate_topic = str(self.get_parameter("coordinate_topic").value)
        complete_topic = str(self.get_parameter("complete_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)

        self.complete_message = str(self.get_parameter("complete_message").value)
        self.base_frame = str(self.get_parameter("frame").value)
        self.end_effector_link = str(self.get_parameter("ee_link").value)
        self.max_step = float(self.get_parameter("cartesian_max_step").value)
        self.auto_waypoint_distance = float(
            self.get_parameter("auto_waypoint_distance").value
        )
        self.jump_threshold = float(
            self.get_parameter("cartesian_jump_threshold").value
        )
        self.min_fraction = float(self.get_parameter("min_cartesian_fraction").value)
        self.velocity_scale = float(self.get_parameter("velocity_scale").value)
        self.collision_check_stride = int(
            self.get_parameter("collision_check_stride").value
        )
        self.allow_pose_goal_fallback = bool(
            self.get_parameter("allow_pose_goal_fallback").value
        )
        self.pose_goal_planning_time = float(
            self.get_parameter("pose_goal_planning_time").value
        )
        self.pose_goal_position_tolerance = float(
            self.get_parameter("pose_goal_position_tolerance").value
        )
        self.pose_goal_orientation_tolerance = float(
            self.get_parameter("pose_goal_orientation_tolerance").value
        )
        self.pose_goal_route_candidates = int(
            self.get_parameter("pose_goal_route_candidates").value
        )
        self.pose_goal_planning_attempts = int(
            self.get_parameter("pose_goal_planning_attempts").value
        )
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.use_current_orientation = bool(
            self.get_parameter("use_current_orientation").value
        )
        self.roll_deg = float(self.get_parameter("roll_deg").value)
        self.pitch_deg = float(self.get_parameter("pitch_deg").value)
        self.yaw_deg = float(self.get_parameter("yaw_deg").value)

        self.pending_coordinates: Deque[tuple[int, tuple[float, float, float]]] = (
            deque()
        )
        self.motion_in_progress = False
        self.current_state = "STARTING"
        self.next_goal_id = 0

        state_qos = QoSProfile(depth=20)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.complete_pub = self.create_publisher(String, complete_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, state_qos)
        self.debug_pub = self.create_publisher(String, debug_topic, state_qos)
        self.create_subscription(Point, coordinate_topic, self.coordinate_cb, 10)

        self.publish_status(
            "Coordinate listener starting. Send geometry_msgs/msg/Point goals to "
            f"{coordinate_topic}; completion publishes to {complete_topic}."
        )
        self.publish_state(
            "STARTING",
            event="node_starting",
            coordinate_topic=coordinate_topic,
            complete_topic=complete_topic,
            status_topic=status_topic,
            state_topic=state_topic,
            debug_topic=debug_topic,
        )

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_complete(self) -> None:
        msg = String()
        msg.data = self.complete_message
        self.complete_pub.publish(msg)

    def publish_debug(
        self,
        event: str,
        state: Optional[str] = None,
        goal_id: Optional[int] = None,
        coordinate: Optional[tuple[float, float, float]] = None,
        **facts,
    ) -> None:
        payload = {
            "event": event,
            "state": state or self.current_state,
            "node_time_sec": self.get_clock().now().nanoseconds / 1e9,
            "wall_time_sec": time.time(),
            "queue_length": len(self.pending_coordinates),
            "motion_in_progress": self.motion_in_progress,
        }
        if goal_id is not None:
            payload["goal_id"] = goal_id
        if coordinate is not None:
            payload["target"] = {
                "x": coordinate[0],
                "y": coordinate[1],
                "z": coordinate[2],
            }
        payload.update(facts)

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.debug_pub.publish(msg)

    def publish_state(self, state: str, event: Optional[str] = None, **facts) -> None:
        self.current_state = state

        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.publish_debug(event or state.lower(), state=state, **facts)

    def coordinate_cb(self, msg: Point) -> None:
        coordinate = (float(msg.x), float(msg.y), float(msg.z))
        if not all(math.isfinite(value) for value in coordinate):
            self.publish_status(f"Ignoring non-finite coordinate goal: {coordinate}")
            self.publish_state(
                "REJECTED",
                event="goal_rejected",
                coordinate=coordinate,
                reason="non_finite_coordinate",
            )
            return

        self.next_goal_id += 1
        goal_id = self.next_goal_id
        self.pending_coordinates.append((goal_id, coordinate))
        self.publish_status(
            "Queued coordinate goal "
            f"#{goal_id}: x={coordinate[0]:.4f}, "
            f"y={coordinate[1]:.4f}, z={coordinate[2]:.4f}"
        )
        self.publish_state(
            "QUEUED",
            event="goal_queued",
            goal_id=goal_id,
            coordinate=coordinate,
        )

    def configured_orientation(self) -> Optional[Quaternion]:
        if self.use_current_orientation:
            return None

        return quaternion_from_rpy(
            math.radians(self.roll_deg),
            math.radians(self.pitch_deg),
            math.radians(self.yaw_deg),
        )

    def process_next_coordinate(self) -> None:
        if self.motion_in_progress or not self.pending_coordinates:
            return

        goal_id, coordinate = self.pending_coordinates.popleft()
        self.motion_in_progress = True
        phase = "planning"
        total_started_at = time.monotonic()
        self.publish_status(
            "Moving to coordinate goal "
            f"#{goal_id}: x={coordinate[0]:.4f}, "
            f"y={coordinate[1]:.4f}, z={coordinate[2]:.4f}"
        )
        self.publish_state(
            "PLANNING",
            event="planning_started",
            goal_id=goal_id,
            coordinate=coordinate,
            base_frame=self.base_frame,
            end_effector_link=self.end_effector_link,
            use_current_orientation=self.use_current_orientation,
            roll_deg=self.roll_deg,
            pitch_deg=self.pitch_deg,
            yaw_deg=self.yaw_deg,
            cartesian_max_step=self.max_step,
            auto_waypoint_distance=self.auto_waypoint_distance,
            cartesian_jump_threshold=self.jump_threshold,
            min_cartesian_fraction=self.min_fraction,
            velocity_scale=self.velocity_scale,
            collision_check_stride=self.collision_check_stride,
            allow_pose_goal_fallback=self.allow_pose_goal_fallback,
            pose_goal_planning_time=self.pose_goal_planning_time,
            pose_goal_position_tolerance=self.pose_goal_position_tolerance,
            pose_goal_orientation_tolerance=self.pose_goal_orientation_tolerance,
            pose_goal_route_candidates=self.pose_goal_route_candidates,
            pose_goal_planning_attempts=self.pose_goal_planning_attempts,
        )

        try:
            planning_started_at = time.monotonic()
            trajectory = self.plan_cartesian_motion(
                [coordinate],
                base_frame=self.base_frame,
                end_effector_link=self.end_effector_link,
                orientation=self.configured_orientation(),
                max_step=self.max_step,
                auto_waypoint_distance=self.auto_waypoint_distance,
                jump_threshold=self.jump_threshold,
                min_fraction=self.min_fraction,
                velocity_scale=self.velocity_scale,
                collision_check_stride=self.collision_check_stride,
                allow_pose_goal_fallback=self.allow_pose_goal_fallback,
                pose_goal_allowed_planning_time=self.pose_goal_planning_time,
                pose_goal_position_tolerance=self.pose_goal_position_tolerance,
                pose_goal_orientation_tolerance=self.pose_goal_orientation_tolerance,
                pose_goal_route_candidates=self.pose_goal_route_candidates,
                pose_goal_planning_attempts=self.pose_goal_planning_attempts,
            )
            planning_duration_sec = time.monotonic() - planning_started_at
            planned_duration_sec = trajectory_duration_sec(trajectory)
            self.publish_state(
                "PLANNED",
                event="planning_succeeded",
                goal_id=goal_id,
                coordinate=coordinate,
                planning_duration_sec=planning_duration_sec,
                planned_execution_duration_sec=planned_duration_sec,
                trajectory_points=len(trajectory.points),
                trajectory_joints=list(trajectory.joint_names),
            )

            if self.dry_run:
                self.motion_in_progress = False
                self.publish_state(
                    "DRY_RUN_COMPLETE",
                    event="dry_run_complete",
                    goal_id=goal_id,
                    coordinate=coordinate,
                    total_duration_sec=time.monotonic() - total_started_at,
                )
                self.publish_status(
                    "Dry run completed for coordinate goal "
                    f"#{goal_id}; trajectory was not executed"
                )
                return

            phase = "execution"
            self.publish_state(
                "EXECUTING",
                event="execution_started",
                goal_id=goal_id,
                coordinate=coordinate,
                planned_execution_duration_sec=planned_duration_sec,
                trajectory_points=len(trajectory.points),
            )
            execution_started_at = time.monotonic()
            self.execute_trajectory(trajectory)
            execution_duration_sec = time.monotonic() - execution_started_at

        except Exception as exc:
            self.motion_in_progress = False
            if phase == "planning":
                failure_state = "INVALID"
                failure_event = "goal_invalid"
                status_prefix = "Invalid coordinate goal"
            else:
                failure_state = "FAILED"
                failure_event = "goal_failed"
                status_prefix = "Failed coordinate goal"

            self.publish_status(
                f"{status_prefix} #{goal_id}: x={coordinate[0]:.4f}, "
                f"y={coordinate[1]:.4f}, z={coordinate[2]:.4f}: {exc}"
            )
            self.publish_state(
                failure_state,
                event=failure_event,
                goal_id=goal_id,
                coordinate=coordinate,
                phase=phase,
                error=str(exc),
                error_type=type(exc).__name__,
                total_duration_sec=time.monotonic() - total_started_at,
            )
        else:
            self.motion_in_progress = False
            self.publish_complete()
            self.publish_state(
                "COMPLETE",
                event="goal_completed",
                goal_id=goal_id,
                coordinate=coordinate,
                planning_duration_sec=planning_duration_sec,
                execution_duration_sec=execution_duration_sec,
                total_duration_sec=time.monotonic() - total_started_at,
            )
            self.publish_status(
                "Completed coordinate goal "
                f"#{goal_id}: x={coordinate[0]:.4f}, y={coordinate[1]:.4f}, "
                f"z={coordinate[2]:.4f}"
            )
        finally:
            self.motion_in_progress = False


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
        description=(
            "Move a UR robot with MoveIt. Supply x y z triples to follow "
            "straight-line Cartesian TCP paths."
        )
    )
    parser.add_argument(
        "coordinates",
        nargs="*",
        type=float,
        help=(
            "Optional Cartesian waypoints as x y z triples in meters. Example: "
            "0.40 0.20 0.25 0.40 0.30 0.25"
        ),
    )
    parser.add_argument(
        "--launch-moveit",
        action="store_true",
        help="Launch the local ur_moveit.launch.py before sending goals.",
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
    parser.add_argument(
        "--frame",
        default=BASE_FRAME,
        help=f"Planning frame for Cartesian coordinates. Default: {BASE_FRAME}",
    )
    parser.add_argument(
        "--ee-link",
        default=END_EFFECTOR_LINK,
        help=f"End-effector link to move. Default: {END_EFFECTOR_LINK}",
    )
    parser.add_argument(
        "--cartesian-max-step",
        type=float,
        default=CARTESIAN_MAX_STEP,
        help=(
            "Maximum Cartesian interpolation step in meters. Smaller values "
            f"track the straight line more tightly. Default: {CARTESIAN_MAX_STEP}"
        ),
    )
    parser.add_argument(
        "--auto-waypoint-distance",
        type=float,
        default=CARTESIAN_AUTO_WAYPOINT_DISTANCE,
        help=(
            "Automatically insert intermediate Cartesian waypoints so no requested "
            "leg is longer than this many meters. Use 0 to disable. Default: "
            f"{CARTESIAN_AUTO_WAYPOINT_DISTANCE}"
        ),
    )
    parser.add_argument(
        "--cartesian-jump-threshold",
        type=float,
        default=CARTESIAN_JUMP_THRESHOLD,
        help=(
            "MoveIt jump threshold for Cartesian paths. Use 0 to disable. "
            f"Default: {CARTESIAN_JUMP_THRESHOLD}"
        ),
    )
    parser.add_argument(
        "--min-cartesian-fraction",
        type=float,
        default=CARTESIAN_MIN_FRACTION,
        help=(
            "Refuse Cartesian paths unless MoveIt computes at least this "
            f"fraction of the straight line. Default: {CARTESIAN_MIN_FRACTION}"
        ),
    )
    parser.add_argument(
        "--no-pose-goal-fallback",
        action="store_true",
        help=(
            "Do not fall back to general MoveIt pose-goal planning when a "
            "single-point Cartesian path is blocked by collision."
        ),
    )
    parser.add_argument(
        "--pose-goal-position-tolerance",
        type=float,
        default=POSE_GOAL_POSITION_TOLERANCE,
        help=(
            "Position tolerance in meters for the alternate pose-goal planner. "
            f"Default: {POSE_GOAL_POSITION_TOLERANCE}"
        ),
    )
    parser.add_argument(
        "--pose-goal-orientation-tolerance",
        type=float,
        default=POSE_GOAL_ORIENTATION_TOLERANCE,
        help=(
            "Orientation tolerance in radians for the alternate pose-goal "
            f"planner. Default: {POSE_GOAL_ORIENTATION_TOLERANCE}"
        ),
    )
    parser.add_argument(
        "--roll-deg",
        type=float,
        help="Optional target tool roll in degrees for Cartesian moves.",
    )
    parser.add_argument(
        "--pitch-deg",
        type=float,
        help="Optional target tool pitch in degrees for Cartesian moves.",
    )
    parser.add_argument(
        "--yaw-deg",
        type=float,
        help="Optional target tool yaw in degrees for Cartesian moves.",
    )
    parser.add_argument(
        "--allowed-planning-time",
        type=float,
        default=5.0,
        help="MoveIt joint-goal planning timeout in seconds. Default: 5.0",
    )
    parser.add_argument(
        "--velocity-scale",
        type=float,
        default=0.2,
        help="Velocity scale from 0 to 1. Default: 0.2",
    )
    parser.add_argument(
        "--acceleration-scale",
        type=float,
        default=0.2,
        help="Joint-goal acceleration scale from 0 to 1. Default: 0.2",
    )
    parser.add_argument(
        "--route-candidates",
        type=int,
        default=ROUTE_CANDIDATES,
        help=(
            "For joint-goal moves and pose-goal fallback, plan this many "
            "candidate routes and execute the shortest collision-free one. "
            f"Default: {ROUTE_CANDIDATES}"
        ),
    )
    parser.add_argument(
        "--moveit-planning-attempts",
        type=int,
        default=MOVEIT_PLANNING_ATTEMPTS,
        help=(
            "MoveIt planning attempts per candidate route. Default: "
            f"{MOVEIT_PLANNING_ATTEMPTS}"
        ),
    )
    parser.add_argument(
        "--collision-check-stride",
        type=int,
        default=COLLISION_CHECK_STRIDE,
        help=(
            "Check every Nth trajectory point with MoveIt's state validity "
            f"service. Default: {COLLISION_CHECK_STRIDE}"
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Plan and select routes, but do not publish trajectories.",
    )
    return parser.parse_args()


def orientation_from_args(args: argparse.Namespace) -> Optional[Quaternion]:
    values = (args.roll_deg, args.pitch_deg, args.yaw_deg)
    if all(value is None for value in values):
        return None
    if any(value is None for value in values):
        raise ValueError("Supply all degree RPY values: --roll-deg --pitch-deg --yaw-deg")

    return quaternion_from_rpy(
        math.radians(args.roll_deg),
        math.radians(args.pitch_deg),
        math.radians(args.yaw_deg),
    )


def main() -> None:
    args = parse_args()
    moveit_process = None

    if args.launch_moveit:
        moveit_process = maybe_launch_moveit(args.ur_type, args.launch_rviz)
        time.sleep(8.0)

    rclpy.init()
    node = MoveItRobotControl()

    try:
        cartesian_coordinates = parse_coordinate_triples(args.coordinates)
        cartesian_orientation = orientation_from_args(args)

        if cartesian_coordinates:
            node.move_to_cartesian_coordinates(
                cartesian_coordinates,
                base_frame=args.frame,
                end_effector_link=args.ee_link,
                orientation=cartesian_orientation,
                max_step=args.cartesian_max_step,
                auto_waypoint_distance=args.auto_waypoint_distance,
                jump_threshold=args.cartesian_jump_threshold,
                min_fraction=args.min_cartesian_fraction,
                velocity_scale=args.velocity_scale,
                collision_check_stride=args.collision_check_stride,
                allow_pose_goal_fallback=not args.no_pose_goal_fallback,
                pose_goal_allowed_planning_time=args.allowed_planning_time,
                pose_goal_position_tolerance=args.pose_goal_position_tolerance,
                pose_goal_orientation_tolerance=args.pose_goal_orientation_tolerance,
                pose_goal_route_candidates=args.route_candidates,
                pose_goal_planning_attempts=args.moveit_planning_attempts,
                dry_run=args.dry_run,
            )
            return

        node.wait_for_moveit()

        first_goal = np.deg2rad([36.10, -75.63, 68.76, -84.23, -88.24, 0.11]).tolist()
        second_goal = np.deg2rad([-49.57, -92.18, -79.95, -93.72, 89.42, 0.0]).tolist()

        node.move_to_joint_positions(
            first_goal,
            allowed_planning_time=args.allowed_planning_time,
            velocity_scale=args.velocity_scale,
            acceleration_scale=args.acceleration_scale,
            route_candidates=args.route_candidates,
            moveit_planning_attempts=args.moveit_planning_attempts,
            collision_check_stride=args.collision_check_stride,
            dry_run=args.dry_run,
        )
        time.sleep(2.0)
        node.move_to_joint_positions(
            second_goal,
            allowed_planning_time=args.allowed_planning_time,
            velocity_scale=args.velocity_scale,
            acceleration_scale=args.acceleration_scale,
            route_candidates=args.route_candidates,
            moveit_planning_attempts=args.moveit_planning_attempts,
            collision_check_stride=args.collision_check_stride,
            dry_run=args.dry_run,
        )

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

        if moveit_process is not None:
            moveit_process.terminate()
            moveit_process.wait(timeout=5)


def coordinate_listener_main() -> None:
    rclpy.init()
    node = MoveItCoordinateTopicControl()

    try:
        node.wait_for_moveit()
        node.wait_for_cartesian_path_service()
        node.wait_for_state_validity()
        node.apply_floor_collision_object(node.base_frame)
        node.publish_status("Coordinate listener ready for target points")
        node.publish_state("READY", event="listener_ready")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.process_next_coordinate()

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    except Exception as exc:
        node.publish_status(f"Coordinate listener failed: {exc}")
        node.publish_state(
            "FAILED",
            event="listener_failed",
            phase="startup_or_runtime",
            error=str(exc),
            error_type=type(exc).__name__,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
