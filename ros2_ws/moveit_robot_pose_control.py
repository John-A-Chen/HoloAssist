#!/usr/bin/env python3
import argparse
import math
import subprocess
import time
from typing import List, Optional, Sequence

import rclpy
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.msg import SafetyMode


JOINT_STATES_TOPIC = "/joint_states"
TRAJECTORY_TOPIC = "/scaled_joint_trajectory_controller/joint_trajectory"
MOVEIT_PLANNING_SERVICE = "/plan_kinematic_path"
MOVEIT_IK_SERVICE = "/compute_ik"
MOVEIT_APPLY_SCENE_SERVICE = "/apply_planning_scene"
MOVEIT_LAUNCH_FILE = "/home/ollie/ros2_ws/ur_moveit.launch.py"
DEFAULT_MOVE_GROUP_NAME = "ur_manipulator"
DEFAULT_PLANNING_FRAME = "base_link"
DEFAULT_IK_LINK_NAME = "tool0"
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


class MoveItRobotPoseControl(Node):
    def __init__(self, move_group_name: str, ik_link_name: str) -> None:
        super().__init__("moveit_robot_pose_control")
        self.move_group_name = move_group_name
        self.ik_link_name = ik_link_name
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
        self.ik_client = self.create_client(GetPositionIK, MOVEIT_IK_SERVICE)
        self.apply_planning_scene_client = self.create_client(
            ApplyPlanningScene, MOVEIT_APPLY_SCENE_SERVICE
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
        self.get_logger().info("Waiting for MoveIt planning and IK services...")
        if not self.plan_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"MoveIt planning service {MOVEIT_PLANNING_SERVICE} is not available")
        if not self.ik_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"MoveIt IK service {MOVEIT_IK_SERVICE} is not available")

    @staticmethod
    def wrap_angle_near_reference(target_angle: float, reference_angle: float) -> float:
        # Keep equivalent angle close to current joint value to avoid large spin moves.
        return reference_angle + math.atan2(
            math.sin(target_angle - reference_angle),
            math.cos(target_angle - reference_angle),
        )

    def apply_virtual_floor(
        self,
        frame_id: str,
        floor_z: float = 0.0,
        floor_size_xy: float = 4.0,
        floor_thickness: float = 0.02,
    ) -> None:
        if not self.apply_planning_scene_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"{MOVEIT_APPLY_SCENE_SERVICE} is not available")

        floor = CollisionObject()
        floor.header.frame_id = frame_id
        floor.id = "virtual_floor"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(floor_size_xy), float(floor_size_xy), float(floor_thickness)]

        floor_pose = Pose()
        floor_pose.position.x = 0.0
        floor_pose.position.y = 0.0
        floor_pose.position.z = float(floor_z) - (float(floor_thickness) / 2.0)
        floor_pose.orientation.w = 1.0

        floor.primitives = [primitive]
        floor.primitive_poses = [floor_pose]
        floor.operation = CollisionObject.ADD

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(floor)

        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        future = self.apply_planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("Failed to apply virtual floor collision object")

        self.get_logger().info(
            f"Applied virtual floor at z={floor_z:.3f} m in frame '{frame_id}'"
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

    def moveit_error_name(self, code: int) -> str:
        return MOVEIT_ERROR_CODES.get(code, f"UNKNOWN_ERROR_{code}")

    def solve_ik_for_pose(
        self,
        pose_stamped: PoseStamped,
        ik_timeout_sec: float = 2.0,
        ik_attempts: int = 10,
        avoid_collisions: bool = True,
    ) -> List[float]:
        current_pos = self.wait_for_joint_state()

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.move_group_name
        request.ik_request.ik_link_name = self.ik_link_name
        request.ik_request.pose_stamped = pose_stamped
        # MoveIt message fields differ across ROS/MoveIt versions.
        # Humble's PositionIKRequest has no 'attempts' field.
        if hasattr(request.ik_request, "attempts"):
            request.ik_request.attempts = ik_attempts
        elif ik_attempts != 10:
            self.get_logger().warn(
                "IK attempts requested, but this MoveIt version does not expose "
                "PositionIKRequest.attempts; ignoring --ik-attempts."
            )
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.timeout.sec = int(ik_timeout_sec)
        request.ik_request.timeout.nanosec = int((ik_timeout_sec % 1.0) * 1e9)
        request.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.robot_state.joint_state.name = list(UR_JOINT_ORDER)
        request.ik_request.robot_state.joint_state.position = list(current_pos)
        request.ik_request.robot_state.is_diff = False

        self.get_logger().info(
            f"Requesting IK for pose in frame {pose_stamped.header.frame_id} using link {self.ik_link_name}"
        )
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=ik_timeout_sec + 5.0)

        if not future.done():
            raise RuntimeError("MoveIt IK service timed out before returning a response")
        if future.exception() is not None:
            raise RuntimeError(f"MoveIt IK service raised an exception: {future.exception()}")

        response = future.result()
        if response is None:
            raise RuntimeError("MoveIt IK service did not return a response")

        error_code = response.error_code.val
        error_name = self.moveit_error_name(error_code)
        if error_code != 1:
            raise RuntimeError(f"MoveIt IK failed: {error_name} ({error_code})")

        solution_names = list(response.solution.joint_state.name)
        solution_positions = list(response.solution.joint_state.position)
        if len(solution_names) != len(solution_positions):
            raise RuntimeError("Received malformed IK solution from MoveIt")

        solution_map = {
            joint_name: joint_position
            for joint_name, joint_position in zip(solution_names, solution_positions)
        }
        if not all(joint_name in solution_map for joint_name in UR_JOINT_ORDER):
            missing = [joint for joint in UR_JOINT_ORDER if joint not in solution_map]
            raise RuntimeError(f"IK solution missing expected UR joints: {missing}")

        ordered_solution = [float(solution_map[joint_name]) for joint_name in UR_JOINT_ORDER]
        return [
            self.wrap_angle_near_reference(target, reference)
            for target, reference in zip(ordered_solution, current_pos)
        ]

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
        request.motion_plan_request.group_name = self.move_group_name
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

        self.get_logger().info(f"Requesting MoveIt plan for group {self.move_group_name}")
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=allowed_planning_time + 10.0)

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

    def move_to_cartesian_pose(
        self,
        pose_stamped: PoseStamped,
        allowed_planning_time: float = 5.0,
        velocity_scale: float = 0.2,
        acceleration_scale: float = 0.2,
        ik_timeout_sec: float = 2.0,
        ik_attempts: int = 10,
        avoid_collisions: bool = True,
    ) -> List[float]:
        joint_positions = self.solve_ik_for_pose(
            pose_stamped,
            ik_timeout_sec=ik_timeout_sec,
            ik_attempts=ik_attempts,
            avoid_collisions=avoid_collisions,
        )
        self.move_to_joint_positions(
            joint_positions,
            allowed_planning_time=allowed_planning_time,
            velocity_scale=velocity_scale,
            acceleration_scale=acceleration_scale,
        )
        return joint_positions


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


def normalized_quaternion(qx: float, qy: float, qz: float, qw: float):
    norm = math.sqrt((qx * qx) + (qy * qy) + (qz * qz) + (qw * qw))
    if norm < 1e-9:
        raise ValueError("Quaternion norm is zero")
    return qx / norm, qy / norm, qz / norm, qw / norm


def pose_stamped_from_values(
    pose_values: Sequence[float], frame_id: str, stamp
) -> PoseStamped:
    if len(pose_values) != 7:
        raise ValueError("Pose requires exactly 7 values: x y z qx qy qz qw")

    x, y, z, qx, qy, qz, qw = [float(value) for value in pose_values]
    qx, qy, qz, qw = normalized_quaternion(qx, qy, qz, qw)

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.header.stamp = stamp
    pose_stamped.pose.position.x = x
    pose_stamped.pose.position.y = y
    pose_stamped.pose.position.z = z
    pose_stamped.pose.orientation.x = qx
    pose_stamped.pose.orientation.y = qy
    pose_stamped.pose.orientation.z = qz
    pose_stamped.pose.orientation.w = qw
    return pose_stamped


def parse_args():
    parser = argparse.ArgumentParser(
        description="Move UR robot to Cartesian poses by solving IK in MoveIt and executing the plan."
    )
    parser.add_argument(
        "--pose",
        action="append",
        nargs=7,
        type=float,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help=(
            "Target Cartesian pose as 7 values. Repeat --pose for multiple targets. "
            "Frame is controlled by --frame-id."
        ),
    )
    parser.add_argument(
        "--frame-id",
        default=DEFAULT_PLANNING_FRAME,
        help="Reference frame for each pose (default: base_link).",
    )
    parser.add_argument(
        "--move-group",
        default=DEFAULT_MOVE_GROUP_NAME,
        help="MoveIt group name (default: ur_manipulator).",
    )
    parser.add_argument(
        "--ik-link-name",
        default=DEFAULT_IK_LINK_NAME,
        help="End-effector link for IK (default: tool0).",
    )
    parser.add_argument(
        "--allowed-planning-time",
        type=float,
        default=5.0,
        help="MoveIt allowed planning time in seconds.",
    )
    parser.add_argument(
        "--velocity-scale",
        type=float,
        default=0.2,
        help="Max velocity scaling factor [0,1].",
    )
    parser.add_argument(
        "--acceleration-scale",
        type=float,
        default=0.2,
        help="Max acceleration scaling factor [0,1].",
    )
    parser.add_argument(
        "--ik-timeout",
        type=float,
        default=2.0,
        help="IK timeout in seconds for each pose.",
    )
    parser.add_argument(
        "--ik-attempts",
        type=int,
        default=10,
        help="Number of IK attempts for each pose.",
    )
    parser.add_argument(
        "--allow-collisions-in-ik",
        action="store_true",
        help="Allow IK results that are in collision (disabled by default).",
    )
    parser.add_argument(
        "--pause-between-goals",
        type=float,
        default=1.0,
        help="Pause in seconds between sequential pose goals.",
    )
    parser.add_argument(
        "--add-virtual-floor",
        action="store_true",
        help="Add a floor collision plane to MoveIt to prevent paths dipping below floor z.",
    )
    parser.add_argument(
        "--floor-z",
        type=float,
        default=0.0,
        help="Floor height in --frame-id coordinates (used with --add-virtual-floor).",
    )
    parser.add_argument(
        "--floor-size",
        type=float,
        default=4.0,
        help="Virtual floor X/Y size in meters (used with --add-virtual-floor).",
    )
    parser.add_argument(
        "--floor-thickness",
        type=float,
        default=0.02,
        help="Virtual floor thickness in meters (used with --add-virtual-floor).",
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
    node = MoveItRobotPoseControl(
        move_group_name=args.move_group,
        ik_link_name=args.ik_link_name,
    )

    default_pose_values = [
        [0.25, -0.20, 0.30, 1.0, 0.0, 0.0, 0.0],
        [0.25, 0.20, 0.30, 1.0, 0.0, 0.0, 0.0],
    ]

    pose_values_list = args.pose if args.pose is not None else default_pose_values
    if args.pose is None:
        node.get_logger().info("No --pose provided; using built-in demo Cartesian poses.")

    try:
        node.wait_for_moveit()
        if args.add_virtual_floor:
            node.apply_virtual_floor(
                frame_id=args.frame_id,
                floor_z=args.floor_z,
                floor_size_xy=args.floor_size,
                floor_thickness=args.floor_thickness,
            )

        for index, pose_values in enumerate(pose_values_list, start=1):
            pose_stamped = pose_stamped_from_values(
                pose_values, args.frame_id, node.get_clock().now().to_msg()
            )
            node.get_logger().info(
                f"Goal {index}: position=({pose_stamped.pose.position.x:.3f}, "
                f"{pose_stamped.pose.position.y:.3f}, {pose_stamped.pose.position.z:.3f}), "
                f"orientation=({pose_stamped.pose.orientation.x:.4f}, "
                f"{pose_stamped.pose.orientation.y:.4f}, {pose_stamped.pose.orientation.z:.4f}, "
                f"{pose_stamped.pose.orientation.w:.4f})"
            )

            solved_joints = node.move_to_cartesian_pose(
                pose_stamped=pose_stamped,
                allowed_planning_time=args.allowed_planning_time,
                velocity_scale=args.velocity_scale,
                acceleration_scale=args.acceleration_scale,
                ik_timeout_sec=args.ik_timeout,
                ik_attempts=args.ik_attempts,
                avoid_collisions=not args.allow_collisions_in_ik,
            )
            solved_degrees = [round(math.degrees(joint), 2) for joint in solved_joints]
            node.get_logger().info(f"Goal {index} IK solution (deg): {solved_degrees}")

            if index < len(pose_values_list):
                time.sleep(args.pause_between_goals)

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
