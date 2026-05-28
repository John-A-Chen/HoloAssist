#!/usr/bin/env python3
import json
import math
import threading
import time
import traceback
from pathlib import Path
from typing import Any, Optional, Sequence

import rclpy
from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


DEFAULT_FRAME_ID = "base_link"
DEFAULT_BLOCK_POSE_TOPIC = "/pick_place/block_pose"
DEFAULT_COMMAND_TOPIC = "/pick_place/command"
DEFAULT_MODE_TOPIC = "/pick_place/mode"
DEFAULT_STATUS_TOPIC = "/pick_place/status"
DEFAULT_TARGET_POSE_TOPIC = "/moveit_robot_control/target_pose"
DEFAULT_TARGET_POINT_TOPIC = "/moveit_robot_control/target_point"
DEFAULT_TARGET_JOINT_STATE_TOPIC = "/moveit_robot_control/target_joint_state"
DEFAULT_MOVE_STATE_TOPIC = "/moveit_robot_control/state"
DEFAULT_MOVE_COMPLETE_TOPIC = "/moveit_robot_control/complete"
DEFAULT_MOVE_STOP_TOPIC = "/moveit_robot_control/stop"
DEFAULT_JOINT_STATES_TOPIC = "/joint_states"
DEFAULT_ARM_TRAJECTORY_TOPIC = "/scaled_joint_trajectory_controller/joint_trajectory"
DEFAULT_GRIPPER_TOPIC = "/finger_width_trajectory_controller/joint_trajectory"
DEFAULT_WORKSPACE_COMMAND_TOPIC = "/workspace_scene/command"
DEFAULT_APPLY_PLANNING_SCENE_SERVICE = "/apply_planning_scene"
PACKAGE_NAME = "moveit_robot_control"

FALLBACK_BIN_POSES = {
    "bin_1": {"xyz": [-0.30, -0.20, 0.05], "rpy_deg": [180.0, 0.0, 0.0]},
    "bin_2": {"xyz": [-0.30, -0.10, 0.05], "rpy_deg": [180.0, 0.0, 0.0]},
    "bin_3": {"xyz": [0.30, -0.20, 0.05], "rpy_deg": [180.0, 0.0, 0.0]},
    "bin_4": {"xyz": [0.30, -0.10, 0.05], "rpy_deg": [180.0, 0.0, 0.0]},
}

FAILURE_STATES = {"FAILED", "INVALID", "REJECTED"}
ACTIVE_STATES = {"QUEUED", "PLANNING", "PLANNED", "EXECUTING"}
RUN_MODE = "RUN"
STOP_MODE = "STOP"
AUTO_ORIENTATION_MODE = "auto"
FIXED_ORIENTATION_MODE = "fixed"
UR_JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
DEFAULT_HOME_JOINT_POSITIONS_DEG = [-75.0, -90.0, -50.0, -120.0, 90.0, 0.0]


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


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


def pose_from_xyz_rpy(xyz: Sequence[float], rpy_deg: Sequence[float]) -> Pose:
    pose = Pose()
    pose.position.x = float(xyz[0])
    pose.position.y = float(xyz[1])
    pose.position.z = float(xyz[2])
    pose.orientation = quaternion_from_rpy(
        math.radians(float(rpy_deg[0])),
        math.radians(float(rpy_deg[1])),
        math.radians(float(rpy_deg[2])),
    )
    return pose


def finite_values(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def set_duration_seconds(duration, seconds: float) -> None:
    seconds = max(0.0, float(seconds))
    duration.sec = int(seconds)
    duration.nanosec = int(round((seconds - duration.sec) * 1e9))
    if duration.nanosec >= 1_000_000_000:
        duration.sec += 1
        duration.nanosec -= 1_000_000_000


def bin_config_candidates() -> list[Path]:
    candidates = []
    try:
        share_dir = Path(get_package_share_directory(PACKAGE_NAME))
        candidates.append(share_dir / "config" / "bin_poses.json")
    except PackageNotFoundError:
        pass

    source_path = Path(__file__).resolve().parent.parent / "config" / "bin_poses.json"
    if source_path not in candidates:
        candidates.append(source_path)
    return candidates


def default_bin_config_path() -> str:
    for candidate in bin_config_candidates():
        if candidate.is_file():
            return str(candidate)
    return str(bin_config_candidates()[0])


def parse_bin_pose_config(raw_config: Any) -> dict[str, dict[str, list[float]]]:
    if not isinstance(raw_config, dict) or not raw_config:
        raise ValueError("bin pose config must be a non-empty JSON object")

    bin_pose_config = {}
    for raw_bin_id, raw_bin_pose in raw_config.items():
        bin_id = str(raw_bin_id).strip()
        if not bin_id:
            raise ValueError("bin pose config contains an empty bin id")
        if not isinstance(raw_bin_pose, dict):
            raise ValueError(f"bin {bin_id} must map to an object")

        xyz = [float(value) for value in raw_bin_pose.get("xyz", [])]
        rpy_deg = [float(value) for value in raw_bin_pose.get("rpy_deg", [])]

        if len(xyz) != 3:
            raise ValueError(f"bin {bin_id} xyz must contain 3 values")
        if len(rpy_deg) != 3:
            raise ValueError(f"bin {bin_id} rpy_deg must contain 3 values")
        if not finite_values(xyz) or not finite_values(rpy_deg):
            raise ValueError(f"bin {bin_id} contains a non-finite value")

        bin_pose_config[bin_id] = {"xyz": xyz, "rpy_deg": rpy_deg}

    return bin_pose_config


def load_bin_pose_defaults(config_path: str) -> tuple[dict[str, dict[str, list[float]]], str]:
    path_text = config_path.strip()
    if path_text:
        requested_path = Path(path_text).expanduser()
        if requested_path.is_file():
            with requested_path.open("r", encoding="utf-8") as config_file:
                return parse_bin_pose_config(json.load(config_file)), str(requested_path)
        raise FileNotFoundError(f"Bin config file not found: {requested_path}")

    for candidate in bin_config_candidates():
        if candidate.is_file():
            with candidate.open("r", encoding="utf-8") as config_file:
                return parse_bin_pose_config(json.load(config_file)), str(candidate)

    return parse_bin_pose_config(FALLBACK_BIN_POSES), "<built-in defaults>"


class PickPlaceSequencer(Node):
    """Runs a bin-aware pick and drop sequence."""

    def __init__(self) -> None:
        super().__init__("pick_place_sequencer")

        self.declare_parameter("frame_id", DEFAULT_FRAME_ID)
        self.declare_parameter("block_pose_topic", DEFAULT_BLOCK_POSE_TOPIC)
        self.declare_parameter("command_topic", DEFAULT_COMMAND_TOPIC)
        self.declare_parameter("mode_topic", DEFAULT_MODE_TOPIC)
        self.declare_parameter("status_topic", DEFAULT_STATUS_TOPIC)
        self.declare_parameter("target_pose_topic", DEFAULT_TARGET_POSE_TOPIC)
        self.declare_parameter("target_point_topic", DEFAULT_TARGET_POINT_TOPIC)
        self.declare_parameter(
            "target_joint_state_topic", DEFAULT_TARGET_JOINT_STATE_TOPIC
        )
        self.declare_parameter("move_state_topic", DEFAULT_MOVE_STATE_TOPIC)
        self.declare_parameter("move_complete_topic", DEFAULT_MOVE_COMPLETE_TOPIC)
        self.declare_parameter("move_stop_topic", DEFAULT_MOVE_STOP_TOPIC)
        self.declare_parameter("joint_states_topic", DEFAULT_JOINT_STATES_TOPIC)
        self.declare_parameter("arm_trajectory_topic", DEFAULT_ARM_TRAJECTORY_TOPIC)
        self.declare_parameter("gripper_topic", DEFAULT_GRIPPER_TOPIC)
        self.declare_parameter(
            "workspace_command_topic", DEFAULT_WORKSPACE_COMMAND_TOPIC
        )
        self.declare_parameter(
            "apply_planning_scene_service",
            DEFAULT_APPLY_PLANNING_SCENE_SERVICE,
        )

        self.declare_parameter("block_id", "block_1")
        self.declare_parameter("block_size", [0.05, 0.05, 0.05])
        self.declare_parameter("block_color", [0.1, 0.45, 0.95, 1.0])
        self.declare_parameter("place_xyz", [0.45, -0.10, 0.05])
        self.declare_parameter("place_rpy_deg", [180.0, 0.0, 0.0])
        self.declare_parameter("bin_config_path", default_bin_config_path())
        self.bin_pose_defaults, self.loaded_bin_config_path = load_bin_pose_defaults(
            str(self.get_parameter("bin_config_path").value)
        )
        for bin_id, bin_pose in self.bin_pose_defaults.items():
            self.declare_parameter(f"{bin_id}_xyz", bin_pose["xyz"])
            self.declare_parameter(f"{bin_id}_rpy_deg", bin_pose["rpy_deg"])
        self.declare_parameter("default_bin_id", "")
        self.declare_parameter("item_bin_map", "{}")

        self.declare_parameter("tool_roll_deg", 180.0)
        self.declare_parameter("tool_pitch_deg", 0.0)
        self.declare_parameter("tool_yaw_offset_deg", 0.0)
        self.declare_parameter("orientation_mode", FIXED_ORIENTATION_MODE)
        self.declare_parameter("use_block_yaw", True)
        self.declare_parameter("use_place_yaw", True)

        self.declare_parameter("pregrasp_z_offset", 0.11)
        self.declare_parameter("grasp_z_offset", 0.05)
        self.declare_parameter("grasp_z_absolute", -1.0)
        self.declare_parameter("place_above_z_offset", 0.17)
        self.declare_parameter("place_z_offset", 0.05)
        self.declare_parameter("place_descent_enabled", False)

        self.declare_parameter("open_width", 0.08)
        self.declare_parameter("close_width", 0.0)
        self.declare_parameter("gripper_motion_sec", 2.0)
        self.declare_parameter("stop_hold_sec", 0.1)
        self.declare_parameter("move_timeout_sec", 60.0)
        self.declare_parameter("scene_update_timeout_sec", 5.0)
        self.declare_parameter("initial_mode", "stop")
        self.declare_parameter("remove_block_collision_before_grasp", True)
        self.declare_parameter("add_block_at_place_after_release", True)
        self.declare_parameter("home_enabled", True)
        self.declare_parameter("home_before_pick", True)
        self.declare_parameter("home_after_place", True)
        self.declare_parameter("home_joint_names", UR_JOINT_ORDER)
        self.declare_parameter(
            "home_joint_positions_deg", DEFAULT_HOME_JOINT_POSITIONS_DEG
        )

        self.frame_id = str(self.get_parameter("frame_id").value)
        block_pose_topic = str(self.get_parameter("block_pose_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        mode_topic = str(self.get_parameter("mode_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        target_point_topic = str(self.get_parameter("target_point_topic").value)
        target_joint_state_topic = str(
            self.get_parameter("target_joint_state_topic").value
        )
        move_state_topic = str(self.get_parameter("move_state_topic").value)
        move_complete_topic = str(self.get_parameter("move_complete_topic").value)
        move_stop_topic = str(self.get_parameter("move_stop_topic").value)
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        arm_trajectory_topic = str(self.get_parameter("arm_trajectory_topic").value)
        gripper_topic = str(self.get_parameter("gripper_topic").value)
        self.workspace_command_topic = str(
            self.get_parameter("workspace_command_topic").value
        )
        apply_scene_service = str(
            self.get_parameter("apply_planning_scene_service").value
        )
        self.bin_config_path = str(self.get_parameter("bin_config_path").value)

        self.default_block_id = str(self.get_parameter("block_id").value)
        self.block_size = self.get_vector_parameter("block_size", 3)
        self.block_color = self.get_vector_parameter("block_color", 4)
        self.default_place_pose = pose_from_xyz_rpy(
            self.get_vector_parameter("place_xyz", 3),
            self.get_vector_parameter("place_rpy_deg", 3),
        )
        self.bin_poses = self.get_bin_poses()
        self.default_bin_id = self.normalize_bin_id(
            str(self.get_parameter("default_bin_id").value).strip()
        )
        if self.default_bin_id and self.default_bin_id not in self.bin_poses:
            available = ", ".join(sorted(self.bin_poses.keys()))
            raise ValueError(
                f"default_bin_id {self.default_bin_id!r} is unknown; "
                f"available bins are {available}"
            )
        self.item_bin_map = self.get_item_bin_map_parameter("item_bin_map")

        self.tool_roll_deg = float(self.get_parameter("tool_roll_deg").value)
        self.tool_pitch_deg = float(self.get_parameter("tool_pitch_deg").value)
        self.tool_yaw_offset_deg = float(
            self.get_parameter("tool_yaw_offset_deg").value
        )
        self.orientation_mode = self.normalize_orientation_mode(
            str(self.get_parameter("orientation_mode").value)
        )
        self.use_block_yaw = bool(self.get_parameter("use_block_yaw").value)
        self.use_place_yaw = bool(self.get_parameter("use_place_yaw").value)

        self.pregrasp_z_offset = float(
            self.get_parameter("pregrasp_z_offset").value
        )
        self.grasp_z_offset = float(self.get_parameter("grasp_z_offset").value)
        self.grasp_z_absolute = float(self.get_parameter("grasp_z_absolute").value)
        self.place_above_z_offset = float(
            self.get_parameter("place_above_z_offset").value
        )
        self.place_z_offset = float(self.get_parameter("place_z_offset").value)
        self.place_descent_enabled = bool(
            self.get_parameter("place_descent_enabled").value
        )

        self.open_width = float(self.get_parameter("open_width").value)
        self.close_width = float(self.get_parameter("close_width").value)
        self.gripper_motion_sec = float(
            self.get_parameter("gripper_motion_sec").value
        )
        self.stop_hold_sec = max(0.0, float(self.get_parameter("stop_hold_sec").value))
        self.move_timeout_sec = float(self.get_parameter("move_timeout_sec").value)
        self.scene_update_timeout_sec = float(
            self.get_parameter("scene_update_timeout_sec").value
        )
        self.current_mode = self.normalize_mode(
            str(self.get_parameter("initial_mode").value)
        )
        self.remove_block_collision_before_grasp = bool(
            self.get_parameter("remove_block_collision_before_grasp").value
        )
        self.add_block_at_place_after_release = bool(
            self.get_parameter("add_block_at_place_after_release").value
        )
        self.home_enabled = bool(self.get_parameter("home_enabled").value)
        self.home_before_pick = bool(self.get_parameter("home_before_pick").value)
        self.home_after_place = bool(self.get_parameter("home_after_place").value)
        self.home_joint_names = [
            str(joint_name)
            for joint_name in self.get_parameter("home_joint_names").value
        ]
        self.home_joint_positions_deg = self.get_vector_parameter(
            "home_joint_positions_deg", len(self.home_joint_names)
        )
        self.home_joint_positions_rad = [
            math.radians(position_deg)
            for position_deg in self.home_joint_positions_deg
        ]

        self.motion_complete = threading.Event()
        self.motion_failed = threading.Event()
        self.motion_started = threading.Event()
        self.motion_waiting = threading.Event()
        self.abort_requested = threading.Event()
        self.sequence_lock = threading.Lock()
        self.sequence_thread: Optional[threading.Thread] = None
        self.pending_request: Optional[dict[str, Any]] = None
        self.last_failure_state = ""
        self.latest_joint_positions: dict[str, float] = {}
        self.latest_gripper_width: Optional[float] = None
        self.current_sequence_context: dict[str, Any] = {}

        self.target_pose_pub = self.create_publisher(Pose, target_pose_topic, 10)
        self.target_point_pub = self.create_publisher(Point, target_point_topic, 10)
        self.target_joint_state_pub = self.create_publisher(
            JointState, target_joint_state_topic, 10
        )
        self.move_stop_pub = self.create_publisher(String, move_stop_topic, 10)
        self.arm_trajectory_pub = self.create_publisher(
            JointTrajectory, arm_trajectory_topic, 10
        )
        self.gripper_pub = self.create_publisher(JointTrajectory, gripper_topic, 10)
        self.workspace_command_pub = self.create_publisher(
            String, self.workspace_command_topic, 10
        )
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, apply_scene_service
        )

        self.create_subscription(JointState, joint_states_topic, self.joint_state_cb, 10)
        self.create_subscription(PoseStamped, block_pose_topic, self.block_pose_cb, 10)
        self.create_subscription(String, command_topic, self.command_cb, 10)
        self.create_subscription(String, mode_topic, self.mode_cb, 10)
        self.create_subscription(String, move_complete_topic, self.move_complete_cb, 10)
        self.create_subscription(String, move_state_topic, self.move_state_cb, 10)

        self.publish_status(
            "Pick-place sequencer ready",
            block_pose_topic=block_pose_topic,
            command_topic=command_topic,
            mode_topic=mode_topic,
            target_pose_topic=target_pose_topic,
            target_point_topic=target_point_topic,
            target_joint_state_topic=target_joint_state_topic,
            move_stop_topic=move_stop_topic,
            joint_states_topic=joint_states_topic,
            arm_trajectory_topic=arm_trajectory_topic,
            gripper_topic=gripper_topic,
            place_xyz=[
                self.default_place_pose.position.x,
                self.default_place_pose.position.y,
                self.default_place_pose.position.z,
            ],
            bin_config_path=self.loaded_bin_config_path,
            bins=sorted(self.bin_poses.keys()),
            default_bin_id=self.default_bin_id,
            mode=self.current_mode.lower(),
            orientation_mode=self.orientation_mode,
            home_enabled=self.home_enabled,
            home_joint_names=self.home_joint_names,
            home_joint_positions_deg=self.home_joint_positions_deg,
        )

    def get_vector_parameter(self, name: str, length: int) -> list[float]:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            text = value.strip()
            if text.startswith("["):
                parsed = json.loads(text)
            else:
                parsed = [part for part in text.replace(",", " ").split() if part]
        else:
            parsed = value

        vector = [float(item) for item in parsed]
        if len(vector) != length:
            raise ValueError(f"Parameter {name} must contain {length} values")
        if not finite_values(vector):
            raise ValueError(f"Parameter {name} contains a non-finite value")
        return vector

    def get_bin_poses(self) -> dict[str, Pose]:
        bin_poses = {}
        for bin_id in self.bin_pose_defaults:
            bin_poses[bin_id] = pose_from_xyz_rpy(
                self.get_vector_parameter(f"{bin_id}_xyz", 3),
                self.get_vector_parameter(f"{bin_id}_rpy_deg", 3),
            )
        return bin_poses

    def get_item_bin_map_parameter(self, name: str) -> dict[str, str]:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            text = value.strip()
            parsed = json.loads(text) if text else {}
        else:
            parsed = value

        if not isinstance(parsed, dict):
            raise ValueError(f"Parameter {name} must be a JSON object")

        item_bin_map = {}
        for item_name, bin_id in parsed.items():
            normalized_bin_id = self.normalize_bin_id(str(bin_id))
            if normalized_bin_id not in self.bin_poses:
                raise ValueError(f"Item {item_name} maps to unknown bin {bin_id}")
            item_bin_map[str(item_name)] = normalized_bin_id
        return item_bin_map

    def set_sequence_step(
        self,
        step: str,
        step_label: str,
        step_index: int,
        step_total: int,
        state: str = "running",
    ) -> None:
        with self.sequence_lock:
            self.current_sequence_context.update(
                {
                    "state": state,
                    "step": step,
                    "step_label": step_label,
                    "step_index": step_index,
                    "step_total": step_total,
                }
            )

    def set_sequence_context(self, **fields: Any) -> None:
        with self.sequence_lock:
            self.current_sequence_context.update(fields)

    def clear_sequence_context(self) -> None:
        with self.sequence_lock:
            self.current_sequence_context.clear()

    def publish_status(self, message: str, **fields: Any) -> None:
        with self.sequence_lock:
            context = dict(self.current_sequence_context)
        payload = {
            "message": message,
            "level": fields.pop("level", "info"),
            "stamp_sec": self.get_clock().now().nanoseconds / 1e9,
            **context,
            **fields,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)
        self.get_logger().info(message)

    def joint_state_cb(self, msg: JointState) -> None:
        with self.sequence_lock:
            self.latest_joint_positions = {
                name: position
                for name, position in zip(msg.name, msg.position)
            }
            if "finger_width" in self.latest_joint_positions:
                self.latest_gripper_width = self.latest_joint_positions["finger_width"]

    def normalize_mode(self, value: str) -> str:
        text = value.strip().upper()
        mode_aliases = {
            "RUN": RUN_MODE,
            "RUNNING": RUN_MODE,
            "START": RUN_MODE,
            "GO": RUN_MODE,
            "TRUE": RUN_MODE,
            "1": RUN_MODE,
            "STOP": STOP_MODE,
            "STOPPED": STOP_MODE,
            "PAUSE": STOP_MODE,
            "PAUSED": STOP_MODE,
            "IDLE": STOP_MODE,
            "FALSE": STOP_MODE,
            "0": STOP_MODE,
        }
        if text not in mode_aliases:
            raise ValueError(
                "mode must be one of run/start/go or stop/pause/idle"
            )
        return mode_aliases[text]

    def normalize_orientation_mode(self, value: str) -> str:
        mode = value.strip().lower()
        if mode not in {AUTO_ORIENTATION_MODE, FIXED_ORIENTATION_MODE}:
            raise ValueError("orientation_mode must be auto or fixed")
        return mode

    def block_pose_cb(self, msg: PoseStamped) -> None:
        frame_id = msg.header.frame_id or self.frame_id
        place_pose, destination_id = self.default_destination()
        self.queue_sequence_request(
            block_id=self.default_block_id,
            block_pose=copy_pose(msg.pose),
            place_pose=place_pose,
            frame_id=frame_id,
            destination_id=destination_id,
            source="block_pose_topic",
        )

    def command_cb(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
            if not isinstance(command, dict):
                raise ValueError("command JSON must be an object")

            block_pose = self.parse_pose_from_command(command, "block_pose")
            if block_pose is None:
                block_pose = self.parse_pose_fields(command)

            place_pose, destination_id = self.destination_from_command(command)

            block_id = str(command.get("block_id", self.default_block_id))
            frame_id = str(command.get("frame_id", self.frame_id))

            self.queue_sequence_request(
                block_id,
                block_pose,
                place_pose,
                frame_id,
                destination_id,
                "command_topic",
            )
        except Exception as exc:
            self.publish_status(f"Ignoring pick-place command: {exc}")

    def mode_cb(self, msg: String) -> None:
        try:
            new_mode = self.normalize_mode(msg.data)
        except Exception as exc:
            self.publish_status(f"Ignoring mode command: {exc}")
            return

        mode_changed = False
        should_abort = False
        with self.sequence_lock:
            if new_mode != self.current_mode:
                self.current_mode = new_mode
                mode_changed = True
            if new_mode == STOP_MODE:
                self.pending_request = None
                should_abort = True

        if mode_changed:
            self.publish_status(
                "Pick-place mode updated",
                mode=self.current_mode.lower(),
            )

        if new_mode == RUN_MODE:
            self.try_start_pending_sequence()
        elif should_abort:
            self.abort_active_sequence("stop command")

    def destination_from_command(self, command: dict[str, Any]) -> tuple[Pose, str]:
        place_pose = self.parse_pose_from_command(command, "place_pose")
        if place_pose is None and "place" in command:
            place_pose = self.parse_pose_object(command["place"])
        if place_pose is not None:
            return place_pose, "place_pose"

        bin_id = self.bin_id_from_command(command)
        if bin_id:
            return self.bin_destination(bin_id)

        item_name = self.item_name_from_command(command)
        if item_name:
            if item_name not in self.item_bin_map:
                raise ValueError(f"item {item_name!r} is not mapped to a bin")
            return self.bin_destination(self.item_bin_map[item_name])

        return self.default_destination()

    def bin_id_from_command(self, command: dict[str, Any]) -> str:
        for key in ("bin_id", "bin", "destination_bin", "target_bin"):
            if key not in command:
                continue
            value = command[key]
            if isinstance(value, dict):
                value = value.get("id", value.get("bin_id", ""))
            return self.normalize_bin_id(str(value))
        return ""

    def item_name_from_command(self, command: dict[str, Any]) -> str:
        for key in ("item", "item_id", "item_type", "class", "label"):
            value = command.get(key)
            if value is not None:
                return str(value)
        return ""

    def normalize_bin_id(self, bin_id: str) -> str:
        text = bin_id.strip()
        if text in {"1", "2", "3", "4"}:
            return f"bin_{text}"
        return text

    def bin_destination(self, bin_id: str) -> tuple[Pose, str]:
        normalized_bin_id = self.normalize_bin_id(bin_id)
        if normalized_bin_id not in self.bin_poses:
            available = ", ".join(sorted(self.bin_poses.keys()))
            raise ValueError(
                f"unknown bin {bin_id!r}; available bins are {available}"
            )
        return copy_pose(self.bin_poses[normalized_bin_id]), normalized_bin_id

    def default_destination(self) -> tuple[Pose, str]:
        if self.default_bin_id:
            return self.bin_destination(self.default_bin_id)
        return copy_pose(self.default_place_pose), "place_xyz"

    def parse_pose_from_command(
        self, command: dict[str, Any], key: str
    ) -> Optional[Pose]:
        if key not in command:
            return None
        return self.parse_pose_object(command[key])

    def parse_pose_object(self, value: Any) -> Pose:
        if not isinstance(value, dict):
            raise ValueError("pose must be a JSON object")

        if "position" in value:
            position = value["position"]
            if isinstance(position, dict):
                xyz = [position["x"], position["y"], position["z"]]
            else:
                xyz = self.parse_vector(position, 3, "position")
        else:
            xyz = [value["x"], value["y"], value["z"]]

        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])

        if "orientation" in value:
            orientation = value["orientation"]
            pose.orientation.x = float(orientation.get("x", 0.0))
            pose.orientation.y = float(orientation.get("y", 0.0))
            pose.orientation.z = float(orientation.get("z", 0.0))
            pose.orientation.w = float(orientation.get("w", 1.0))
        elif "rpy_deg" in value:
            rpy_deg = self.parse_vector(value["rpy_deg"], 3, "rpy_deg")
            pose.orientation = pose_from_xyz_rpy([0.0, 0.0, 0.0], rpy_deg).orientation
        else:
            pose.orientation.w = 1.0

        if not finite_values(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        ):
            raise ValueError("pose contains a non-finite value")

        return pose

    def parse_pose_fields(self, command: dict[str, Any]) -> Pose:
        return self.parse_pose_object(command)

    def parse_vector(self, value: Any, length: int, field_name: str) -> list[float]:
        if isinstance(value, str):
            text = value.strip()
            if text.startswith("["):
                parsed = json.loads(text)
            else:
                parsed = [part for part in text.replace(",", " ").split() if part]
        else:
            parsed = value
        vector = [float(item) for item in parsed]
        if len(vector) != length:
            raise ValueError(f"{field_name} must contain {length} values")
        if not finite_values(vector):
            raise ValueError(f"{field_name} contains a non-finite value")
        return vector

    def queue_sequence_request(
        self,
        block_id: str,
        block_pose: Pose,
        place_pose: Pose,
        frame_id: str,
        destination_id: str,
        source: str,
    ) -> None:
        request = {
            "block_id": block_id,
            "block_pose": copy_pose(block_pose),
            "place_pose": copy_pose(place_pose),
            "frame_id": frame_id,
            "destination_id": destination_id,
            "source": source,
        }

        status_message = ""
        status_fields: dict[str, Any] = {}
        with self.sequence_lock:
            self.pending_request = request
            sequence_running = (
                self.sequence_thread is not None and self.sequence_thread.is_alive()
            )
            if self.current_mode != RUN_MODE:
                status_message = "Stored pick-place request while mode is stop"
            elif sequence_running:
                status_message = "Queued latest pick-place request while busy"
            else:
                status_message = ""

            status_fields = {
                "block_id": block_id,
                "destination": destination_id,
                "mode": self.current_mode.lower(),
                "source": source,
            }

        if status_message:
            self.publish_status(status_message, **status_fields)

        self.try_start_pending_sequence()

    def try_start_pending_sequence(self) -> bool:
        request: Optional[dict[str, Any]] = None
        with self.sequence_lock:
            sequence_running = (
                self.sequence_thread is not None and self.sequence_thread.is_alive()
            )
            if self.current_mode != RUN_MODE or sequence_running:
                return False
            if self.pending_request is None:
                return False

            request = self.pending_request
            self.pending_request = None
            self.abort_requested.clear()
            self.sequence_thread = threading.Thread(
                target=self.run_sequence,
                args=(
                    request["block_id"],
                    request["block_pose"],
                    request["place_pose"],
                    request["frame_id"],
                    request["destination_id"],
                ),
                daemon=True,
            )
            self.sequence_thread.start()
            return True

    def abort_active_sequence(self, reason: str) -> None:
        self.abort_requested.set()
        self.motion_failed.set()
        self.motion_complete.set()
        msg = String()
        msg.data = "stop"
        self.move_stop_pub.publish(msg)
        self.publish_hold_trajectories()
        self.publish_status(
            "Pick-place sequence aborted and reset",
            reason=reason,
            mode=self.current_mode.lower(),
        )

    def publish_hold_trajectories(self) -> None:
        joint_positions = None
        gripper_width = None
        with self.sequence_lock:
            if all(name in self.latest_joint_positions for name in self.home_joint_names):
                joint_positions = [
                    self.latest_joint_positions[name]
                    for name in self.home_joint_names
                ]
            gripper_width = self.latest_gripper_width

        if joint_positions is not None:
            trajectory = JointTrajectory()
            trajectory.joint_names = list(self.home_joint_names)
            point = JointTrajectoryPoint()
            point.positions = list(joint_positions)
            set_duration_seconds(point.time_from_start, self.stop_hold_sec)
            trajectory.points.append(point)
            self.arm_trajectory_pub.publish(trajectory)
        else:
            self.publish_status(
                "Could not publish arm hold trajectory: waiting for joint_states"
            )

        if gripper_width is not None:
            self.publish_gripper_trajectory(gripper_width, self.stop_hold_sec)

    def ensure_not_aborted(self, label: str = "pick-place sequence") -> None:
        if self.abort_requested.is_set() or self.current_mode != RUN_MODE:
            raise RuntimeError(f"{label} aborted")

    def tool_pose(
        self,
        source_pose: Pose,
        z: float,
        use_source_yaw: bool,
    ) -> Pose:
        yaw_deg = self.tool_yaw_offset_deg
        if use_source_yaw:
            yaw_deg += math.degrees(yaw_from_quaternion(source_pose.orientation))

        pose = Pose()
        pose.position.x = source_pose.position.x
        pose.position.y = source_pose.position.y
        pose.position.z = z
        pose.orientation = quaternion_from_rpy(
            math.radians(self.tool_roll_deg),
            math.radians(self.tool_pitch_deg),
            math.radians(yaw_deg),
        )
        return pose

    def run_sequence(
        self,
        block_id: str,
        block_pose: Pose,
        place_pose: Pose,
        frame_id: str,
        destination_id: str,
    ) -> None:
        sequence_steps = []
        if self.home_enabled and self.home_before_pick:
            sequence_steps.append(("home_before_pick", "move to pick-place home before pick"))
        sequence_steps.extend(
            [
                ("move_above_block", "move above block"),
                ("open_gripper", "open gripper"),
            ]
        )
        if self.remove_block_collision_before_grasp:
            sequence_steps.append(("remove_block_collision", f"remove {block_id} collision"))
        sequence_steps.extend(
            [
                ("move_down_to_block", "move down onto block"),
                ("close_gripper", "close gripper"),
                ("lift_block", "move back above block"),
                ("move_above_destination", f"move above {destination_id}"),
            ]
        )
        if self.place_descent_enabled:
            sequence_steps.append(("move_down_to_destination", f"move down to {destination_id}"))
        sequence_steps.append(("release_block", "drop block"))
        if self.add_block_at_place_after_release:
            sequence_steps.append(("add_block_collision", f"add {block_id} at place"))
        sequence_steps.append(("move_up_after_release", "move up after release"))
        if self.home_enabled and self.home_after_place:
            sequence_steps.append(("home_after_place", "move to pick-place home after place"))

        step_total = len(sequence_steps)
        step_index = 0

        def begin_step(step: str, label: str) -> None:
            nonlocal step_index
            step_index += 1
            self.set_sequence_step(step, label, step_index, step_total)

        try:
            self.ensure_not_aborted("pick-place sequence")
            self.set_sequence_context(
                state="starting",
                block_id=block_id,
                destination=destination_id,
                step="starting",
                step_label="Starting pick-place sequence",
                step_index=0,
                step_total=step_total,
            )
            self.publish_status(
                "Starting pick-place sequence",
                block_id=block_id,
                destination=destination_id,
                block_x=block_pose.position.x,
                block_y=block_pose.position.y,
                block_z=block_pose.position.z,
                place_x=place_pose.position.x,
                place_y=place_pose.position.y,
                place_z=place_pose.position.z,
            )

            pregrasp_pose = self.tool_pose(
                block_pose,
                block_pose.position.z + self.pregrasp_z_offset,
                self.use_block_yaw,
            )
            grasp_z = (
                self.grasp_z_absolute
                if self.grasp_z_absolute >= 0.0
                else block_pose.position.z + self.grasp_z_offset
            )
            grasp_pose = self.tool_pose(
                block_pose,
                grasp_z,
                self.use_block_yaw,
            )
            place_above_pose = self.tool_pose(
                place_pose,
                place_pose.position.z + self.place_above_z_offset,
                self.use_place_yaw,
            )
            place_down_pose = self.tool_pose(
                place_pose,
                place_pose.position.z + self.place_z_offset,
                self.use_place_yaw,
            )

            if self.home_enabled and self.home_before_pick:
                begin_step("home_before_pick", "move to pick-place home before pick")
                self.move_to_home("move to pick-place home before pick")

            self.ensure_not_aborted("pick-place sequence")
            begin_step("move_above_block", "move above block")
            self.move_to_pose(pregrasp_pose, "move above block")
            begin_step("open_gripper", "open gripper")
            self.send_gripper(self.open_width, "open gripper")

            self.ensure_not_aborted("pick-place sequence")
            if self.remove_block_collision_before_grasp:
                begin_step("remove_block_collision", f"remove {block_id} collision")
                self.remove_block_from_scene(block_id, frame_id)

            begin_step("move_down_to_block", "move down onto block")
            self.move_to_pose(grasp_pose, "move down onto block")
            begin_step("close_gripper", "close gripper")
            self.send_gripper(self.close_width, "close gripper")
            self.ensure_not_aborted("pick-place sequence")
            begin_step("lift_block", "move back above block")
            self.move_to_pose(pregrasp_pose, "move back above block")
            begin_step("move_above_destination", f"move above {destination_id}")
            self.move_to_pose(place_above_pose, f"move above {destination_id}")

            if self.place_descent_enabled:
                begin_step("move_down_to_destination", f"move down to {destination_id}")
                self.move_to_pose(place_down_pose, f"move down to {destination_id}")

            self.ensure_not_aborted("pick-place sequence")
            begin_step("release_block", "drop block")
            self.send_gripper(self.open_width, "drop block")

            self.ensure_not_aborted("pick-place sequence")
            if self.add_block_at_place_after_release:
                begin_step("add_block_collision", f"add {block_id} at place")
                final_block_pose = copy_pose(place_pose)
                self.add_block_to_scene(block_id, frame_id, final_block_pose)

            begin_step("move_up_after_release", "move up after release")
            self.move_to_pose(place_above_pose, "move up after release")
            if self.home_enabled and self.home_after_place:
                begin_step("home_after_place", "move to pick-place home after place")
                self.move_to_home("move to pick-place home after place")

            self.ensure_not_aborted("pick-place sequence")
            self.set_sequence_context(
                state="complete",
                step="complete",
                step_label="Pick-place sequence complete",
                step_index=step_total,
                step_total=step_total,
            )
            self.publish_status(
                "Pick-place sequence complete",
                block_id=block_id,
                destination=destination_id,
            )
        except Exception as exc:
            error_detail = traceback.format_exc()
            self.set_sequence_context(
                state="error",
                error=str(exc),
                error_detail=error_detail,
            )
            self.publish_status(
                "Pick-place sequence failed",
                level="error",
                block_id=block_id,
                destination=destination_id,
                error=str(exc),
                error_detail=error_detail,
            )
        finally:
            with self.sequence_lock:
                self.sequence_thread = None
                stopped = self.current_mode == STOP_MODE
            if not stopped:
                self.try_start_pending_sequence()

    def move_to_pose(self, pose: Pose, label: str) -> None:
        self.publish_status(
            label,
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
        )
        self.motion_complete.clear()
        self.motion_failed.clear()
        self.motion_started.clear()
        self.motion_waiting.set()
        self.last_failure_state = ""
        self.ensure_not_aborted(label)
        if self.orientation_mode == AUTO_ORIENTATION_MODE:
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z
            self.target_point_pub.publish(point)
        else:
            self.target_pose_pub.publish(pose)
        try:
            deadline = time.monotonic() + self.move_timeout_sec
            while time.monotonic() < deadline and rclpy.ok():
                self.ensure_not_aborted(label)
                if self.motion_complete.wait(timeout=0.1):
                    self.ensure_not_aborted(label)
                    return
                if self.motion_failed.is_set():
                    self.ensure_not_aborted(label)
                    raise RuntimeError(
                        f"{label} failed in coordinate listener: "
                        f"{self.last_failure_state}"
                    )

            raise RuntimeError(f"{label} timed out after {self.move_timeout_sec:.1f} s")
        finally:
            self.motion_waiting.clear()

    def move_to_home(self, label: str) -> None:
        self.publish_status(
            label,
            joint_names=self.home_joint_names,
            joint_positions_deg=self.home_joint_positions_deg,
        )
        self.motion_complete.clear()
        self.motion_failed.clear()
        self.motion_started.clear()
        self.motion_waiting.set()
        self.last_failure_state = ""
        self.ensure_not_aborted(label)

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.home_joint_names)
        joint_state.position = list(self.home_joint_positions_rad)
        self.target_joint_state_pub.publish(joint_state)

        try:
            deadline = time.monotonic() + self.move_timeout_sec
            while time.monotonic() < deadline and rclpy.ok():
                self.ensure_not_aborted(label)
                if self.motion_complete.wait(timeout=0.1):
                    self.ensure_not_aborted(label)
                    return
                if self.motion_failed.is_set():
                    self.ensure_not_aborted(label)
                    raise RuntimeError(
                        f"{label} failed in coordinate listener: "
                        f"{self.last_failure_state}"
                    )

            raise RuntimeError(f"{label} timed out after {self.move_timeout_sec:.1f} s")
        finally:
            self.motion_waiting.clear()

    def move_complete_cb(self, msg: String) -> None:
        _ = msg
        self.motion_complete.set()

    def move_state_cb(self, msg: String) -> None:
        state = msg.data.strip().upper()
        if state in ACTIVE_STATES:
            self.motion_started.set()
        if state in FAILURE_STATES and self.motion_waiting.is_set():
            self.last_failure_state = state
            self.motion_failed.set()

    def send_gripper(self, width: float, label: str) -> None:
        self.publish_status(label, width=width)
        self.ensure_not_aborted(label)
        self.publish_gripper_trajectory(width, self.gripper_motion_sec)
        deadline = time.monotonic() + max(0.0, self.gripper_motion_sec)
        while time.monotonic() < deadline:
            self.ensure_not_aborted(label)
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    def publish_gripper_trajectory(self, width: float, duration_sec: float) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = ["finger_width"]

        point = JointTrajectoryPoint()
        point.positions = [float(width)]
        set_duration_seconds(point.time_from_start, max(0.0, duration_sec))
        trajectory.points.append(point)

        self.gripper_pub.publish(trajectory)

    def remove_block_from_scene(self, block_id: str, frame_id: str) -> None:
        self.publish_workspace_command({"action": "remove", "id": block_id})

        collision_object = CollisionObject()
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.header.frame_id = frame_id
        collision_object.id = block_id
        collision_object.operation = CollisionObject.REMOVE
        self.apply_collision_objects([collision_object], f"remove {block_id}")

    def add_block_to_scene(self, block_id: str, frame_id: str, pose: Pose) -> None:
        command = {
            "action": "add_block",
            "id": block_id,
            "frame_id": frame_id,
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "size": self.block_size,
            "color": self.block_color,
        }
        self.publish_workspace_command(command)

        collision_object = CollisionObject()
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.header.frame_id = frame_id
        collision_object.id = block_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(self.block_size)
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(copy_pose(pose))
        collision_object.operation = CollisionObject.ADD
        self.apply_collision_objects([collision_object], f"add {block_id} at place")

    def publish_workspace_command(self, command: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(command, sort_keys=True)
        self.workspace_command_pub.publish(msg)

    def apply_collision_objects(
        self,
        collision_objects: Sequence[CollisionObject],
        description: str,
    ) -> bool:
        if not self.apply_scene_client.service_is_ready():
            if not self.apply_scene_client.wait_for_service(
                timeout_sec=self.scene_update_timeout_sec
            ):
                self.publish_status(
                    "MoveIt apply planning scene service is not available",
                    description=description,
                )
                return False

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.extend(collision_objects)

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_client.call_async(request)
        deadline = time.monotonic() + self.scene_update_timeout_sec
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.05)

        if not future.done():
            self.publish_status(
                "Timed out applying planning scene update",
                description=description,
            )
            return False

        try:
            response = future.result()
        except Exception as exc:
            self.publish_status(
                "MoveIt apply planning scene service raised an exception",
                description=description,
                error=str(exc),
            )
            return False
        if response is None or not response.success:
            self.publish_status(
                "MoveIt did not apply planning scene update",
                description=description,
            )
            return False

        self.publish_status("Planning scene updated", description=description)
        return True


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = PickPlaceSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
