from __future__ import annotations

from typing import Dict

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


VALID_TRUE = {"1", "true", "yes", "on"}
VALID_FALSE = {"0", "false", "no", "off"}


def _resolve_bool(raw: str, default: bool) -> bool:
    v = raw.strip().lower()
    if v in ("", "auto"):
        return default
    if v in VALID_TRUE:
        return True
    if v in VALID_FALSE:
        return False
    raise ValueError(f"Expected boolean or 'auto', got '{raw}'")


def _profile_defaults(profile: str) -> Dict[str, bool]:
    if profile == "full_hardware":
        return {
            "enable_observability": True,
            "enable_foxglove_bridge": True,
            "enable_manager": True,
            "enable_tf_marker_bridge": False,
            "enable_object_pose_adapter": True,
            "enable_apriltag_tracking": False,
            "enable_workspace_perception": True,
            "enable_depth_tracker": True,
            "enable_depth_camera": True,
            "enable_pointcloud_obstacle": True,
            "enable_unity_endpoint": True,
            "enable_unity_bringup": True,
            "enable_click_to_plan": True,
            "enable_ur3_keyboard_teleop": True,
            "enable_ur3_joint_controller": True,
            "enable_robot_demo": False,
        }

    # Default profile: software-safe startup before robot/hardware are connected.
    return {
        "enable_observability": True,
        "enable_foxglove_bridge": True,
        "enable_manager": True,
        "enable_tf_marker_bridge": False,
        "enable_object_pose_adapter": True,
        "enable_apriltag_tracking": False,
        "enable_workspace_perception": False,
        "enable_depth_tracker": False,
        "enable_depth_camera": False,
        "enable_pointcloud_obstacle": False,
        "enable_unity_endpoint": False,
        "enable_unity_bringup": False,
        "enable_click_to_plan": False,
        "enable_ur3_keyboard_teleop": False,
        "enable_ur3_joint_controller": False,
        "enable_robot_demo": False,
    }


def _setup(context):
    profile = LaunchConfiguration("profile").perform(context).strip()
    if profile not in {"no_hardware", "full_hardware"}:
        raise RuntimeError(
            "Invalid profile '%s'. Valid values: no_hardware, full_hardware." % profile
        )

    defaults = _profile_defaults(profile)

    resolved: Dict[str, bool] = {}
    for key, default in defaults.items():
        resolved[key] = _resolve_bool(LaunchConfiguration(key).perform(context), default)

    object_pose_adapter_raw = (
        LaunchConfiguration("enable_object_pose_adapter").perform(context).strip().lower()
    )
    object_pose_adapter_explicit = object_pose_adapter_raw not in ("", "auto")
    if resolved["enable_workspace_perception"] and not object_pose_adapter_explicit:
        # Workspace perception publishes /holoassist/perception/object_pose directly.
        # Disable legacy obstacle->object adapter unless user explicitly forces it on.
        resolved["enable_object_pose_adapter"] = False

    ros_ip = LaunchConfiguration("ros_ip")
    ros_tcp_port = LaunchConfiguration("ros_tcp_port")
    command_frame = LaunchConfiguration("command_frame")
    eef_frame = LaunchConfiguration("eef_frame")
    object_workspace_frame = LaunchConfiguration("object_workspace_frame")
    workspace_perception_params_file = LaunchConfiguration("workspace_perception_params_file")
    apriltag_start_camera = LaunchConfiguration("apriltag_start_camera")
    apriltag_image_topic = LaunchConfiguration("apriltag_image_topic")
    apriltag_camera_info_topic = LaunchConfiguration("apriltag_camera_info_topic")
    apriltag_params_file = LaunchConfiguration("apriltag_params_file")

    runtime_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("holoassist_foxglove"), "launch", "holoassist_foxglove_runtime.launch.py"]
            )
        ),
        launch_arguments={
            "enable_unity_bringup": str(resolved["enable_unity_bringup"]).lower(),
            "enable_click_to_plan": str(resolved["enable_click_to_plan"]).lower(),
            "ros_ip": ros_ip,
            "ros_tcp_port": ros_tcp_port,
            "command_frame": command_frame,
            "eef_frame": eef_frame,
            "enable_depth_tracker": str(resolved["enable_depth_tracker"]).lower(),
            "enable_depth_camera": str(resolved["enable_depth_camera"]).lower(),
            "enable_workspace_perception": str(resolved["enable_workspace_perception"]).lower(),
            "workspace_perception_params_file": workspace_perception_params_file,
            "enable_pointcloud_obstacle": str(resolved["enable_pointcloud_obstacle"]).lower(),
            "enable_ur3_keyboard_teleop": str(resolved["enable_ur3_keyboard_teleop"]).lower(),
            "enable_ur3_joint_controller": str(resolved["enable_ur3_joint_controller"]).lower(),
            "enable_robot_demo": str(resolved["enable_robot_demo"]).lower(),
            "enable_foxglove_observability": str(resolved["enable_observability"]).lower(),
            "enable_foxglove_bridge": str(resolved["enable_foxglove_bridge"]).lower(),
            "enable_manager": str(resolved["enable_manager"]).lower(),
            "enable_tf_marker_bridge": str(resolved["enable_tf_marker_bridge"]).lower(),
            "enable_object_pose_adapter": str(resolved["enable_object_pose_adapter"]).lower(),
            "object_workspace_frame": object_workspace_frame,
        }.items(),
    )

    actions = [
        LogInfo(
            msg=(
                "[holoassist_stack] profile=%s observability=%s unity_bringup=%s "
                "unity_endpoint=%s depth_tracker=%s depth_camera=%s pointcloud_obstacle=%s "
                "apriltag_tracking=%s workspace_perception=%s object_pose_adapter=%s"
            )
            % (
                profile,
                resolved["enable_observability"],
                resolved["enable_unity_bringup"],
                resolved["enable_unity_endpoint"],
                resolved["enable_depth_tracker"],
                resolved["enable_depth_camera"],
                resolved["enable_pointcloud_obstacle"],
                resolved["enable_apriltag_tracking"],
                resolved["enable_workspace_perception"],
                resolved["enable_object_pose_adapter"],
            )
        ),
        runtime_launch,
    ]

    # unity_movement_bringup already includes tcp_endpoint.launch.py.
    # Only include standalone endpoint when requested and bringup is disabled.
    if resolved["enable_unity_endpoint"] and not resolved["enable_unity_bringup"]:
        unity_endpoint_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("holoassist_unity_bridge"), "launch", "tcp_endpoint.launch.py"]
                )
            ),
            launch_arguments={
                "ros_ip": ros_ip,
                "ros_tcp_port": ros_tcp_port,
            }.items(),
        )
        actions.append(unity_endpoint_launch)
    elif resolved["enable_unity_endpoint"] and resolved["enable_unity_bringup"]:
        actions.append(
            LogInfo(
                msg=(
                    "[holoassist_stack] unity endpoint requested but unity bringup is enabled; "
                    "skipping standalone endpoint include to avoid duplicate server startup."
                )
            )
        )

    if resolved["enable_apriltag_tracking"]:
        apriltag_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("holoassist_foxglove"), "launch", "apriltag_workbench.launch.py"]
                )
            ),
            launch_arguments={
                "start_camera": apriltag_start_camera,
                "image_topic": apriltag_image_topic,
                "camera_info_topic": apriltag_camera_info_topic,
                "params_file": apriltag_params_file,
            }.items(),
        )
        actions.append(apriltag_launch)

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="no_hardware",
                description="Stack profile: no_hardware (default) or full_hardware.",
            ),
            DeclareLaunchArgument("enable_observability", default_value="auto"),
            DeclareLaunchArgument("enable_foxglove_bridge", default_value="auto"),
            DeclareLaunchArgument("enable_manager", default_value="auto"),
            DeclareLaunchArgument("enable_tf_marker_bridge", default_value="auto"),
            DeclareLaunchArgument("enable_object_pose_adapter", default_value="auto"),
            DeclareLaunchArgument("enable_apriltag_tracking", default_value="auto"),
            DeclareLaunchArgument("enable_workspace_perception", default_value="auto"),
            DeclareLaunchArgument("enable_depth_tracker", default_value="auto"),
            DeclareLaunchArgument("enable_depth_camera", default_value="auto"),
            DeclareLaunchArgument("enable_pointcloud_obstacle", default_value="auto"),
            DeclareLaunchArgument("enable_unity_endpoint", default_value="auto"),
            DeclareLaunchArgument("enable_unity_bringup", default_value="auto"),
            DeclareLaunchArgument("enable_click_to_plan", default_value="auto"),
            DeclareLaunchArgument("enable_ur3_keyboard_teleop", default_value="auto"),
            DeclareLaunchArgument("enable_ur3_joint_controller", default_value="auto"),
            DeclareLaunchArgument("enable_robot_demo", default_value="auto"),
            DeclareLaunchArgument("ros_ip", default_value="0.0.0.0"),
            DeclareLaunchArgument("ros_tcp_port", default_value="10000"),
            DeclareLaunchArgument("command_frame", default_value="base_link"),
            DeclareLaunchArgument("eef_frame", default_value="tool0"),
            DeclareLaunchArgument("object_workspace_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "workspace_perception_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("holo_assist_depth_tracker"),
                        "config",
                        "workspace_perception_params.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("apriltag_start_camera", default_value="false"),
            DeclareLaunchArgument(
                "apriltag_image_topic",
                default_value="/camera/color/image_raw",
            ),
            DeclareLaunchArgument(
                "apriltag_camera_info_topic",
                default_value="/camera/color/camera_info",
            ),
            DeclareLaunchArgument(
                "apriltag_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("holoassist_foxglove"),
                        "config",
                        "apriltag_workbench_36h11.yaml",
                    ]
                ),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
