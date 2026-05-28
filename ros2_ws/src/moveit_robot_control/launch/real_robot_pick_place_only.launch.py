from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    moveit_robot_control_pkg = FindPackageShare("moveit_robot_control")
    bin_config_default = PathJoinSubstitution(
        [moveit_robot_control_pkg, "config", "bin_poses.json"]
    )

    frame_arg = DeclareLaunchArgument("frame_id", default_value="base_link")
    cube_pose_topic_prefix_arg = DeclareLaunchArgument(
        "cube_pose_topic_prefix",
        default_value="/holoassist/perception",
        description="Prefix for real April cube PoseStamped topics.",
    )
    bin_config_path_arg = DeclareLaunchArgument(
        "bin_config_path",
        default_value=bin_config_default,
    )
    initial_mode_arg = DeclareLaunchArgument(
        "initial_mode",
        default_value="run",
        description="Initial pick-place mode: run or stop.",
    )
    orientation_mode_arg = DeclareLaunchArgument(
        "orientation_mode",
        default_value="auto",
    )
    pregrasp_z_offset_arg = DeclareLaunchArgument(
        "pregrasp_z_offset",
        default_value="0.10",
    )
    grasp_z_offset_arg = DeclareLaunchArgument(
        "grasp_z_offset",
        default_value="0.0",
    )
    grasp_z_absolute_arg = DeclareLaunchArgument(
        "grasp_z_absolute",
        default_value="0.05",
        description=(
            "Absolute Z height for the pick descent pose. "
            "Use -1.0 to fall back to cube_z + grasp_z_offset."
        ),
    )
    place_above_z_offset_arg = DeclareLaunchArgument(
        "place_above_z_offset",
        default_value="0.15",
    )
    place_z_offset_arg = DeclareLaunchArgument(
        "place_z_offset",
        default_value="0.05",
    )
    place_descent_enabled_arg = DeclareLaunchArgument(
        "place_descent_enabled",
        default_value="true",
    )
    open_width_arg = DeclareLaunchArgument("open_width", default_value="0.08")
    close_width_arg = DeclareLaunchArgument("close_width", default_value="0.0")

    pick_place_sequencer = Node(
        package="moveit_robot_control",
        executable="pick_place_sequencer",
        name="pick_place_sequencer",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "frame_id": LaunchConfiguration("frame_id"),
                "command_topic": "/pick_place/command",
                "mode_topic": "/pick_place/mode",
                "status_topic": "/pick_place/status",
                "target_pose_topic": "/moveit_robot_control/target_pose",
                "target_point_topic": "/moveit_robot_control/target_point",
                "target_joint_state_topic": "/moveit_robot_control/target_joint_state",
                "move_state_topic": "/moveit_robot_control/state",
                "move_complete_topic": "/moveit_robot_control/complete",
                "move_stop_topic": "/moveit_robot_control/stop",
                "joint_states_topic": "/joint_states",
                "arm_trajectory_topic": "/scaled_joint_trajectory_controller/joint_trajectory",
                "gripper_topic": "/finger_width_trajectory_controller/joint_trajectory",
                "workspace_command_topic": "/workspace_scene/command",
                "apply_planning_scene_service": "/apply_planning_scene",
                "bin_config_path": LaunchConfiguration("bin_config_path"),
                "initial_mode": LaunchConfiguration("initial_mode"),
                "orientation_mode": LaunchConfiguration("orientation_mode"),
                "pregrasp_z_offset": ParameterValue(
                    LaunchConfiguration("pregrasp_z_offset"),
                    value_type=float,
                ),
                "grasp_z_offset": ParameterValue(
                    LaunchConfiguration("grasp_z_offset"),
                    value_type=float,
                ),
                "grasp_z_absolute": ParameterValue(
                    LaunchConfiguration("grasp_z_absolute"),
                    value_type=float,
                ),
                "place_above_z_offset": ParameterValue(
                    LaunchConfiguration("place_above_z_offset"),
                    value_type=float,
                ),
                "place_z_offset": ParameterValue(
                    LaunchConfiguration("place_z_offset"),
                    value_type=float,
                ),
                "place_descent_enabled": ParameterValue(
                    LaunchConfiguration("place_descent_enabled"),
                    value_type=bool,
                ),
                "open_width": ParameterValue(
                    LaunchConfiguration("open_width"),
                    value_type=float,
                ),
                "close_width": ParameterValue(
                    LaunchConfiguration("close_width"),
                    value_type=float,
                ),
            }
        ],
    )

    pick_place_service = Node(
        package="holo_assist_depth_tracker_sim",
        executable="pick_place_service_node",
        name="holoassist_pick_place_service",
        output="screen",
        parameters=[
            {"cube_pose_topic_prefix": LaunchConfiguration("cube_pose_topic_prefix")}
        ],
    )

    return LaunchDescription(
        [
            frame_arg,
            cube_pose_topic_prefix_arg,
            bin_config_path_arg,
            initial_mode_arg,
            orientation_mode_arg,
            pregrasp_z_offset_arg,
            grasp_z_offset_arg,
            grasp_z_absolute_arg,
            place_above_z_offset_arg,
            place_z_offset_arg,
            place_descent_enabled_arg,
            open_width_arg,
            close_width_arg,
            pick_place_sequencer,
            pick_place_service,
        ]
    )
