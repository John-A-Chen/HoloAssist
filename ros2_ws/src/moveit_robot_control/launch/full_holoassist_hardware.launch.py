import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """MoveIt-only launch — robot driver + perception provided by launch.sh --perception."""
    moveit_robot_control_pkg = FindPackageShare("moveit_robot_control")
    moveit_pkg = FindPackageShare("ur_onrobot_moveit_config")
    robot_description_pkg = FindPackageShare("ur_onrobot_description")

    hw_config_default = PathJoinSubstitution(
        [moveit_robot_control_pkg, "config", "full_holoassist_hw.yaml"]
    )
    rviz_default = PathJoinSubstitution(
        [moveit_robot_control_pkg, "rviz", "holoassist_hw.rviz"]
    )

    moveit_launch = PathJoinSubstitution(
        [moveit_pkg, "launch", LaunchConfiguration("moveit_launch_file")]
    )
    coordinate_listener_launch = PathJoinSubstitution(
        [moveit_robot_control_pkg, "launch", "coordinate_listener.launch.py"]
    )
    robot_xacro = PathJoinSubstitution(
        [robot_description_pkg, "urdf", "ur_onrobot.urdf.xacro"]
    )
    srdf_xacro = PathJoinSubstitution(
        [moveit_pkg, "srdf", "ur_onrobot.srdf.xacro"]
    )

    # ── Declared arguments ───────────────────────────────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        description="IP address of the UR3e robot (used for xacro only, driver already running).",
    )
    ur_type_arg = DeclareLaunchArgument("ur_type", default_value="ur3e")
    onrobot_type_arg = DeclareLaunchArgument("onrobot_type", default_value="rg2")
    use_fake_gripper_hardware_arg = DeclareLaunchArgument(
        "use_fake_gripper_hardware",
        default_value="false",
        description=(
            "Describe the OnRobot gripper as fake hardware while keeping the UR arm real."
        ),
    )
    robot_base_yaw_rad_arg = DeclareLaunchArgument(
        "robot_base_yaw_rad",
        default_value="0.0",
    )
    kinematics_config_arg = DeclareLaunchArgument(
        "kinematics_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_description"), "config", "ur3e_calibration.yaml"]
        ),
    )

    moveit_launch_file_arg = DeclareLaunchArgument(
        "moveit_launch_file",
        default_value="ur_onrobot_moveit.launch.py",
    )
    move_group_name_arg = DeclareLaunchArgument(
        "move_group_name", default_value="ur_onrobot_manipulator"
    )
    ee_link_arg = DeclareLaunchArgument("ee_link", default_value="gripper_tcp")
    frame_arg = DeclareLaunchArgument("frame", default_value="base_link")

    hw_config_arg = DeclareLaunchArgument(
        "hw_config",
        default_value=hw_config_default,
    )

    velocity_scale_arg = DeclareLaunchArgument(
        "velocity_scale",
        default_value="0.05",
    )
    cartesian_retime_velocity_scale_arg = DeclareLaunchArgument(
        "cartesian_retime_velocity_scale",
        default_value="0.0",
        description=(
            "Optional cap used only when retiming straight Cartesian paths. "
            "Set 0.0 to follow velocity_scale."
        ),
    )
    orientation_mode_arg = DeclareLaunchArgument(
        "orientation_mode", default_value="auto"
    )
    avoid_flange_forearm_clamp_arg = DeclareLaunchArgument(
        "avoid_flange_forearm_clamp", default_value="true"
    )
    pose_goal_planning_time_arg = DeclareLaunchArgument(
        "pose_goal_planning_time", default_value="5.0"
    )

    start_pick_place_arg = DeclareLaunchArgument(
        "start_pick_place",
        default_value="true",
    )
    grasp_z_absolute_arg = DeclareLaunchArgument(
        "grasp_z_absolute",
        default_value="0.05",
        description=(
            "Absolute Z height for the pick descent pose. "
            "Use -1.0 to fall back to cube_z + grasp_z_offset."
        ),
    )
    cube_pose_topic_prefix_arg = DeclareLaunchArgument(
        "cube_pose_topic_prefix",
        default_value="/holoassist/perception",
        description="Prefix for april_cube_N_pose topics used by the pick-place service.",
    )

    trajectory_topic_arg = DeclareLaunchArgument(
        "trajectory_topic",
        default_value="/scaled_joint_trajectory_controller/joint_trajectory",
        description="JointTrajectory topic for MoveIt execution. "
                    "Use /joint_trajectory_controller/joint_trajectory for fake hardware.",
    )
    arm_trajectory_topic_arg = DeclareLaunchArgument(
        "arm_trajectory_topic",
        default_value="/scaled_joint_trajectory_controller/joint_trajectory",
        description="JointTrajectory topic for the pick-place sequencer. "
                    "Use /joint_trajectory_controller/joint_trajectory for fake hardware.",
    )
    require_robot_status_arg = DeclareLaunchArgument(
        "require_robot_status",
        default_value="true",
        description="Set false for fake hardware (no UR driver status topics).",
    )
    require_controller_check_arg = DeclareLaunchArgument(
        "require_controller_check",
        default_value="true",
        description="Set false for fake hardware.",
    )

    use_calibrated_workspace_arg = DeclareLaunchArgument(
        "use_calibrated_workspace",
        default_value="false",
    )
    calibration_yaml_arg = DeclareLaunchArgument(
        "calibration_yaml",
        default_value=os.path.expanduser("~/.holoassist/calibration/calibration_latest.yaml"),
    )

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value=rviz_default)

    rviz_robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    robot_xacro,
                    " ",
                    "robot_ip:=", LaunchConfiguration("robot_ip"),
                    " ",
                    "ur_type:=", LaunchConfiguration("ur_type"),
                    " ",
                    "onrobot_type:=", LaunchConfiguration("onrobot_type"),
                    " ",
                    "name:=ur_onrobot",
                    " ",
                    "use_fake_hardware:=false",
                    " ",
                    "use_fake_gripper_hardware:=", LaunchConfiguration("use_fake_gripper_hardware"),
                    " ",
                    "base_yaw_rad:=", LaunchConfiguration("robot_base_yaw_rad"),
                    " ",
                    "kinematics_parameters_file:=", LaunchConfiguration("kinematics_config"),
                ]
            ),
            value_type=str,
        )
    }
    rviz_robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    srdf_xacro,
                    " ",
                    "name:=ur_onrobot",
                    " ",
                    "prefix:=",
                    "",
                ]
            ),
            value_type=str,
        )
    }

    # ── MoveIt: move_group + OMPL + SRDF ─────────────────────────────────────
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "onrobot_type": LaunchConfiguration("onrobot_type"),
            "use_fake_hardware": "false",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "launch_rviz": "false",
            "launch_servo": "false",
            "base_yaw_rad": LaunchConfiguration("robot_base_yaw_rad"),
            "use_fake_gripper_hardware": LaunchConfiguration("use_fake_gripper_hardware"),
        }.items(),
    )

    # ── workspace_frame static TF broadcaster (calibrated mode only) ──────────
    workspace_frame_tf = Node(
        package="moveit_robot_control",
        executable="workspace_frame_tf",
        name="workspace_frame_tf",
        output="screen",
        parameters=[LaunchConfiguration("calibration_yaml")],
        condition=IfCondition(LaunchConfiguration("use_calibrated_workspace")),
    )

    # ── Workspace collision scene (trolley mesh visual) ───────────────────────
    workspace_scene = Node(
        package="moveit_robot_control",
        executable="workspace_scene_manager",
        name="workspace_scene_manager",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("hw_config")],
    )

    # ── Coordinate listener: topic-driven MoveIt goals → trajectory execution ─
    coordinate_listener = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(coordinate_listener_launch),
        launch_arguments={
            "move_group_name": LaunchConfiguration("move_group_name"),
            "ee_link": LaunchConfiguration("ee_link"),
            "frame": LaunchConfiguration("frame"),
            "require_robot_status": LaunchConfiguration("require_robot_status"),
            "require_controller_check": LaunchConfiguration("require_controller_check"),
            "allow_pose_goal_fallback": "true",
            "orientation_mode": LaunchConfiguration("orientation_mode"),
            "avoid_flange_forearm_clamp": LaunchConfiguration("avoid_flange_forearm_clamp"),
            "pose_goal_planning_time": LaunchConfiguration("pose_goal_planning_time"),
            "velocity_scale": LaunchConfiguration("velocity_scale"),
            "cartesian_retime_velocity_scale": LaunchConfiguration("cartesian_retime_velocity_scale"),
            "trajectory_topic": LaunchConfiguration("trajectory_topic"),
        }.items(),
    )

    # ── Pick-and-place sequencer ──────────────────────────────────────────────
    pick_place_sequencer = Node(
        package="moveit_robot_control",
        executable="pick_place_sequencer",
        name="pick_place_sequencer",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_pick_place")),
        parameters=[{
            "initial_mode": "run",
            "orientation_mode": "auto",
            "pregrasp_z_offset": 0.10,
            "grasp_z_offset": 0.0,
            "grasp_z_absolute": LaunchConfiguration("grasp_z_absolute"),
            "place_above_z_offset": 0.15,
            "place_z_offset": 0.05,
            "place_descent_enabled": True,
            "arm_trajectory_topic": LaunchConfiguration("arm_trajectory_topic"),
        }],
    )

    # ── Pick-cube-to-bin service ──────────────────────────────────────────────
    pick_place_service = Node(
        package="holo_assist_depth_tracker_sim",
        executable="pick_place_service_node",
        name="holoassist_pick_place_service",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_pick_place")),
        parameters=[{"cube_pose_topic_prefix": LaunchConfiguration("cube_pose_topic_prefix")}],
    )

    # ── Selected-cube → MoveIt target adapter ────────────────────────────────
    selected_cube_adapter = Node(
        package="holo_assist_depth_tracker_sim",
        executable="selected_cube_to_moveit_target_node",
        name="holoassist_selected_cube_to_moveit_target",
        output="screen",
        parameters=[
            LaunchConfiguration("hw_config"),
            {
                "target_frame": LaunchConfiguration("frame"),
                "output_point_topic": "/moveit_robot_control/target_point",
                "output_pose_topic": "/moveit_robot_control/target_pose",
            },
        ],
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="holoassist_moveit_hw_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            rviz_robot_description,
            rviz_robot_description_semantic,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # Startup sequencing (robot driver + perception already running via launch.sh):
    #  t=0s   moveit_stack   — move_group waits for controller_manager + joint_states
    #  t=0s   workspace_frame_tf
    #  t=5s   workspace_scene — needs TF to be up
    #  t=8s   coordinate_listener + pick_place_sequencer
    #  t=10s  pick_place_service + selected_cube_adapter
    #  t=12s  RViz

    return LaunchDescription(
        [
            robot_ip_arg,
            ur_type_arg,
            onrobot_type_arg,
            use_fake_gripper_hardware_arg,
            robot_base_yaw_rad_arg,
            kinematics_config_arg,
            moveit_launch_file_arg,
            move_group_name_arg,
            ee_link_arg,
            frame_arg,
            hw_config_arg,
            velocity_scale_arg,
            cartesian_retime_velocity_scale_arg,
            orientation_mode_arg,
            avoid_flange_forearm_clamp_arg,
            pose_goal_planning_time_arg,
            start_pick_place_arg,
            grasp_z_absolute_arg,
            cube_pose_topic_prefix_arg,
            trajectory_topic_arg,
            arm_trajectory_topic_arg,
            require_robot_status_arg,
            require_controller_check_arg,
            use_calibrated_workspace_arg,
            calibration_yaml_arg,
            use_rviz_arg,
            rviz_config_arg,
            moveit_stack,
            workspace_frame_tf,
            TimerAction(period=5.0, actions=[workspace_scene]),
            TimerAction(period=8.0, actions=[coordinate_listener, pick_place_sequencer]),
            TimerAction(period=10.0, actions=[pick_place_service, selected_cube_adapter]),
            TimerAction(period=12.0, actions=[
                LogInfo(msg="[hw] t=12s: launching RViz"),
                rviz,
            ]),
        ]
    )
