"""Eye-to-hand calibration launch.

sim mode (default):
  Starts fake robot + MoveIt + calibration node.  No real hardware needed.
  The node auto-generates random poses, computes synthetic camera observations,
  solves, and publishes the result.

hardware mode:
  Starts only the calibration node (assumes robot driver, MoveIt, camera, and
  apriltag_ros are already running in other terminals).

Usage:
  # Sim (everything self-contained):
  ros2 launch holo_assist_depth_tracker eye_hand_calibration.launch.py

  # Hardware (other terminals already running robot + camera + apriltag):
  ros2 launch holo_assist_depth_tracker eye_hand_calibration.launch.py mode:=hardware
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _build_sim_robot(context, *args, **kwargs):
    """Spin up a fake-hardware UR3e + controllers (only in sim mode)."""
    mode = LaunchConfiguration("mode").perform(context)
    if mode != "sim":
        return []

    ur_type = "ur3e"
    onrobot_type = "rg2"

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("ur_onrobot_description"), "urdf", "ur_onrobot.urdf.xacro"]
    )
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ", xacro_file, " ",
        "ur_type:=", ur_type, " ",
        "onrobot_type:=", onrobot_type, " ",
        "name:=ur_onrobot", " ",
        "use_fake_hardware:=true", " ",
        "robot_ip:=0.0.0.0",
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    ur_update_rate = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "config", ur_type + "_update_rate.yaml"]
    )

    # Try to use sim_controllers.yaml from moveit_robot_control if available
    try:
        from ament_index_python.packages import get_package_share_directory
        ctrl_cfg = PathJoinSubstitution(
            [FindPackageShare("moveit_robot_control"), "config", "sim_controllers.yaml"]
        )
    except Exception:
        ctrl_cfg = PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_control"), "config", "ur_onrobot_controllers.yaml"]
        )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, ur_update_rate, ctrl_cfg],
    )
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "15",
            "joint_state_broadcaster",
            "joint_trajectory_controller",
        ],
        output="screen",
    )
    return [rsp, control_node, spawner]


def _build_moveit(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    start = LaunchConfiguration("start_moveit").perform(context)
    if mode != "sim" or start.lower() != "true":
        return []

    moveit_pkg = FindPackageShare("ur_onrobot_moveit_config")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([moveit_pkg, "launch", "ur_onrobot_moveit.launch.py"])
            ),
            launch_arguments={
                "ur_type": "ur3e",
                "onrobot_type": "rg2",
                "use_fake_hardware": "true",
                "launch_rviz": "false",
                "launch_servo": "false",
            }.items(),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    perc_pkg = FindPackageShare("holo_assist_depth_tracker")

    mode_arg = DeclareLaunchArgument("mode", default_value="sim")
    samples_arg = DeclareLaunchArgument("num_samples", default_value="20")
    start_moveit_arg = DeclareLaunchArgument("start_moveit", default_value="true")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [perc_pkg, "config", "eye_hand_calibration_params.yaml"]
        ),
    )

    robot_stack = OpaqueFunction(function=_build_sim_robot)
    moveit_stack = OpaqueFunction(function=_build_moveit)

    calibration_node = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_eye_hand_calibration",
        name="eye_hand_calibration",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "mode": LaunchConfiguration("mode"),
                "num_samples": LaunchConfiguration("num_samples"),
            },
        ],
    )

    rviz_config = PathJoinSubstitution(
        [perc_pkg, "config", "eye_hand_calibration.rviz"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="eye_hand_cal_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        mode_arg,
        samples_arg,
        start_moveit_arg,
        use_rviz_arg,
        params_arg,
        LogInfo(msg=["[eye_hand_cal] Mode: ", LaunchConfiguration("mode")]),
        # t=0: robot stack (sim only)
        robot_stack,
        moveit_stack,
        # t=5: calibration node (wait for controllers to be ready)
        TimerAction(period=5.0, actions=[
            LogInfo(msg="[eye_hand_cal] Starting calibration node..."),
            calibration_node,
        ]),
        # t=6: RViz
        TimerAction(period=6.0, actions=[rviz]),
    ])
