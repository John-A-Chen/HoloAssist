from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    moveit_robot_control_pkg = FindPackageShare("moveit_robot_control")
    sim_pkg = FindPackageShare("holo_assist_depth_tracker_sim")

    full_hardware_launch = PathJoinSubstitution(
        [
            moveit_robot_control_pkg,
            "launch",
            "full_holoassist_hardware.launch.py",
        ]
    )
    sim_perception_launch = PathJoinSubstitution(
        [sim_pkg, "launch", "sim_april_cube_perception.launch.py"]
    )

    sim_visual_config_default = PathJoinSubstitution(
        [moveit_robot_control_pkg, "config", "full_holoassist_sim.yaml"]
    )
    sim_scene_default = PathJoinSubstitution([sim_pkg, "config", "sim_scene.yaml"])
    sim_camera_default = PathJoinSubstitution([sim_pkg, "config", "sim_camera.yaml"])
    sim_cubes_default = PathJoinSubstitution([sim_pkg, "config", "sim_cubes.yaml"])
    sim_rviz_default = PathJoinSubstitution(
        [sim_pkg, "rviz", "holoassist_moveit_full.rviz"]
    )

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.0.197",
        description="IP address of the UR3e robot.",
    )
    reverse_ip_arg = DeclareLaunchArgument(
        "reverse_ip",
        default_value="192.168.0.52",
        description="IP address the robot uses to connect back to this PC.",
    )
    headless_mode_arg = DeclareLaunchArgument(
        "headless_mode",
        default_value="false",
        description=(
            "Let the UR driver send the External Control program automatically. "
            "Useful for URSim/headless bringup; keep false when using the pendant."
        ),
    )
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    velocity_scale_arg = DeclareLaunchArgument("velocity_scale", default_value="0.02")
    cartesian_retime_velocity_scale_arg = DeclareLaunchArgument(
        "cartesian_retime_velocity_scale",
        default_value="0.0",
        description=(
            "Optional cap used only when retiming straight Cartesian paths. "
            "Set 0.0 to follow velocity_scale."
        ),
    )
    robot_base_yaw_rad_arg = DeclareLaunchArgument(
        "robot_base_yaw_rad",
        default_value="0.0",
        description="0.0 matches full_holoassist_moveit_sim.launch.py visually.",
    )
    controller_spawner_timeout_arg = DeclareLaunchArgument(
        "controller_spawner_timeout",
        default_value="60",
    )
    start_pick_place_arg = DeclareLaunchArgument(
        "start_pick_place",
        default_value="false",
        description=(
            "Keep false for a visual-only sim scene. Set true only when the "
            "sim workspace intentionally matches the physical table."
        ),
    )
    start_sim_perception_arg = DeclareLaunchArgument(
        "start_sim_perception",
        default_value="true",
        description="Publish the same fake cube/camera markers used by the MoveIt sim.",
    )
    use_fake_gripper_hardware_arg = DeclareLaunchArgument(
        "use_fake_gripper_hardware",
        default_value="true",
        description=(
            "Keep the UR arm real but use fake gripper hardware so this view "
            "does not require the OnRobot /tmp/ttyUR tool bridge."
        ),
    )
    sim_visual_config_arg = DeclareLaunchArgument(
        "sim_visual_config",
        default_value=sim_visual_config_default,
        description="Workspace TF and trolley scene config matching the MoveIt sim.",
    )
    sim_scene_arg = DeclareLaunchArgument(
        "sim_scene_config",
        default_value=sim_scene_default,
    )
    sim_camera_arg = DeclareLaunchArgument(
        "sim_camera_config",
        default_value=sim_camera_default,
    )
    sim_cubes_arg = DeclareLaunchArgument(
        "sim_cubes_config",
        default_value=sim_cubes_default,
    )
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value=sim_rviz_default)

    full_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(full_hardware_launch),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "reverse_ip": LaunchConfiguration("reverse_ip"),
            "headless_mode": LaunchConfiguration("headless_mode"),
            "start_camera": "false",
            "start_perception": "false",
            "start_rosbridge": "false",
            "start_pick_place": LaunchConfiguration("start_pick_place"),
            "cube_pose_topic_prefix": "/holoassist/sim/truth",
            "start_selected_cube_adapter": "false",
            "use_fake_gripper_hardware": LaunchConfiguration(
                "use_fake_gripper_hardware"
            ),
            "allow_pose_goal_fallback": "false",
            "pick_place_orientation_mode": "fixed",
            "use_calibrated_workspace": "false",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "hw_config": LaunchConfiguration("sim_visual_config"),
            "robot_base_yaw_rad": LaunchConfiguration("robot_base_yaw_rad"),
            "velocity_scale": LaunchConfiguration("velocity_scale"),
            "cartesian_retime_velocity_scale": LaunchConfiguration(
                "cartesian_retime_velocity_scale"
            ),
            "controller_spawner_timeout": LaunchConfiguration(
                "controller_spawner_timeout"
            ),
        }.items(),
    )

    workspace_tf = Node(
        package="moveit_robot_control",
        executable="workspace_frame_tf",
        name="holoassist_workspace_frame_tf",
        output="screen",
        parameters=[LaunchConfiguration("sim_visual_config")],
    )

    sim_perception = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_perception_launch),
                condition=IfCondition(LaunchConfiguration("start_sim_perception")),
                launch_arguments={
                    "use_rviz": "false",
                    "use_sim_time": "false",
                    "sim_scene_config": LaunchConfiguration("sim_scene_config"),
                    "sim_camera_config": LaunchConfiguration("sim_camera_config"),
                    "sim_cubes_config": LaunchConfiguration("sim_cubes_config"),
                    "publish_scene_state_publisher": "false",
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            robot_ip_arg,
            reverse_ip_arg,
            headless_mode_arg,
            use_rviz_arg,
            velocity_scale_arg,
            cartesian_retime_velocity_scale_arg,
            robot_base_yaw_rad_arg,
            controller_spawner_timeout_arg,
            start_pick_place_arg,
            start_sim_perception_arg,
            use_fake_gripper_hardware_arg,
            sim_visual_config_arg,
            sim_scene_arg,
            sim_camera_arg,
            sim_cubes_arg,
            rviz_config_arg,
            full_hardware,
            TimerAction(period=2.0, actions=[workspace_tf]),
            TimerAction(period=5.0, actions=[sim_perception]),
        ]
    )
