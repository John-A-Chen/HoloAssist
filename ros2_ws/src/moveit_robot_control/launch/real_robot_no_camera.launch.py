from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    full_hardware_launch = PathJoinSubstitution(
        [
            FindPackageShare("moveit_robot_control"),
            "launch",
            "full_holoassist_hardware.launch.py",
        ]
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
    use_fake_gripper_hardware_arg = DeclareLaunchArgument(
        "use_fake_gripper_hardware",
        default_value="true",
        description="Keep the UR arm real but use fake OnRobot gripper hardware.",
    )
    robot_base_yaw_rad_arg = DeclareLaunchArgument(
        "robot_base_yaw_rad",
        default_value="3.14159",
    )
    controller_spawner_timeout_arg = DeclareLaunchArgument(
        "controller_spawner_timeout",
        default_value="60",
    )

    full_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(full_hardware_launch),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "reverse_ip": LaunchConfiguration("reverse_ip"),
            "start_camera": "false",
            "start_perception": "false",
            "start_rosbridge": "false",
            "start_selected_cube_adapter": "false",
            "use_fake_gripper_hardware": LaunchConfiguration("use_fake_gripper_hardware"),
            "allow_pose_goal_fallback": "false",
            "pick_place_orientation_mode": "fixed",
            "use_rviz": LaunchConfiguration("use_rviz"),
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

    return LaunchDescription(
        [
            robot_ip_arg,
            reverse_ip_arg,
            use_rviz_arg,
            velocity_scale_arg,
            cartesian_retime_velocity_scale_arg,
            use_fake_gripper_hardware_arg,
            robot_base_yaw_rad_arg,
            controller_spawner_timeout_arg,
            full_hardware,
        ]
    )
