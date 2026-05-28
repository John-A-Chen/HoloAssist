from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("holo_assist_depth_tracker")
    rviz_pkg_share = FindPackageShare("ur_onrobot_description")

    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz", default_value="true",
        description="Open RViz2 with debug image display.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([rviz_pkg_share, "rviz", "view_robot.rviz"]),
    )

    # Camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "camera_only.launch.py"])
        ),
    )

    # apriltag + cube_pose + tracker overlay (no second camera launch)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "holoassist_4tag_board_4cube.launch.py"])
        ),
        launch_arguments={
            "start_camera": "false",
            "start_workspace": "false",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return LaunchDescription([
        start_rviz_arg,
        rviz_config_arg,
        camera_launch,
        perception_launch,
        rviz_node,
    ])
