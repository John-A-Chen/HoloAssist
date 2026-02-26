from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("move_group_name", default_value="ur_manipulator"),
            DeclareLaunchArgument("planning_frame", default_value="base_link"),
            DeclareLaunchArgument("tcp_frame", default_value="tool0"),
            Node(
                package="holoassist_manipulation",
                executable="clicked_point_to_moveit",
                name="clicked_point_to_moveit",
                output="screen",
                parameters=[
                    {
                        "move_group_name": LaunchConfiguration("move_group_name"),
                        "planning_frame": LaunchConfiguration("planning_frame"),
                        "tcp_frame": LaunchConfiguration("tcp_frame"),
                    }
                ],
            ),
        ]
    )
