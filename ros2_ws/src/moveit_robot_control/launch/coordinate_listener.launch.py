from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="moveit_robot_control",
                executable="coordinate_listener",
                name="moveit_robot_control",
                output="screen",
                emulate_tty=True,
            )
        ]
    )
