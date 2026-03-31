from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ur3_joint_position_controller",
                executable="ur3_joint_position_controller",
                name="ur3_joint_position_controller",
                output="screen",
            )
        ]
    )
