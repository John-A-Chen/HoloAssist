from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ur3_keyboard_teleop",
                executable="keyboard_joint_teleop",
                name="ur3_keyboard_teleop",
                output="screen",
                emulate_tty=True,
            )
        ]
    )
