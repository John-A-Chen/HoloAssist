from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    duration = LaunchConfiguration('duration')

    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='7.0'),
        Node(
            package='holoassist_movement',
            executable='robot_demo_control',
            name='robot_demo_control',
            output='screen',
            arguments=[
                '--duration', duration,
            ],
        ),
    ])
