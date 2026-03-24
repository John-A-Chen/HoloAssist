from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    command_frame = LaunchConfiguration('command_frame')
    eef_frame = LaunchConfiguration('eef_frame')
    target_topic = LaunchConfiguration('target_topic')
    twist_topic = LaunchConfiguration('twist_topic')
    k_lin = LaunchConfiguration('k_lin')
    max_lin = LaunchConfiguration('max_lin')

    return LaunchDescription([
        DeclareLaunchArgument('command_frame', default_value='base_link'),
        DeclareLaunchArgument('eef_frame', default_value='tool0'),
        DeclareLaunchArgument('target_topic', default_value='/servo_target_pose'),
        DeclareLaunchArgument('twist_topic', default_value='/servo_node/delta_twist_cmds'),
        DeclareLaunchArgument('k_lin', default_value='1.5'),
        DeclareLaunchArgument('max_lin', default_value='0.25'),
        Node(
            package='holoassist_servo_tools',
            executable='pose_to_twist_servo',
            name='pose_to_twist_servo',
            output='screen',
            parameters=[{
                'command_frame': command_frame,
                'eef_frame': eef_frame,
                'target_topic': target_topic,
                'twist_topic': twist_topic,
                'k_lin': k_lin,
                'max_lin': max_lin,
            }],
        ),
    ])
