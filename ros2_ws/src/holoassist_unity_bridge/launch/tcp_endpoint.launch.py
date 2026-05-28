from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ros_ip = LaunchConfiguration('ros_ip')
    ros_tcp_port = LaunchConfiguration('ros_tcp_port')

    return LaunchDescription([
        DeclareLaunchArgument('ros_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('ros_tcp_port', default_value='10000'),
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='unity_tcp_endpoint',
            output='screen',
            parameters=[{
                'ROS_IP': ros_ip,
                'ROS_TCP_PORT': ros_tcp_port,
            }],
        ),
    ])
