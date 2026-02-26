from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            name="unity_tcp_endpoint",
            output="screen",
            parameters=[{
                "ROS_IP": "0.0.0.0",
                "ROS_TCP_PORT": 10000
            }]
        )
    ])