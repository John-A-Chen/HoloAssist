from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_click_to_plan = LaunchConfiguration('enable_click_to_plan')

    ros_ip = LaunchConfiguration('ros_ip')
    ros_tcp_port = LaunchConfiguration('ros_tcp_port')

    command_frame = LaunchConfiguration('command_frame')
    eef_frame = LaunchConfiguration('eef_frame')
    target_topic = LaunchConfiguration('target_topic')
    twist_topic = LaunchConfiguration('twist_topic')
    k_lin = LaunchConfiguration('k_lin')
    max_lin = LaunchConfiguration('max_lin')

    move_group_name = LaunchConfiguration('move_group_name')
    planning_frame = LaunchConfiguration('planning_frame')
    tcp_frame = LaunchConfiguration('tcp_frame')
    tcp_yaw_rad = LaunchConfiguration('tcp_yaw_rad')

    return LaunchDescription([
        DeclareLaunchArgument('enable_click_to_plan', default_value='true'),
        DeclareLaunchArgument('ros_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('ros_tcp_port', default_value='10000'),
        DeclareLaunchArgument('command_frame', default_value='base_link'),
        DeclareLaunchArgument('eef_frame', default_value='tool0'),
        DeclareLaunchArgument('target_topic', default_value='/servo_target_pose'),
        DeclareLaunchArgument('twist_topic', default_value='/servo_node/delta_twist_cmds'),
        DeclareLaunchArgument('k_lin', default_value='1.5'),
        DeclareLaunchArgument('max_lin', default_value='0.25'),
        DeclareLaunchArgument('move_group_name', default_value='ur_manipulator'),
        DeclareLaunchArgument('planning_frame', default_value='base_link'),
        DeclareLaunchArgument('tcp_frame', default_value='tool0'),
        DeclareLaunchArgument('tcp_yaw_rad', default_value='0.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('holoassist_unity_bridge'),
                    'launch',
                    'tcp_endpoint.launch.py',
                ])
            ),
            launch_arguments={
                'ros_ip': ros_ip,
                'ros_tcp_port': ros_tcp_port,
            }.items(),
        ),
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
        Node(
            condition=IfCondition(enable_click_to_plan),
            package='holoassist_manipulation',
            executable='clicked_point_to_moveit',
            name='clicked_point_to_moveit',
            output='screen',
            parameters=[{
                'move_group_name': move_group_name,
                'planning_frame': planning_frame,
                'tcp_frame': tcp_frame,
                'tcp_yaw_rad': tcp_yaw_rad,
            }],
        ),
    ])
