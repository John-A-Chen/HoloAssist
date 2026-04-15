from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    enable_unity_bringup_arg = DeclareLaunchArgument(
        "enable_unity_bringup",
        default_value="true",
        description="Start Unity ROS-TCP endpoint and servo/click-to-plan stack.",
    )
    enable_click_to_plan_arg = DeclareLaunchArgument(
        "enable_click_to_plan",
        default_value="true",
        description="Enable clicked_point MoveIt planning helper.",
    )
    ros_ip_arg = DeclareLaunchArgument(
        "ros_ip",
        default_value="0.0.0.0",
        description="ROS TCP endpoint bind host for Unity bridge.",
    )
    ros_tcp_port_arg = DeclareLaunchArgument(
        "ros_tcp_port",
        default_value="10000",
        description="ROS TCP endpoint bind port for Unity bridge.",
    )
    command_frame_arg = DeclareLaunchArgument(
        "command_frame",
        default_value="base_link",
        description="Command frame for pose-to-twist servoing.",
    )
    eef_frame_arg = DeclareLaunchArgument(
        "eef_frame",
        default_value="tool0",
        description="End-effector TF frame for pose-to-twist servoing.",
    )

    enable_depth_tracker_arg = DeclareLaunchArgument(
        "enable_depth_tracker",
        default_value="false",
        description="Start depth tracker node (without RViz by default).",
    )
    enable_depth_camera_arg = DeclareLaunchArgument(
        "enable_depth_camera",
        default_value="false",
        description="Start RealSense camera launch along with depth tracker.",
    )
    enable_pointcloud_obstacle_arg = DeclareLaunchArgument(
        "enable_pointcloud_obstacle",
        default_value="false",
        description="Start pointcloud_to_moveit_obstacle node for MoveIt scene updates.",
    )
    enable_ur3_keyboard_teleop_arg = DeclareLaunchArgument(
        "enable_ur3_keyboard_teleop",
        default_value="false",
        description="Start ur3_keyboard_teleop node.",
    )
    enable_ur3_joint_controller_arg = DeclareLaunchArgument(
        "enable_ur3_joint_controller",
        default_value="false",
        description="Start ur3_joint_position_controller node.",
    )
    enable_robot_demo_arg = DeclareLaunchArgument(
        "enable_robot_demo",
        default_value="false",
        description="Start holoassist_movement robot demo runner node.",
    )

    enable_foxglove_observability_arg = DeclareLaunchArgument(
        "enable_foxglove_observability",
        default_value="true",
        description="Enable holoassist_foxglove observability stack.",
    )
    enable_foxglove_bridge_arg = DeclareLaunchArgument(
        "enable_foxglove_bridge",
        default_value="true",
        description="Enable foxglove_bridge launch from observability stack.",
    )
    enable_manager_arg = DeclareLaunchArgument(
        "enable_manager",
        default_value="true",
        description="Enable holoassist_manager node from observability stack.",
    )
    enable_tf_marker_bridge_arg = DeclareLaunchArgument(
        "enable_tf_marker_bridge",
        default_value="false",
        description="Enable tf_marker_bridge from observability stack.",
    )

    unity_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("holoassist_unity_bridge"),
                    "launch",
                    "unity_movement_bringup.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("enable_unity_bringup")),
        launch_arguments={
            "enable_click_to_plan": LaunchConfiguration("enable_click_to_plan"),
            "ros_ip": LaunchConfiguration("ros_ip"),
            "ros_tcp_port": LaunchConfiguration("ros_tcp_port"),
            "command_frame": LaunchConfiguration("command_frame"),
            "eef_frame": LaunchConfiguration("eef_frame"),
            "enable_foxglove_observability": "false",
            "enable_foxglove_bridge": LaunchConfiguration("enable_foxglove_bridge"),
            "enable_manager": LaunchConfiguration("enable_manager"),
            "enable_tf_marker_bridge": LaunchConfiguration("enable_tf_marker_bridge"),
        }.items(),
    )

    observability = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("holoassist_foxglove"),
                    "launch",
                    "observability.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("enable_foxglove_observability")),
        launch_arguments={
            "enable_foxglove_bridge": LaunchConfiguration("enable_foxglove_bridge"),
            "enable_manager": LaunchConfiguration("enable_manager"),
            "enable_tf_marker_bridge": LaunchConfiguration("enable_tf_marker_bridge"),
            "unity_tcp_port": LaunchConfiguration("ros_tcp_port"),
        }.items(),
    )

    depth_tracker = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("holo_assist_depth_tracker"),
                    "launch",
                    "visualize_depth_tracker.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("enable_depth_tracker")),
        launch_arguments={
            "start_camera": LaunchConfiguration("enable_depth_camera"),
            "start_tracker": "true",
            "start_rviz": "false",
        }.items(),
    )

    pointcloud_obstacle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("holoassist_manipulation"),
                    "launch",
                    "pointcloud_to_moveit_obstacle.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("enable_pointcloud_obstacle")),
        launch_arguments={
            "planning_frame": LaunchConfiguration("command_frame"),
        }.items(),
    )

    ur3_keyboard_teleop = Node(
        package="ur3_keyboard_teleop",
        executable="keyboard_joint_teleop",
        name="ur3_keyboard_teleop",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_ur3_keyboard_teleop")),
    )

    ur3_joint_controller = Node(
        package="ur3_joint_position_controller",
        executable="ur3_joint_position_controller",
        name="ur3_joint_position_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_ur3_joint_controller")),
    )

    robot_demo = Node(
        package="holoassist_movement",
        executable="robot_demo_control",
        name="robot_demo_control",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_robot_demo")),
    )

    return LaunchDescription(
        [
            enable_unity_bringup_arg,
            enable_click_to_plan_arg,
            ros_ip_arg,
            ros_tcp_port_arg,
            command_frame_arg,
            eef_frame_arg,
            enable_depth_tracker_arg,
            enable_depth_camera_arg,
            enable_pointcloud_obstacle_arg,
            enable_ur3_keyboard_teleop_arg,
            enable_ur3_joint_controller_arg,
            enable_robot_demo_arg,
            enable_foxglove_observability_arg,
            enable_foxglove_bridge_arg,
            enable_manager_arg,
            enable_tf_marker_bridge_arg,
            unity_bringup,
            observability,
            depth_tracker,
            pointcloud_obstacle,
            ur3_keyboard_teleop,
            ur3_joint_controller,
            robot_demo,
        ]
    )
