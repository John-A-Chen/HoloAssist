from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bind_host_arg = DeclareLaunchArgument(
        "bind_host",
        default_value="0.0.0.0",
        description="HTTP bind host for dashboard relay.",
    )
    bind_port_arg = DeclareLaunchArgument(
        "bind_port",
        default_value="8765",
        description="HTTP bind port for dashboard relay.",
    )
    stale_timeout_arg = DeclareLaunchArgument(
        "stale_timeout_s",
        default_value="1.5",
        description="Tracker stale timeout (seconds).",
    )
    max_image_fps_arg = DeclareLaunchArgument(
        "max_image_fps",
        default_value="8.0",
        description="Maximum JPEG refresh rate served to dashboard.",
    )
    jpeg_quality_arg = DeclareLaunchArgument(
        "jpeg_quality",
        default_value="80",
        description="JPEG quality (40..95).",
    )
    enable_rgb_fallback_arg = DeclareLaunchArgument(
        "enable_rgb_fallback",
        default_value="true",
        description="Enable RGB image fallback mode when depth tracker is unavailable.",
    )
    fallback_rgb_topic_arg = DeclareLaunchArgument(
        "fallback_rgb_topic",
        default_value="/image_raw",
        description="Raw RGB topic used in fallback mode.",
    )
    enable_robot_state_arg = DeclareLaunchArgument(
        "enable_robot_state",
        default_value="true",
        description="Enable robot telemetry subscriptions (/joint_states, TF, command topics).",
    )
    joint_states_topic_arg = DeclareLaunchArgument(
        "joint_states_topic",
        default_value="/joint_states",
        description="Joint state topic.",
    )
    target_pose_topic_arg = DeclareLaunchArgument(
        "target_pose_topic",
        default_value="/servo_target_pose",
        description="Target pose / goal command topic.",
    )
    twist_topic_arg = DeclareLaunchArgument(
        "twist_topic",
        default_value="/servo_node/delta_twist_cmds",
        description="Twist command topic.",
    )
    clicked_point_topic_arg = DeclareLaunchArgument(
        "clicked_point_topic",
        default_value="/clicked_point",
        description="Clicked point topic used for click-to-plan.",
    )
    command_frame_arg = DeclareLaunchArgument(
        "command_frame",
        default_value="base_link",
        description="Reference frame for end-effector pose lookup.",
    )
    eef_frame_arg = DeclareLaunchArgument(
        "eef_frame",
        default_value="tool0",
        description="End-effector frame for TF lookup.",
    )
    unity_tcp_host_arg = DeclareLaunchArgument(
        "unity_tcp_host",
        default_value="127.0.0.1",
        description="Host used to probe Unity ROS-TCP endpoint.",
    )
    unity_tcp_port_arg = DeclareLaunchArgument(
        "unity_tcp_port",
        default_value="10000",
        description="Port used to probe Unity ROS-TCP endpoint.",
    )
    unity_map_loaded_topic_arg = DeclareLaunchArgument(
        "unity_map_loaded_topic",
        default_value="/unity/map_loaded",
        description="Optional Bool topic signalling Unity map/scene readiness.",
    )

    relay_node = Node(
        package="holo_assist_depth_tracker",
        executable="holo_assist_depth_tracker_dashboard_relay",
        name="holo_assist_depth_tracker_dashboard_relay",
        output="screen",
        parameters=[
            {
                "bind_host": LaunchConfiguration("bind_host"),
                "bind_port": LaunchConfiguration("bind_port"),
                "stale_timeout_s": LaunchConfiguration("stale_timeout_s"),
                "max_image_fps": LaunchConfiguration("max_image_fps"),
                "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                "enable_rgb_fallback": LaunchConfiguration("enable_rgb_fallback"),
                "fallback_rgb_topic": LaunchConfiguration("fallback_rgb_topic"),
                "enable_robot_state": LaunchConfiguration("enable_robot_state"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                "twist_topic": LaunchConfiguration("twist_topic"),
                "clicked_point_topic": LaunchConfiguration("clicked_point_topic"),
                "command_frame": LaunchConfiguration("command_frame"),
                "eef_frame": LaunchConfiguration("eef_frame"),
                "unity_tcp_host": LaunchConfiguration("unity_tcp_host"),
                "unity_tcp_port": LaunchConfiguration("unity_tcp_port"),
                "unity_map_loaded_topic": LaunchConfiguration("unity_map_loaded_topic"),
            }
        ],
    )

    return LaunchDescription(
        [
            bind_host_arg,
            bind_port_arg,
            stale_timeout_arg,
            max_image_fps_arg,
            jpeg_quality_arg,
            enable_rgb_fallback_arg,
            fallback_rgb_topic_arg,
            enable_robot_state_arg,
            joint_states_topic_arg,
            target_pose_topic_arg,
            twist_topic_arg,
            clicked_point_topic_arg,
            command_frame_arg,
            eef_frame_arg,
            unity_tcp_host_arg,
            unity_tcp_port_arg,
            unity_map_loaded_topic_arg,
            relay_node,
        ]
    )
