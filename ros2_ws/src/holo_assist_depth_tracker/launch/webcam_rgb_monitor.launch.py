from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/holo_assist/webcam/image_raw",
        description="Webcam raw image topic.",
    )
    device_index_arg = DeclareLaunchArgument(
        "device_index",
        default_value="0",
        description="OpenCV webcam device index.",
    )
    fps_arg = DeclareLaunchArgument(
        "fps",
        default_value="15.0",
        description="Target webcam frame rate.",
    )
    width_arg = DeclareLaunchArgument(
        "width",
        default_value="640",
        description="Target webcam width.",
    )
    height_arg = DeclareLaunchArgument(
        "height",
        default_value="480",
        description="Target webcam height.",
    )
    bind_host_arg = DeclareLaunchArgument(
        "bind_host",
        default_value="0.0.0.0",
        description="Dashboard relay bind host.",
    )
    bind_port_arg = DeclareLaunchArgument(
        "bind_port",
        default_value="8765",
        description="Dashboard relay bind port.",
    )

    webcam_node = Node(
        package="holo_assist_depth_tracker",
        executable="holo_assist_webcam_image_publisher",
        name="holo_assist_webcam_image_publisher",
        output="screen",
        parameters=[
            {
                "device_index": LaunchConfiguration("device_index"),
                "image_topic": LaunchConfiguration("image_topic"),
                "fps": LaunchConfiguration("fps"),
                "width": LaunchConfiguration("width"),
                "height": LaunchConfiguration("height"),
            }
        ],
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
                "enable_rgb_fallback": True,
                "fallback_rgb_topic": LaunchConfiguration("image_topic"),
            }
        ],
    )

    return LaunchDescription(
        [
            image_topic_arg,
            device_index_arg,
            fps_arg,
            width_arg,
            height_arg,
            bind_host_arg,
            bind_port_arg,
            webcam_node,
            relay_node,
        ]
    )
