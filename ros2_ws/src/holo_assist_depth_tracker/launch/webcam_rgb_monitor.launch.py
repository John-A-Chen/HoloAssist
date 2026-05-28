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
    use_sensor_data_qos_arg = DeclareLaunchArgument(
        "use_sensor_data_qos",
        default_value="true",
        description="Use sensor-data QoS (best-effort) for image and camera_info topics.",
    )
    qos_depth_arg = DeclareLaunchArgument(
        "qos_depth",
        default_value="5",
        description="QoS history depth for webcam topics.",
    )
    publish_camera_info_arg = DeclareLaunchArgument(
        "publish_camera_info",
        default_value="true",
        description="Publish synthetic CameraInfo alongside webcam images.",
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/holo_assist/webcam/camera_info",
        description="Webcam CameraInfo topic.",
    )
    camera_info_frame_id_arg = DeclareLaunchArgument(
        "camera_info_frame_id",
        default_value="webcam_frame",
        description="Frame id used in CameraInfo header.",
    )
    hfov_deg_arg = DeclareLaunchArgument(
        "hfov_deg",
        default_value="69.4",
        description="Approximate horizontal FOV (deg) used for auto intrinsics.",
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
                "use_sensor_data_qos": LaunchConfiguration("use_sensor_data_qos"),
                "qos_depth": LaunchConfiguration("qos_depth"),
                "publish_camera_info": LaunchConfiguration("publish_camera_info"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "camera_info_frame_id": LaunchConfiguration("camera_info_frame_id"),
                "hfov_deg": LaunchConfiguration("hfov_deg"),
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
            use_sensor_data_qos_arg,
            qos_depth_arg,
            publish_camera_info_arg,
            camera_info_topic_arg,
            camera_info_frame_id_arg,
            hfov_deg_arg,
            bind_host_arg,
            bind_port_arg,
            webcam_node,
            relay_node,
        ]
    )
