import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    start_camera_arg = DeclareLaunchArgument(
        "start_camera",
        default_value="false",
        description="Start RealSense camera before launching AprilTag detector.",
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/camera/color/image_raw",
        description="Image topic remapped into apriltag_ros image_rect input.",
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/camera/color/camera_info",
        description="CameraInfo topic remapped into apriltag_ros camera_info input.",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        description="Path to apriltag_ros parameters YAML.",
    )

    try:
        realsense_launch_path = os.path.join(
            get_package_share_directory("realsense2_camera"),
            "launch",
            "rs_launch.py",
        )
        start_realsense = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path),
            condition=IfCondition(LaunchConfiguration("start_camera")),
        )
    except PackageNotFoundError:
        start_realsense = LogInfo(
            condition=IfCondition(LaunchConfiguration("start_camera")),
            msg=(
                "[apriltag_workbench] realsense2_camera not found. "
                "Install ros-humble-realsense2-camera or set start_camera:=false."
            ),
        )

    try:
        get_package_share_directory("apriltag_ros")
        apriltag_node = Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
            remappings=[
                ("image_rect", LaunchConfiguration("image_topic")),
                ("camera_info", LaunchConfiguration("camera_info_topic")),
            ],
        )
    except PackageNotFoundError:
        apriltag_node = LogInfo(
            msg=(
                "[apriltag_workbench] apriltag_ros not found. "
                "Install ros-humble-apriltag ros-humble-apriltag-ros."
            ),
        )

    return LaunchDescription(
        [
            start_camera_arg,
            image_topic_arg,
            camera_info_topic_arg,
            params_file_arg,
            start_realsense,
            apriltag_node,
        ]
    )
