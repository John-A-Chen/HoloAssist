import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
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
    start_overlay_arg = DeclareLaunchArgument(
        "start_overlay",
        default_value="true",
        description=(
            "Start RGB overlay node that draws AprilTag detections onto the image."
        ),
    )
    overlay_output_topic_arg = DeclareLaunchArgument(
        "overlay_output_topic",
        default_value="/holoassist/apriltag/debug_image",
        description="Output image topic for AprilTag overlay debug stream.",
    )
    detections_topic_arg = DeclareLaunchArgument(
        "detections_topic",
        default_value="/detections",
        description="AprilTag detection topic to subscribe for overlays.",
    )
    workspace_outline_enabled_arg = DeclareLaunchArgument(
        "workspace_outline_enabled",
        default_value="true",
        description="Draw workspace quadrilateral on RGB overlay using tag geometry.",
    )
    workspace_left_tag_id_arg = DeclareLaunchArgument(
        "workspace_left_tag_id",
        default_value="0",
        description="Tag ID at front-left board corner.",
    )
    workspace_right_tag_id_arg = DeclareLaunchArgument(
        "workspace_right_tag_id",
        default_value="1",
        description="Tag ID at front-right board corner.",
    )
    workspace_width_m_arg = DeclareLaunchArgument(
        "workspace_width_m",
        default_value="0.70",
        description="Workspace board width in meters (front edge span).",
    )
    workspace_depth_m_arg = DeclareLaunchArgument(
        "workspace_depth_m",
        default_value="0.50",
        description="Workspace board depth in meters.",
    )
    workspace_tag_size_m_arg = DeclareLaunchArgument(
        "workspace_tag_size_m",
        default_value="0.15",
        description="Physical tag size in meters (including white border if printed that way).",
    )
    workspace_left_corner_index_offset_arg = DeclareLaunchArgument(
        "workspace_left_corner_index_offset",
        default_value="0",
        description="Corner index offset for left tag overlay mapping (0..3).",
    )
    workspace_right_corner_index_offset_arg = DeclareLaunchArgument(
        "workspace_right_corner_index_offset",
        default_value="0",
        description="Corner index offset for right tag overlay mapping (0..3).",
    )
    workspace_left_corner_reverse_arg = DeclareLaunchArgument(
        "workspace_left_corner_reverse",
        default_value="false",
        description="Reverse corner order for left tag overlay mapping.",
    )
    workspace_right_corner_reverse_arg = DeclareLaunchArgument(
        "workspace_right_corner_reverse",
        default_value="false",
        description="Reverse corner order for right tag overlay mapping.",
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

    apriltag_overlay_node = Node(
        package="holoassist_foxglove",
        executable="apriltag_image_overlay_node",
        name="apriltag_image_overlay",
        output="screen",
        parameters=[
            {
                "input_image_topic": LaunchConfiguration("image_topic"),
                "detections_topic": LaunchConfiguration("detections_topic"),
                "output_image_topic": LaunchConfiguration("overlay_output_topic"),
                "workspace_outline_enabled": LaunchConfiguration(
                    "workspace_outline_enabled"
                ),
                "workspace_left_tag_id": LaunchConfiguration("workspace_left_tag_id"),
                "workspace_right_tag_id": LaunchConfiguration("workspace_right_tag_id"),
                "workspace_width_m": LaunchConfiguration("workspace_width_m"),
                "workspace_depth_m": LaunchConfiguration("workspace_depth_m"),
                "workspace_tag_size_m": LaunchConfiguration("workspace_tag_size_m"),
                "workspace_left_corner_index_offset": LaunchConfiguration(
                    "workspace_left_corner_index_offset"
                ),
                "workspace_right_corner_index_offset": LaunchConfiguration(
                    "workspace_right_corner_index_offset"
                ),
                "workspace_left_corner_reverse": LaunchConfiguration(
                    "workspace_left_corner_reverse"
                ),
                "workspace_right_corner_reverse": LaunchConfiguration(
                    "workspace_right_corner_reverse"
                ),
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_overlay")),
    )

    return LaunchDescription(
        [
            start_camera_arg,
            image_topic_arg,
            camera_info_topic_arg,
            params_file_arg,
            start_overlay_arg,
            overlay_output_topic_arg,
            detections_topic_arg,
            workspace_outline_enabled_arg,
            workspace_left_tag_id_arg,
            workspace_right_tag_id_arg,
            workspace_width_m_arg,
            workspace_depth_m_arg,
            workspace_tag_size_m_arg,
            workspace_left_corner_index_offset_arg,
            workspace_right_corner_index_offset_arg,
            workspace_left_corner_reverse_arg,
            workspace_right_corner_reverse_arg,
            start_realsense,
            apriltag_node,
            apriltag_overlay_node,
        ]
    )
