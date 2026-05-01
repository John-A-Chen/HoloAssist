import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    enable_foxglove_bridge_arg = DeclareLaunchArgument(
        "enable_foxglove_bridge",
        default_value="true",
        description="Start foxglove_bridge alongside HoloAssist runtime observability.",
    )
    enable_manager_arg = DeclareLaunchArgument(
        "enable_manager",
        default_value="true",
        description="Start holoassist_manager mode/diagnostics node.",
    )
    enable_tf_marker_bridge_arg = DeclareLaunchArgument(
        "enable_tf_marker_bridge",
        default_value="false",
        description="Start tf_marker_bridge marker publisher from holoassist_manager.",
    )
    enable_object_pose_adapter_arg = DeclareLaunchArgument(
        "enable_object_pose_adapter",
        default_value="true",
        description=(
            "Start object pose adapter that maps obstacle markers into "
            "/holoassist/perception/object_pose topics."
        ),
    )
    object_workspace_frame_arg = DeclareLaunchArgument(
        "object_workspace_frame",
        default_value="base_link",
        description=(
            "Target frame used for /holoassist/perception/object_pose_workspace."
        ),
    )
    diagnostics_rate_hz_arg = DeclareLaunchArgument(
        "diagnostics_rate_hz",
        default_value="1.0",
        description="Diagnostic publish rate for runtime_observability_node.",
    )
    stale_timeout_s_arg = DeclareLaunchArgument(
        "stale_timeout_s",
        default_value="2.5",
        description="Topic freshness timeout used for diagnostics state transitions.",
    )
    unity_tcp_host_arg = DeclareLaunchArgument(
        "unity_tcp_host",
        default_value="127.0.0.1",
        description="Unity TCP endpoint host to probe for network health metrics.",
    )
    unity_tcp_port_arg = DeclareLaunchArgument(
        "unity_tcp_port",
        default_value="10000",
        description="Unity TCP endpoint port to probe for network health metrics.",
    )
    foxglove_bridge_host_arg = DeclareLaunchArgument(
        "foxglove_bridge_host",
        default_value="127.0.0.1",
        description="Host to probe for Foxglove bridge reachability.",
    )
    foxglove_bridge_port_arg = DeclareLaunchArgument(
        "foxglove_bridge_port",
        default_value="8765",
        description="Port to probe for Foxglove bridge reachability.",
    )
    foxglove_bridge_topic_whitelist_arg = DeclareLaunchArgument(
        "foxglove_bridge_topic_whitelist",
        default_value=(
            "['^(?!/camera/camera/color/image_raw/compressedDepth$)"
            "(?!/camera/camera/depth/image_rect_raw/compressed$).*']"
        ),
        description=(
            "Regex whitelist for foxglove_bridge topics. "
            "Defaults to excluding invalid RealSense transport combinations."
        ),
    )

    runtime_observability_node = Node(
        package="holoassist_foxglove",
        executable="runtime_observability_node",
        name="holoassist_runtime_observability",
        output="screen",
        parameters=[
            {
                "diagnostics_rate_hz": LaunchConfiguration("diagnostics_rate_hz"),
                "stale_timeout_s": LaunchConfiguration("stale_timeout_s"),
                "unity_tcp_host": LaunchConfiguration("unity_tcp_host"),
                "unity_tcp_port": LaunchConfiguration("unity_tcp_port"),
                "foxglove_bridge_host": LaunchConfiguration("foxglove_bridge_host"),
                "foxglove_bridge_port": LaunchConfiguration("foxglove_bridge_port"),
            }
        ],
    )

    manager_node = Node(
        package="holoassist_manager",
        executable="manager_node",
        name="holoassist_manager",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_manager")),
    )

    tf_marker_bridge_node = Node(
        package="holoassist_manager",
        executable="tf_marker_bridge",
        name="tf_marker_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_tf_marker_bridge")),
    )
    object_pose_adapter_node = Node(
        package="holoassist_foxglove",
        executable="obstacle_to_object_pose_adapter",
        name="holoassist_object_pose_adapter",
        output="screen",
        parameters=[
            {
                "workspace_frame": LaunchConfiguration("object_workspace_frame"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_object_pose_adapter")),
    )

    try:
        foxglove_launch_file = os.path.join(
            get_package_share_directory("foxglove_bridge"),
            "launch",
            "foxglove_bridge_launch.xml",
        )
        foxglove_bridge_action = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(foxglove_launch_file),
            condition=IfCondition(LaunchConfiguration("enable_foxglove_bridge")),
            launch_arguments={
                "topic_whitelist": LaunchConfiguration("foxglove_bridge_topic_whitelist"),
                "client_topic_whitelist": LaunchConfiguration("foxglove_bridge_topic_whitelist"),
            }.items(),
        )
    except PackageNotFoundError:
        foxglove_bridge_action = LogInfo(
            condition=IfCondition(LaunchConfiguration("enable_foxglove_bridge")),
            msg=(
                "[holoassist_foxglove] foxglove_bridge package not found. "
                "Install foxglove_bridge to enable live Foxglove connections."
            ),
        )

    return LaunchDescription(
        [
            enable_foxglove_bridge_arg,
            enable_manager_arg,
            enable_tf_marker_bridge_arg,
            enable_object_pose_adapter_arg,
            object_workspace_frame_arg,
            diagnostics_rate_hz_arg,
            stale_timeout_s_arg,
            unity_tcp_host_arg,
            unity_tcp_port_arg,
            foxglove_bridge_host_arg,
            foxglove_bridge_port_arg,
            foxglove_bridge_topic_whitelist_arg,
            runtime_observability_node,
            manager_node,
            tf_marker_bridge_node,
            object_pose_adapter_node,
            foxglove_bridge_action,
        ]
    )
