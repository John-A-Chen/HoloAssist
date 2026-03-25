from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("holo_assist_depth_tracker")

    default_params = PathJoinSubstitution(
        [pkg_share, "config", "tracker_params.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [pkg_share, "config", "depth_tracker_visualization.rviz"]
    )

    start_camera_arg = DeclareLaunchArgument(
        "start_camera",
        default_value="true",
        description="Start RealSense depth camera launch.",
    )
    start_tracker_arg = DeclareLaunchArgument(
        "start_tracker",
        default_value="true",
        description="Start holo_assist_depth_tracker node.",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Start RViz2 with pointcloud/marker/image displays.",
    )
    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="camera",
        description="RealSense camera node name/prefix.",
    )
    depth_profile_arg = DeclareLaunchArgument(
        "depth_profile",
        default_value="848,480,15",
        description="Depth stream profile W,H,FPS.",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to tracker parameter YAML.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Path to RViz2 config file.",
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_share, "launch", "camera_only.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_camera")),
        launch_arguments={
            "camera_name": LaunchConfiguration("camera_name"),
            "depth_profile": LaunchConfiguration("depth_profile"),
        }.items(),
    )

    tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_share, "launch", "holo_assist_depth_tracker.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_tracker")),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="holo_assist_depth_tracker_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return LaunchDescription(
        [
            start_camera_arg,
            start_tracker_arg,
            start_rviz_arg,
            camera_name_arg,
            depth_profile_arg,
            params_file_arg,
            rviz_config_arg,
            camera_launch,
            tracker_launch,
            rviz_node,
        ]
    )
