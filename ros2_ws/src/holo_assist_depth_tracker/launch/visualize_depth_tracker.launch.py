from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
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
    start_workspace_perception_arg = DeclareLaunchArgument(
        "start_workspace_perception",
        default_value="true",
        description=(
            "Start workspace perception adapter (bench plane + culling + "
            "foreground object localization)."
        ),
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
    color_profile_arg = DeclareLaunchArgument(
        "color_profile",
        default_value="640,480,15",
        description="Color stream profile W,H,FPS.",
    )
    enable_color_arg = DeclareLaunchArgument(
        "enable_color",
        default_value="true",
        description="Enable RGB stream for debug image overlays.",
    )
    align_depth_arg = DeclareLaunchArgument(
        "align_depth",
        default_value="false",
        description="Enable RealSense align depth filter.",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to tracker parameter YAML.",
    )
    workspace_params_file_arg = DeclareLaunchArgument(
        "workspace_params_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "workspace_perception_params.yaml"]
        ),
        description="Path to workspace perception parameter YAML.",
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
            "color_profile": LaunchConfiguration("color_profile"),
            "enable_color": LaunchConfiguration("enable_color"),
            "align_depth": LaunchConfiguration("align_depth"),
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

    workspace_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_share, "launch", "workspace_perception.launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_workspace_perception")),
        launch_arguments={
            "params_file": LaunchConfiguration("workspace_params_file"),
        }.items(),
    )

    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/camera/color/image_raw",
        description="Color image topic fed to apriltag_ros.",
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/camera/color/camera_info",
        description="Camera info topic fed to apriltag_ros.",
    )
    apriltag_params_file_arg = DeclareLaunchArgument(
        "apriltag_params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "apriltag_all.yaml"]),
        description="apriltag_ros parameter YAML (tag family, size, IDs).",
    )
    cube_pose_params_file_arg = DeclareLaunchArgument(
        "cube_pose_params_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "cubes.yaml"]),
        description="Cube pose node parameter YAML.",
    )

    # apriltag_ros: detects board tags (0-3) and cube tags (10-33), publishes TF +
    # /detections_all.  workspace_perception_node reads the TF; cube_pose_node reads
    # both the TF and the detections.
    try:
        get_package_share_directory("apriltag_ros")
        apriltag_node = Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            parameters=[LaunchConfiguration("apriltag_params_file")],
            remappings=[
                ("image_rect", LaunchConfiguration("image_topic")),
                ("camera_info", LaunchConfiguration("camera_info_topic")),
                ("detections", "/detections_all"),
            ],
        )
    except PackageNotFoundError:
        apriltag_node = LogInfo(
            msg=(
                "[visualize_depth_tracker] apriltag_ros package not found. "
                "Install: sudo apt install ros-humble-apriltag ros-humble-apriltag-ros"
            )
        )

    # cube_pose_node: estimates 4 physical cube centres from grouped tag IDs and
    # publishes /holoassist/perception/april_cube_N_{pose,marker,status} + TF frames.
    cube_pose_node = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_cube_pose_node",
        name="holoassist_cube_pose",
        output="screen",
        parameters=[LaunchConfiguration("cube_pose_params_file")],
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
            start_workspace_perception_arg,
            start_rviz_arg,
            camera_name_arg,
            depth_profile_arg,
            color_profile_arg,
            enable_color_arg,
            align_depth_arg,
            params_file_arg,
            workspace_params_file_arg,
            rviz_config_arg,
            image_topic_arg,
            camera_info_topic_arg,
            apriltag_params_file_arg,
            cube_pose_params_file_arg,
            camera_launch,
            apriltag_node,
            tracker_launch,
            workspace_launch,
            cube_pose_node,
            rviz_node,
        ]
    )
