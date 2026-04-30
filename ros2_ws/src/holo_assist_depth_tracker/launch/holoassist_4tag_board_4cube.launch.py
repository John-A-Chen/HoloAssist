from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("holo_assist_depth_tracker")

    board_params_default = PathJoinSubstitution([pkg_share, "config", "apriltag_board.yaml"])
    cubes_params_default = PathJoinSubstitution([pkg_share, "config", "apriltag_cubes.yaml"])
    workspace_params_default = PathJoinSubstitution([pkg_share, "config", "workspace.yaml"])
    cube_pose_params_default = PathJoinSubstitution([pkg_share, "config", "cubes.yaml"])
    tracker_params_default = PathJoinSubstitution([pkg_share, "config", "tracker_params.yaml"])

    start_camera_arg = DeclareLaunchArgument("start_camera", default_value="false")
    image_topic_arg = DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw")
    camera_info_topic_arg = DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info")
    start_tracker_arg = DeclareLaunchArgument("start_tracker", default_value="true")
    start_overlay_arg = DeclareLaunchArgument("start_overlay", default_value="true")

    board_params_arg = DeclareLaunchArgument("board_params_file", default_value=board_params_default)
    cubes_params_arg = DeclareLaunchArgument("cubes_params_file", default_value=cubes_params_default)
    workspace_params_arg = DeclareLaunchArgument("workspace_params_file", default_value=workspace_params_default)
    cube_pose_params_arg = DeclareLaunchArgument("cube_pose_params_file", default_value=cube_pose_params_default)
    tracker_params_arg = DeclareLaunchArgument("tracker_params_file", default_value=tracker_params_default)

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("holo_assist_depth_tracker"), "launch", "camera_only.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    try:
        get_package_share_directory("apriltag_ros")
        apriltag_board_node = Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag_board",
            output="screen",
            parameters=[LaunchConfiguration("board_params_file")],
            remappings=[
                ("image_rect", LaunchConfiguration("image_topic")),
                ("camera_info", LaunchConfiguration("camera_info_topic")),
                ("detections", "/detections_board"),
            ],
        )
        apriltag_cubes_node = Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag_cubes",
            output="screen",
            parameters=[LaunchConfiguration("cubes_params_file")],
            remappings=[
                ("image_rect", LaunchConfiguration("image_topic")),
                ("camera_info", LaunchConfiguration("camera_info_topic")),
                ("detections", "/detections_cubes"),
            ],
        )
    except PackageNotFoundError:
        apriltag_board_node = LogInfo(
            msg=(
                "[holoassist_4tag_board_4cube] apriltag_ros not found. "
                "Install ros-humble-apriltag ros-humble-apriltag-ros."
            )
        )
        apriltag_cubes_node = LogInfo(msg="[holoassist_4tag_board_4cube] skipping cube detector")

    merge_node = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_detection_merge_node",
        name="holoassist_detection_merge",
        output="screen",
        parameters=[{"board_topic": "/detections_board", "cubes_topic": "/detections_cubes", "output_topic": "/detections_all"}],
    )

    workspace_node = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_workspace_board_node",
        name="holoassist_workspace_board",
        output="screen",
        parameters=[LaunchConfiguration("workspace_params_file")],
    )

    cube_pose_node = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_cube_pose_node",
        name="holoassist_cube_pose",
        output="screen",
        parameters=[LaunchConfiguration("cube_pose_params_file")],
    )

    overlay_node = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_overlay_node",
        name="holoassist_overlay",
        output="screen",
        parameters=[
            {
                "input_image_topic": LaunchConfiguration("image_topic"),
                "detections_topic": "/detections_all",
                "output_image_topic": "/holoassist/perception/apriltag_overlay",
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_overlay")),
    )

    tracker_node = Node(
        package="holo_assist_depth_tracker",
        executable="holo_assist_depth_tracker_node",
        name="holo_assist_depth_tracker",
        output="screen",
        parameters=[LaunchConfiguration("tracker_params_file")],
        condition=IfCondition(LaunchConfiguration("start_tracker")),
    )

    return LaunchDescription(
        [
            start_camera_arg,
            image_topic_arg,
            camera_info_topic_arg,
            start_tracker_arg,
            start_overlay_arg,
            board_params_arg,
            cubes_params_arg,
            workspace_params_arg,
            cube_pose_params_arg,
            tracker_params_arg,
            camera_launch,
            apriltag_board_node,
            apriltag_cubes_node,
            merge_node,
            workspace_node,
            cube_pose_node,
            overlay_node,
            tracker_node,
        ]
    )
