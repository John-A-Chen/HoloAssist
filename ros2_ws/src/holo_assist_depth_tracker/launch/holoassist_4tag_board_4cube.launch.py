from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import re


def _validate_apriltag_params(context, *args):
    del args
    expected = 0.032
    path = LaunchConfiguration("apriltag_params_file").perform(context)
    messages = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
    except Exception as exc:
        return [LogInfo(msg=f"[WARN] unable to read apriltag params file '{path}': {exc}")]

    match = re.search(r"^\\s*size\\s*:\\s*([-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)\\s*$", text, re.MULTILINE)
    if not match:
        messages.append(
            LogInfo(
                msg=f"[WARN] apriltag params file '{path}' has no 'size:' entry; expected {expected:.3f} m."
            )
        )
        return messages

    value = float(match.group(1))
    if abs(value - expected) > 1e-6:
        messages.append(
            LogInfo(
                msg=f"[WARN] apriltag detector size is {value:.6f} m in '{path}', expected {expected:.3f} m."
            )
        )
    if value > 0.05:
        messages.append(
            LogInfo(
                msg=f"[WARN] apriltag detector size {value:.6f} m is unusually large for current 32 mm tags."
            )
        )
    if value >= 1.0:
        messages.append(
            LogInfo(
                msg=f"[WARN] apriltag detector size {value:.6f} looks like mm passed as meters (e.g. 32/40)."
            )
        )
    return messages


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("holo_assist_depth_tracker")

    apriltag_params_default = PathJoinSubstitution([pkg_share, "config", "apriltag_all.yaml"])
    workspace_params_default = PathJoinSubstitution([pkg_share, "config", "workspace.yaml"])
    cube_pose_params_default = PathJoinSubstitution([pkg_share, "config", "cubes.yaml"])
    tracker_params_default = PathJoinSubstitution([pkg_share, "config", "tracker_params.yaml"])

    start_camera_arg = DeclareLaunchArgument("start_camera", default_value="false")
    image_topic_arg = DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw")
    camera_info_topic_arg = DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info")
    start_tracker_arg = DeclareLaunchArgument("start_tracker", default_value="true")
    start_overlay_arg = DeclareLaunchArgument("start_overlay", default_value="true")

    apriltag_params_arg = DeclareLaunchArgument("apriltag_params_file", default_value=apriltag_params_default)
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
                "[holoassist_4tag_board_4cube] apriltag_ros not found. "
                "Install ros-humble-apriltag ros-humble-apriltag-ros."
            )
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
            apriltag_params_arg,
            workspace_params_arg,
            cube_pose_params_arg,
            tracker_params_arg,
            camera_launch,
            OpaqueFunction(function=_validate_apriltag_params),
            apriltag_node,
            workspace_node,
            cube_pose_node,
            overlay_node,
            tracker_node,
        ]
    )
