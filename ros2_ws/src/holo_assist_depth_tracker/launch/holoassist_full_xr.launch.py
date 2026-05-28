"""
Full HoloAssist XR perception launch — no robot required.

Starts the complete AprilTag perception pipeline (camera, board solver, cube tracker)
and a rosbridge WebSocket server so Unity/HoloLens can subscribe to cube poses in
real time.

Unity topics of interest (all PoseStamped in workspace_frame):
  /holoassist/perception/april_cube_1_pose   ... april_cube_4_pose
  /holoassist/perception/april_cube_1_status ... april_cube_4_status
  /holoassist/perception/workspace_mode        (String: LOCKED / INVALID)
  /holoassist/perception/ur3e_base_link0_pose  (robot base in workspace frame)
  /holoassist/perception/bench_plane_coefficients

rosbridge WebSocket endpoint: ws://<host>:9090
Connect from Unity with ROSBridgeWebSocketConnection or roslibjs.

Startup sequence:
  1. RealSense camera → image + camera_info
  2. apriltag_ros → tag TF frames (tag36h11:N) in camera_color_optical_frame
  3. workspace_board_node → workspace_frame TF locked from board corner tags 0-3
  4. cube_pose_node → cube poses appear once workspace is locked and cube tags visible
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("holo_assist_depth_tracker")

    perception_launch = PathJoinSubstitution(
        [pkg_share, "launch", "holoassist_4tag_board_4cube.launch.py"]
    )

    # ── Arguments ────────────────────────────────────────────────────────────
    start_camera_arg = DeclareLaunchArgument(
        "start_camera",
        default_value="true",
        description="Launch the RealSense camera node.",
    )
    start_rosbridge_arg = DeclareLaunchArgument(
        "start_rosbridge",
        default_value="true",
        description="Launch rosbridge WebSocket server for Unity/HoloLens.",
    )
    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port",
        default_value="9090",
        description="WebSocket port for rosbridge (Unity connects here).",
    )

    # ── Perception pipeline ───────────────────────────────────────────────────
    # Starts: (optionally) RealSense camera, apriltag_ros, workspace_board_node,
    # cube_pose_node, overlay_node (apriltag debug overlay).
    perception_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch),
        launch_arguments={
            "start_camera": LaunchConfiguration("start_camera"),
        }.items(),
    )

    # ── rosbridge WebSocket server ────────────────────────────────────────────
    # Unity/HoloLens connects to ws://<host>:9090 and subscribes to cube pose
    # topics. Works with Unity Robotics Hub WebSocket adapter and roslibjs.
    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("rosbridge_port"),
            "address": "",           # bind all interfaces
            "ssl": False,
            "authenticate": False,
        }],
        condition=IfCondition(LaunchConfiguration("start_rosbridge")),
    )

    return LaunchDescription(
        [
            start_camera_arg,
            start_rosbridge_arg,
            rosbridge_port_arg,
            perception_stack,
            rosbridge,
        ]
    )
