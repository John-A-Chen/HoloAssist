"""
HoloAssist Board Calibration Launch File
=========================================

Starts the interactive robot-FK board calibration workflow.

ASSUMPTIONS
-----------
This launch file assumes the UR3e robot driver and MoveIt move_group are
already running in another terminal (they take longer to start and are
typically shared with other workflows).

If they are NOT already running, add the arguments:
  start_robot:=true  robot_ip:=192.168.0.194
  start_moveit:=true

QUICK START (robot + MoveIt already running)
--------------------------------------------
  ros2 launch holo_assist_depth_tracker board_calibration.launch.py \\
      start_camera:=true

FULL LAUNCH (bring up everything from scratch)
----------------------------------------------
  ros2 launch holo_assist_depth_tracker board_calibration.launch.py \\
      start_robot:=true   robot_ip:=192.168.0.194 \\
      start_moveit:=true  start_camera:=true

CAMERA VERIFICATION NOTE
------------------------
workspace_board_node is intentionally NOT started here.
workspace_board_node publishes  camera_color_optical_frame → workspace_frame,
the calibration node publishes  base_link                  → workspace_frame.
Both cannot run simultaneously (workspace_frame can only have one TF parent).

The optional camera verification pass uses tag TFs that workspace_board_node
broadcasts.  If you want camera verification, run workspace_board_node in a
*separate* terminal ONLY for the verification step, then stop it before the
calibration node publishes its static TF.  Set verify_with_camera:=false to
skip camera verification and proceed with FK-only calibration.

CONTROLLER NOTE
---------------
The calibration uses the scaled_joint_trajectory_controller.
If forward_velocity_controller is active (teleop mode), switch controllers:
  ros2 control switch_controllers \\
      --activate   scaled_joint_trajectory_controller \\
      --deactivate forward_velocity_controller
"""

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("holo_assist_depth_tracker")

    # ── Default file paths ────────────────────────────────────────────────────
    default_cal_params = os.path.join(pkg_share, "config", "board_calibration_params.yaml")
    default_apriltag_params = os.path.join(pkg_share, "config", "apriltag_all.yaml")

    # ── Launch arguments ──────────────────────────────────────────────────────
    cal_params_arg = DeclareLaunchArgument(
        "cal_params_file",
        default_value=default_cal_params,
        description="Calibration node parameter YAML.",
    )
    apriltag_params_arg = DeclareLaunchArgument(
        "apriltag_params_file",
        default_value=default_apriltag_params,
        description="AprilTag detector parameter YAML.",
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/camera/color/image_raw",
        description="Colour image topic for AprilTag detection.",
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/camera/color/camera_info",
        description="Camera info topic for AprilTag detection.",
    )
    start_camera_arg = DeclareLaunchArgument(
        "start_camera",
        default_value="false",
        description="Launch the RealSense camera driver (set true if not already running).",
    )
    start_robot_arg = DeclareLaunchArgument(
        "start_robot",
        default_value="false",
        description="Launch the UR3e + OnRobot robot driver (set true if not already running).",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.0.194",
        description="UR3e robot IP address (used only when start_robot:=true).",
    )
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur3e",
    )
    onrobot_type_arg = DeclareLaunchArgument(
        "onrobot_type",
        default_value="rg2",
    )
    start_moveit_arg = DeclareLaunchArgument(
        "start_moveit",
        default_value="false",
        description="Launch MoveIt move_group (set true if not already running).",
    )
    verify_with_camera_arg = DeclareLaunchArgument(
        "verify_with_camera",
        default_value="true",
        description=(
            "Run camera-based residual verification after SVD solve. "
            "Requires AprilTag detector to be running. "
            "workspace_board_node must NOT be running concurrently."
        ),
    )

    # ── Optional: camera driver ───────────────────────────────────────────────
    try:
        camera_launch_path = PathJoinSubstitution(
            [FindPackageShare("holo_assist_depth_tracker"), "launch", "camera_only.launch.py"]
        )
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path),
            condition=IfCondition(LaunchConfiguration("start_camera")),
        )
    except Exception:
        camera_launch = LogInfo(
            msg="[board_calibration] camera_only.launch.py not found — skipping camera start."
        )

    # ── Optional: robot driver ────────────────────────────────────────────────
    try:
        robot_launch_path = PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_control"), "launch", "start_robot.launch.py"]
        )
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch_path),
            launch_arguments={
                "ur_type": LaunchConfiguration("ur_type"),
                "onrobot_type": LaunchConfiguration("onrobot_type"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "launch_rviz": "false",
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_robot")),
        )
    except Exception:
        robot_launch = LogInfo(
            msg="[board_calibration] ur_onrobot_control not found — skipping robot launch."
        )

    # ── Optional: MoveIt move_group ───────────────────────────────────────────
    try:
        moveit_launch_path = PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_moveit_config"), "launch", "ur_onrobot_moveit.launch.py"]
        )
        moveit_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch_path),
            condition=IfCondition(LaunchConfiguration("start_moveit")),
        )
        # Give the robot driver 5 s head-start before MoveIt tries to connect
        moveit_launch = TimerAction(period=5.0, actions=[moveit_launch])
    except Exception:
        moveit_launch = LogInfo(
            msg="[board_calibration] ur_onrobot_moveit_config not found — skipping MoveIt launch."
        )

    # ── AprilTag detector (for camera verification) ───────────────────────────
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
                "[board_calibration] apriltag_ros not found — "
                "camera verification will be skipped. "
                "Install: sudo apt install ros-humble-apriltag ros-humble-apriltag-ros"
            )
        )

    # ── Board calibration node ────────────────────────────────────────────────
    calibration_node = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_board_calibration",
        name="holoassist_board_calibration",
        output="screen",
        parameters=[
            LaunchConfiguration("cal_params_file"),
            {
                # Allow per-launch override of camera verification flag
                "verify_with_camera": LaunchConfiguration("verify_with_camera"),
            },
        ],
    )

    # Separator info messages
    info_pre = LogInfo(msg=(
        "\n"
        "================================================================\n"
        "  HoloAssist Board Calibration\n"
        "  IMPORTANT: workspace_board_node must NOT be running.\n"
        "  The calibration node will publish base_link → workspace_frame.\n"
        "================================================================"
    ))

    return LaunchDescription(
        [
            # arguments
            cal_params_arg,
            apriltag_params_arg,
            image_topic_arg,
            camera_info_topic_arg,
            start_camera_arg,
            start_robot_arg,
            robot_ip_arg,
            ur_type_arg,
            onrobot_type_arg,
            start_moveit_arg,
            verify_with_camera_arg,
            # optional infrastructure
            camera_launch,
            robot_launch,
            moveit_launch,
            # detection + calibration
            info_pre,
            apriltag_node,
            calibration_node,
        ]
    )
