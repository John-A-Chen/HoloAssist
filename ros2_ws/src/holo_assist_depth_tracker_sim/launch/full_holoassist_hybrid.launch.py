"""Hybrid mode: fake MoveIt sim robot + real RealSense perception pipeline.

TF tree
-------
world ──┬── base_link ──► [sim robot links]          (robot_state_publisher)
        └── camera_link ──► camera_color_optical_frame (RealSense driver)
                               └── workspace_frame    (workspace_board_node, dynamic)
                                      └── apriltag_cube_N  (cube_pose_node)

workspace_frame owner: workspace_board_node (real camera AprilTag detection).
Cube pose source:      /holoassist/perception/april_cube_N_pose (cube_pose_node).

NOT started: workspace_frame_tf, sim_cube_truth_node, sim_cube_perception_node.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

try:
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
except ImportError:
    PackageNotFoundError = Exception


# ── Robot description builder (identical to full_holoassist_moveit_sim) ──────

def _build_robot_stack(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    onrobot_type = LaunchConfiguration("onrobot_type").perform(context)
    robot_base_yaw_rad = LaunchConfiguration("robot_base_yaw_rad").perform(context)

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("ur_onrobot_description"), "urdf", "ur_onrobot.urdf.xacro"]
    )
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            xacro_file, " ",
            "ur_type:=", ur_type, " ",
            "onrobot_type:=", onrobot_type, " ",
            "name:=ur_onrobot", " ",
            "use_fake_hardware:=true", " ",
            "robot_ip:=0.0.0.0", " ",
            "base_yaw_rad:=", robot_base_yaw_rad,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    ur_update_rate_config = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "config", ur_type + "_update_rate.yaml"]
    )
    controller_config = PathJoinSubstitution(
        [FindPackageShare("moveit_robot_control"), "config", "sim_controllers.yaml"]
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, ur_update_rate_config, controller_config],
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "15",
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "finger_width_trajectory_controller",
        ],
        output="screen",
    )
    return [rsp, ros2_control_node, controller_spawner]


# ── Launch description ────────────────────────────────────────────────────────

def generate_launch_description() -> LaunchDescription:
    sim_pkg = FindPackageShare("holo_assist_depth_tracker_sim")
    ctrl_pkg = FindPackageShare("moveit_robot_control")
    perc_pkg = FindPackageShare("holo_assist_depth_tracker")
    moveit_pkg = FindPackageShare("ur_onrobot_moveit_config")

    # ── Arguments: robot ──────────────────────────────────────────────────────
    ur_type_arg = DeclareLaunchArgument("ur_type", default_value="ur3e")
    onrobot_type_arg = DeclareLaunchArgument("onrobot_type", default_value="rg2")
    robot_base_yaw_rad_arg = DeclareLaunchArgument(
        "robot_base_yaw_rad",
        default_value="3.14159",
        description="world→base_link mounting yaw. Default π matches HoloAssist trolley orientation.",
    )
    moveit_launch_file_arg = DeclareLaunchArgument(
        "moveit_launch_file", default_value="ur_onrobot_moveit.launch.py"
    )
    start_moveit_arg = DeclareLaunchArgument("start_moveit", default_value="true")
    move_group_name_arg = DeclareLaunchArgument(
        "move_group_name", default_value="ur_onrobot_manipulator"
    )
    ee_link_arg = DeclareLaunchArgument("ee_link", default_value="gripper_tcp")
    frame_arg = DeclareLaunchArgument("frame", default_value="base_link")
    velocity_scale_arg = DeclareLaunchArgument("velocity_scale", default_value="0.05")
    orientation_mode_arg = DeclareLaunchArgument("orientation_mode", default_value="auto")
    avoid_flange_forearm_clamp_arg = DeclareLaunchArgument(
        "avoid_flange_forearm_clamp", default_value="true"
    )
    pose_goal_planning_time_arg = DeclareLaunchArgument(
        "pose_goal_planning_time", default_value="5.0"
    )
    start_pick_place_arg = DeclareLaunchArgument(
        "start_pick_place",
        default_value="true",
        description="Launch pick_place_sequencer and pick_place_service_node.",
    )

    # ── Arguments: real camera perception ────────────────────────────────────
    start_camera_arg = DeclareLaunchArgument(
        "start_camera",
        default_value="true",
        description="Start RealSense camera driver.",
    )
    apriltag_params_arg = DeclareLaunchArgument(
        "apriltag_params_file",
        default_value=PathJoinSubstitution([perc_pkg, "config", "apriltag_all.yaml"]),
    )
    workspace_params_arg = DeclareLaunchArgument(
        "workspace_params_file",
        default_value=PathJoinSubstitution([perc_pkg, "config", "workspace.yaml"]),
    )
    cube_pose_params_arg = DeclareLaunchArgument(
        "cube_pose_params_file",
        default_value=PathJoinSubstitution([perc_pkg, "config", "cubes.yaml"]),
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic", default_value="/camera/camera/color/image_raw"
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic", default_value="/camera/camera/color/camera_info"
    )

    # ── Arguments: workspace_frame position in base_link ─────────────────────
    # Used by workspace_align_camera_tf_node to snap the real board detection
    # onto the known workspace position, automatically computing camera mount TF.
    # Defaults match workspace_frame_tf hardcoded values (no calibration file needed).
    # Override by passing a calibration YAML: workspace_x_m, workspace_y_m, etc.
    camera_link_frame_arg = DeclareLaunchArgument(
        "camera_link_frame",
        default_value="camera_link",
        description="Root TF frame published by the RealSense driver.",
    )
    workspace_x_arg = DeclareLaunchArgument(
        "workspace_x_m", default_value="-0.10",
        description="Known workspace_frame X in base_link (metres).",
    )
    workspace_y_arg = DeclareLaunchArgument(
        "workspace_y_m", default_value="-0.314",
        description="Known workspace_frame Y in base_link (metres).",
    )
    workspace_z_arg = DeclareLaunchArgument(
        "workspace_z_m", default_value="0.015",
        description="Known workspace_frame Z in base_link (metres).",
    )
    workspace_roll_arg = DeclareLaunchArgument("workspace_roll_rad", default_value="0.0")
    workspace_pitch_arg = DeclareLaunchArgument("workspace_pitch_rad", default_value="0.0")
    workspace_yaw_arg = DeclareLaunchArgument("workspace_yaw_rad", default_value="0.0")

    # ── Arguments: display ────────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([ctrl_pkg, "rviz", "holoassist_hw.rviz"]),
    )

    # ── Fake robot stack ──────────────────────────────────────────────────────
    robot_bringup = OpaqueFunction(function=_build_robot_stack)

    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_pkg, "launch", LaunchConfiguration("moveit_launch_file")])
        ),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "onrobot_type": LaunchConfiguration("onrobot_type"),
            "use_fake_hardware": "true",
            "launch_rviz": "false",
            "launch_servo": "false",
            "base_yaw_rad": LaunchConfiguration("robot_base_yaw_rad"),
        }.items(),
    )

    # ── Camera TF alignment node ───────────────────────────────────────────────
    # Replaces the hand-tuned static camera TF. Waits for workspace_board_node
    # to detect the real board, then snaps the camera to the position that makes
    # the detected workspace_frame align with the known base_link→workspace_frame.
    # No manual camera position measurement needed.
    camera_align_tf = Node(
        package="holo_assist_depth_tracker_sim",
        executable="workspace_align_camera_tf_node",
        name="holoassist_workspace_align_camera_tf",
        output="screen",
        parameters=[{
            "workspace_x_m": LaunchConfiguration("workspace_x_m"),
            "workspace_y_m": LaunchConfiguration("workspace_y_m"),
            "workspace_z_m": LaunchConfiguration("workspace_z_m"),
            "workspace_roll_rad": LaunchConfiguration("workspace_roll_rad"),
            "workspace_pitch_rad": LaunchConfiguration("workspace_pitch_rad"),
            "workspace_yaw_rad": LaunchConfiguration("workspace_yaw_rad"),
            "camera_link_frame": LaunchConfiguration("camera_link_frame"),
        }],
    )

    # ── RealSense camera driver ───────────────────────────────────────────────
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([perc_pkg, "launch", "camera_only.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    # ── AprilTag detector ─────────────────────────────────────────────────────
    # Detects board tags (IDs 0-3) and cube tags (IDs 10-33).
    # Publishes /detections_all and TF frames for each detected tag.
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
            msg="[hybrid] apriltag_ros not found. Install: sudo apt install ros-humble-apriltag ros-humble-apriltag-ros"
        )

    # ── workspace_board_node: board AprilTags → workspace_frame TF ───────────
    # Subscribes to /detections_all, locks board geometry from tags 0-3,
    # publishes camera_color_optical_frame → workspace_frame (dynamic).
    # This is the SOLE owner of workspace_frame in hybrid mode.
    workspace_board_node = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_workspace_board_node",
        name="holoassist_workspace_board",
        output="screen",
        parameters=[LaunchConfiguration("workspace_params_file")],
    )

    # ── cube_pose_node: cube AprilTags → /holoassist/perception/april_cube_N_pose ──
    # Subscribes to /detections_all, fuses 6 tags per cube, publishes poses
    # in workspace_frame. This is the source of truth for robot targeting.
    cube_pose_node = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_cube_pose_node",
        name="holoassist_cube_pose",
        output="screen",
        parameters=[LaunchConfiguration("cube_pose_params_file")],
    )

    # ── Trolley mesh (visual + optional collision) ────────────────────────────
    workspace_scene = Node(
        package="moveit_robot_control",
        executable="workspace_scene_manager",
        name="workspace_scene_manager",
        output="screen",
        emulate_tty=True,
        parameters=[PathJoinSubstitution([ctrl_pkg, "config", "full_holoassist_hw.yaml"])],
    )

    # ── Coordinate listener: topic-driven MoveIt targets → trajectories ───────
    coordinate_listener = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ctrl_pkg, "launch", "coordinate_listener.launch.py"])
        ),
        launch_arguments={
            "move_group_name": LaunchConfiguration("move_group_name"),
            "ee_link": LaunchConfiguration("ee_link"),
            "frame": LaunchConfiguration("frame"),
            "require_robot_status": "false",
            "require_controller_check": "false",
            "allow_pose_goal_fallback": "true",
            "orientation_mode": LaunchConfiguration("orientation_mode"),
            "avoid_flange_forearm_clamp": LaunchConfiguration("avoid_flange_forearm_clamp"),
            "pose_goal_planning_time": LaunchConfiguration("pose_goal_planning_time"),
            "velocity_scale": LaunchConfiguration("velocity_scale"),
            "trajectory_topic": "/joint_trajectory_controller/joint_trajectory",
        }.items(),
    )

    # ── Pick-and-place sequencer ──────────────────────────────────────────────
    pick_place_sequencer = Node(
        package="moveit_robot_control",
        executable="pick_place_sequencer",
        name="pick_place_sequencer",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_pick_place")),
        parameters=[{
            "initial_mode": "run",
            "orientation_mode": "auto",
            "pregrasp_z_offset": 0.10,
            "grasp_z_offset": 0.0,
            "place_above_z_offset": 0.15,
            "place_z_offset": 0.05,
            "place_descent_enabled": True,
        }],
    )

    # ── MoveIt planning scene bridge (real cube poses → collision objects) ────
    # cube_pose_topic_prefix=/holoassist/perception routes this to the real pipeline.
    # Publishes /holoassist/teleop/selected_cube_pose from the selected cube's real pose.
    # NOTE: CubePerceptionStatus subscriptions will receive nothing (real node publishes
    # String, not CubePerceptionStatus) — visibility warnings are silently suppressed.
    moveit_bridge = Node(
        package="holo_assist_depth_tracker_sim",
        executable="sim_cube_moveit_bridge_node",
        name="holoassist_cube_moveit_bridge",
        output="screen",
        parameters=[
            PathJoinSubstitution([sim_pkg, "config", "sim_scene.yaml"]),
            {"cube_pose_topic_prefix": "/holoassist/perception"},
        ],
    )

    # ── Selected-cube → MoveIt target adapter ────────────────────────────────
    # Reads /holoassist/teleop/selected_cube_pose (in workspace_frame, from bridge above),
    # TF-transforms to base_link, applies 10 cm hover offset, publishes MoveIt targets.
    selected_cube_adapter = Node(
        package="holo_assist_depth_tracker_sim",
        executable="selected_cube_to_moveit_target_node",
        name="holoassist_selected_cube_to_moveit_target",
        output="screen",
        parameters=[
            PathJoinSubstitution([sim_pkg, "config", "sim_scene.yaml"]),
            PathJoinSubstitution([ctrl_pkg, "config", "full_holoassist_hw.yaml"]),
            {
                "target_frame": LaunchConfiguration("frame"),
                "output_point_topic": "/moveit_robot_control/target_point",
                "output_pose_topic": "/moveit_robot_control/target_pose",
            },
        ],
    )

    # ── Pick-cube-to-bin service ──────────────────────────────────────────────
    # cube_pose_topic_prefix=/holoassist/perception uses real camera cube poses.
    pick_place_service = Node(
        package="holo_assist_depth_tracker_sim",
        executable="pick_place_service_node",
        name="holoassist_pick_place_service",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_pick_place")),
        parameters=[{"cube_pose_topic_prefix": "/holoassist/perception"}],
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="holoassist_hybrid_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # ── Startup sequencing ────────────────────────────────────────────────────
    # t=0  fake robot + MoveIt + camera + static camera TF + apriltag + perception nodes
    # t=3  workspace_scene_manager (trolley; needs TF to be up)
    # t=4  coordinate_listener + pick_place_sequencer (need move_group)
    # t=5  moveit_bridge + selected_cube_adapter + pick_place_service
    #         (need cube poses and TF — workspace_board_node may not have locked yet;
    #          bridge will just publish nothing until the first cube is detected)
    # t=6  RViz

    return LaunchDescription(
        [
            LogInfo(msg=(
                "[hybrid] HYBRID MODE: fake MoveIt robot + real RealSense perception. "
                "workspace_frame owned by workspace_board_node. "
                "Cube poses from /holoassist/perception/april_cube_N_pose. "
                "NOT started: workspace_frame_tf, sim_cube_truth_node, sim_cube_perception_node."
            )),
            # Robot args
            ur_type_arg,
            onrobot_type_arg,
            robot_base_yaw_rad_arg,
            moveit_launch_file_arg,
            start_moveit_arg,
            move_group_name_arg,
            ee_link_arg,
            frame_arg,
            velocity_scale_arg,
            orientation_mode_arg,
            avoid_flange_forearm_clamp_arg,
            pose_goal_planning_time_arg,
            start_pick_place_arg,
            # Perception args
            start_camera_arg,
            apriltag_params_arg,
            workspace_params_arg,
            cube_pose_params_arg,
            image_topic_arg,
            camera_info_topic_arg,
            # Workspace position args (for camera alignment)
            camera_link_frame_arg,
            workspace_x_arg,
            workspace_y_arg,
            workspace_z_arg,
            workspace_roll_arg,
            workspace_pitch_arg,
            workspace_yaw_arg,
            # Display args
            use_rviz_arg,
            rviz_config_arg,
            # t=0
            robot_bringup,
            moveit_stack,
            camera_align_tf,
            camera_launch,
            apriltag_node,
            workspace_board_node,
            cube_pose_node,
            # t=3
            TimerAction(period=3.0, actions=[workspace_scene]),
            # t=4
            TimerAction(period=4.0, actions=[coordinate_listener, pick_place_sequencer]),
            # t=5
            TimerAction(period=5.0, actions=[moveit_bridge, selected_cube_adapter, pick_place_service]),
            # t=6
            TimerAction(period=6.0, actions=[
                LogInfo(msg="[hybrid] t=6s: launching RViz"),
                rviz,
            ]),
        ]
    )
