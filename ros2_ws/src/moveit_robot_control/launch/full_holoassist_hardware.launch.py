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


def generate_launch_description() -> LaunchDescription:
    moveit_robot_control_pkg = FindPackageShare("moveit_robot_control")
    perception_pkg = FindPackageShare("holo_assist_depth_tracker")
    moveit_pkg = FindPackageShare("ur_onrobot_moveit_config")
    robot_control_pkg = FindPackageShare("ur_onrobot_control")

    hw_config_default = PathJoinSubstitution(
        [moveit_robot_control_pkg, "config", "full_holoassist_hw.yaml"]
    )
    rviz_default = PathJoinSubstitution(
        [FindPackageShare("holo_assist_depth_tracker_sim"), "rviz", "holoassist_moveit_full.rviz"]
    )

    robot_launch = PathJoinSubstitution([robot_control_pkg, "launch", "start_robot.launch.py"])
    moveit_launch = PathJoinSubstitution(
        [moveit_pkg, "launch", LaunchConfiguration("moveit_launch_file")]
    )
    perception_launch = PathJoinSubstitution(
        [perception_pkg, "launch", "holoassist_4tag_board_4cube.launch.py"]
    )
    coordinate_listener_launch = PathJoinSubstitution(
        [moveit_robot_control_pkg, "launch", "coordinate_listener.launch.py"]
    )

    # ── Declared arguments ───────────────────────────────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        description="IP address of the UR3e robot.",
    )
    ur_type_arg = DeclareLaunchArgument("ur_type", default_value="ur3e")
    onrobot_type_arg = DeclareLaunchArgument("onrobot_type", default_value="rg2")

    moveit_launch_file_arg = DeclareLaunchArgument(
        "moveit_launch_file",
        default_value="ur_onrobot_moveit.launch.py",
    )
    move_group_name_arg = DeclareLaunchArgument(
        "move_group_name", default_value="ur_onrobot_manipulator"
    )
    ee_link_arg = DeclareLaunchArgument("ee_link", default_value="gripper_tcp")
    frame_arg = DeclareLaunchArgument("frame", default_value="base_link")

    hw_config_arg = DeclareLaunchArgument(
        "hw_config",
        default_value=hw_config_default,
        description="Hardware tuning config (trolley mesh, hover offsets).",
    )

    velocity_scale_arg = DeclareLaunchArgument(
        "velocity_scale",
        default_value="0.05",
        description="Trajectory velocity scale 0.0–1.0. Keep low during initial hardware bringup.",
    )
    orientation_mode_arg = DeclareLaunchArgument(
        "orientation_mode", default_value="auto"
    )
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
    start_camera_arg = DeclareLaunchArgument(
        "start_camera",
        default_value="true",
        description="Launch the RealSense camera node inside the perception stack.",
    )
    start_rosbridge_arg = DeclareLaunchArgument(
        "start_rosbridge",
        default_value="true",
        description="Launch rosbridge WebSocket server for Unity/HoloLens (port 9090).",
    )
    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port",
        default_value="9090",
        description="WebSocket port rosbridge listens on.",
    )
    start_moveit_arg = DeclareLaunchArgument("start_moveit", default_value="true")

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value=rviz_default)

    # ── UR3e + OnRobot robot driver ───────────────────────────────────────────
    # Starts ur_ros2_control_node, controller_manager, RSP, dashboard client,
    # controller_stopper, tool_communication (serial → /tmp/ttyUR for OnRobot).
    robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "onrobot_type": LaunchConfiguration("onrobot_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_fake_hardware": "false",
            "launch_rviz": "false",          # we launch our own RViz below
            "activate_joint_controller": "true",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )

    # ── MoveIt: move_group + OMPL + SRDF ─────────────────────────────────────
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "onrobot_type": LaunchConfiguration("onrobot_type"),
            "use_fake_hardware": "false",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "launch_rviz": "false",
            "launch_servo": "false",
        }.items(),
    )

    # ── AprilTag perception pipeline ──────────────────────────────────────────
    # Starts: (optionally) RealSense camera, apriltag_ros, workspace_board_node
    # (which publishes workspace_frame TF from board corner tags), cube_pose_node
    # (which publishes /holoassist/perception/april_cube_{1-4}_pose).
    # NOTE: workspace_frame_tf static broadcaster is NOT started here — the board
    # node dynamically solves and broadcasts workspace_frame from AprilTag detections.
    perception_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch),
        launch_arguments={
            "start_camera": LaunchConfiguration("start_camera"),
        }.items(),
    )

    # ── Workspace collision scene (trolley mesh visual) ───────────────────────
    workspace_scene = Node(
        package="moveit_robot_control",
        executable="workspace_scene_manager",
        name="workspace_scene_manager",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("hw_config")],
    )

    # ── Coordinate listener: topic-driven MoveIt goals → trajectory execution ─
    # Uses scaled_joint_trajectory_controller (real UR) and enables all hardware
    # safety checks (require_robot_status, require_controller_check).
    coordinate_listener = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(coordinate_listener_launch),
        launch_arguments={
            "move_group_name": LaunchConfiguration("move_group_name"),
            "ee_link": LaunchConfiguration("ee_link"),
            "frame": LaunchConfiguration("frame"),
            "require_robot_status": "true",
            "require_controller_check": "true",
            "allow_pose_goal_fallback": "true",
            "orientation_mode": LaunchConfiguration("orientation_mode"),
            "avoid_flange_forearm_clamp": LaunchConfiguration("avoid_flange_forearm_clamp"),
            "pose_goal_planning_time": LaunchConfiguration("pose_goal_planning_time"),
            "velocity_scale": LaunchConfiguration("velocity_scale"),
            # Real UR uses scaled_joint_trajectory_controller.
            "trajectory_topic": "/scaled_joint_trajectory_controller/joint_trajectory",
        }.items(),
    )

    # ── Pick-and-place sequencer ──────────────────────────────────────────────
    # Hardware: cube_pose_node reports 3-D cube centres in workspace_frame (same
    # convention as sim truth node), so grasp_z_offset stays at 0.0. Tune if
    # the gripper consistently misses by a fixed vertical amount.
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

    # ── Pick-cube-to-bin service ──────────────────────────────────────────────
    # Reads live cube poses from the real perception pipeline
    # (/holoassist/perception/april_cube_{1-4}_pose) and converts a service call
    # into a pick_place_sequencer command.
    pick_place_service = Node(
        package="holo_assist_depth_tracker_sim",
        executable="pick_place_service_node",
        name="holoassist_pick_place_service",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_pick_place")),
        parameters=[{"cube_pose_topic_prefix": "/holoassist/perception"}],
    )

    # ── Selected-cube → MoveIt target adapter (teleop / HoloLens mode) ───────
    # Subscribes to /holoassist/teleop/selected_cube{,_pose} — published by an
    # external selector (HoloLens, RViz panel, etc.) — and continuously keeps
    # the robot hovering above the selected cube. Leave unused if only the
    # pick_place_service is needed.
    selected_cube_adapter = Node(
        package="holo_assist_depth_tracker_sim",
        executable="selected_cube_to_moveit_target_node",
        name="holoassist_selected_cube_to_moveit_target",
        output="screen",
        parameters=[
            LaunchConfiguration("hw_config"),
            {
                "target_frame": LaunchConfiguration("frame"),
                "output_point_topic": "/moveit_robot_control/target_point",
                "output_pose_topic": "/moveit_robot_control/target_pose",
            },
        ],
    )

    # ── rosbridge WebSocket server ────────────────────────────────────────────
    # Unity/HoloLens connects to ws://<host>:9090 and subscribes to cube pose topics.
    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("rosbridge_port"),
            "address": "",
            "ssl": False,
            "authenticate": False,
        }],
        condition=IfCondition(LaunchConfiguration("start_rosbridge")),
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="holoassist_moveit_hw_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # Startup sequencing:
    #  t=0s   robot_stack    — UR driver takes a few seconds to handshake
    #  t=0s   moveit_stack   — move_group waits for controller_manager + joint_states
    #  t=0s   perception     — independent; camera + AprilTag pipeline
    #  t=8s   workspace_scene — needs TF to be up (robot_state_publisher)
    #  t=10s  coordinate_listener + pick_place_sequencer
    #                         — needs move_group + scaled_joint_trajectory_controller active
    #  t=12s  pick_place_service + selected_cube_adapter
    #                         — needs cube pose topics + TF
    #  t=15s  RViz

    return LaunchDescription(
        [
            robot_ip_arg,
            ur_type_arg,
            onrobot_type_arg,
            moveit_launch_file_arg,
            move_group_name_arg,
            ee_link_arg,
            frame_arg,
            hw_config_arg,
            velocity_scale_arg,
            orientation_mode_arg,
            avoid_flange_forearm_clamp_arg,
            pose_goal_planning_time_arg,
            start_pick_place_arg,
            start_camera_arg,
            start_rosbridge_arg,
            rosbridge_port_arg,
            start_moveit_arg,
            use_rviz_arg,
            rviz_config_arg,
            robot_stack,
            moveit_stack,
            perception_stack,
            TimerAction(period=8.0, actions=[workspace_scene]),
            TimerAction(period=10.0, actions=[coordinate_listener, pick_place_sequencer]),
            TimerAction(period=12.0, actions=[pick_place_service, selected_cube_adapter]),
            rosbridge,
            TimerAction(period=15.0, actions=[
                LogInfo(msg="[hw] t=15s: launching RViz"),
                rviz,
            ]),
        ]
    )
