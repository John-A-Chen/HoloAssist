from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
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


def generate_robot_description(context, *args, **kwargs):
    """Build robot_description from xacro with current launch config values."""
    ur_type = LaunchConfiguration("ur_type").perform(context)
    onrobot_type = LaunchConfiguration("onrobot_type").perform(context)
    robot_base_yaw_rad = LaunchConfiguration("robot_base_yaw_rad").perform(context)

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("ur_onrobot_description"), "urdf", "ur_onrobot.urdf.xacro"]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "ur_type:=", ur_type,
            " ",
            "onrobot_type:=", onrobot_type,
            " ",
            "name:=ur_onrobot",
            " ",
            "use_fake_hardware:=true",
            " ",
            # Dummy IP — not used by fake hardware but required by the URDF.
            "robot_ip:=0.0.0.0",
            " ",
            "base_yaw_rad:=", robot_base_yaw_rad,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    ur_update_rate_config = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "config", ur_type + "_update_rate.yaml"]
    )
    # Sim-specific controller config: plain joint names (no xacro $(var tf_prefix) substitutions)
    # so the YAML can be passed directly to ros2_control_node without xacro processing.
    controller_config = PathJoinSubstitution(
        [FindPackageShare("moveit_robot_control"), "config", "sim_controllers.yaml"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ros2_control_node with fake hardware plugin
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            ur_update_rate_config,
            controller_config,
        ],
    )

    # Spawn only the controllers that are compatible with fake hardware.
    # UR-specific controllers (io_and_status_controller, speed_scaling_state_broadcaster,
    # ur_configuration_controller, scaled_joint_trajectory_controller, etc.) require
    # real UR driver hardware interfaces and will fail with the fake hardware plugin.
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

    return [robot_state_publisher, ros2_control_node, controller_spawner]


def generate_launch_description() -> LaunchDescription:
    moveit_robot_control_pkg = FindPackageShare("moveit_robot_control")
    sim_pkg = FindPackageShare("holo_assist_depth_tracker_sim")
    moveit_pkg = FindPackageShare("ur_onrobot_moveit_config")

    full_sim_config_default = PathJoinSubstitution(
        [moveit_robot_control_pkg, "config", "full_holoassist_sim.yaml"]
    )
    sim_scene_default = PathJoinSubstitution([sim_pkg, "config", "sim_scene.yaml"])
    sim_camera_default = PathJoinSubstitution([sim_pkg, "config", "sim_camera.yaml"])
    sim_cubes_default = PathJoinSubstitution([sim_pkg, "config", "sim_cubes.yaml"])
    rviz_default = PathJoinSubstitution([sim_pkg, "rviz", "holoassist_moveit_full.rviz"])

    perception_launch = PathJoinSubstitution(
        [sim_pkg, "launch", "sim_april_cube_perception.launch.py"]
    )
    coordinate_listener_launch = PathJoinSubstitution(
        [moveit_robot_control_pkg, "launch", "coordinate_listener.launch.py"]
    )
    moveit_launch = PathJoinSubstitution(
        [moveit_pkg, "launch", LaunchConfiguration("moveit_launch_file")]
    )

    # ── Declared arguments ───────────────────────────────────────────────────
    start_moveit_arg = DeclareLaunchArgument("start_moveit", default_value="true")
    robot_base_yaw_rad_arg = DeclareLaunchArgument(
        "robot_base_yaw_rad",
        default_value="3.14159",
        description=(
            "Yaw of the world→base mounting joint in radians. "
            "Default 3.14159 (π) rotates the arm 180° to match HoloAssist trolley orientation."
        ),
    )
    moveit_launch_file_arg = DeclareLaunchArgument(
        "moveit_launch_file",
        default_value="ur_onrobot_moveit.launch.py",
        description="Launch file inside ur_onrobot_moveit_config for MoveIt bringup.",
    )
    ur_type_arg = DeclareLaunchArgument("ur_type", default_value="ur3e")
    onrobot_type_arg = DeclareLaunchArgument("onrobot_type", default_value="rg2")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    start_pick_place_arg = DeclareLaunchArgument(
        "start_pick_place",
        default_value="true",
        description="Launch pick_place_sequencer and pick_place_service_node.",
    )

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value=rviz_default)

    full_sim_config_arg = DeclareLaunchArgument(
        "full_sim_config",
        default_value=full_sim_config_default,
        description="Full-stack sim tuning file (workspace TF, trolley pose, hover offsets).",
    )
    sim_scene_arg = DeclareLaunchArgument("sim_scene_config", default_value=sim_scene_default)
    sim_camera_arg = DeclareLaunchArgument("sim_camera_config", default_value=sim_camera_default)
    sim_cubes_arg = DeclareLaunchArgument("sim_cubes_config", default_value=sim_cubes_default)

    move_group_name_arg = DeclareLaunchArgument(
        "move_group_name",
        default_value="ur_onrobot_manipulator",
    )
    ee_link_arg = DeclareLaunchArgument("ee_link", default_value="gripper_tcp")
    frame_arg = DeclareLaunchArgument("frame", default_value="base_link")
    require_robot_status_arg = DeclareLaunchArgument(
        "require_robot_status",
        default_value="false",
        description="Set false for fake hardware simulation.",
    )
    velocity_scale_arg = DeclareLaunchArgument(
        "velocity_scale",
        default_value="0.05",
        description="Trajectory velocity scale 0.0–1.0. 0.05 = 5% for safe sim testing.",
    )
    orientation_mode_arg = DeclareLaunchArgument(
        "orientation_mode",
        default_value="auto",
        description="EE orientation policy for point goals: auto, current, or fixed.",
    )
    avoid_flange_forearm_clamp_arg = DeclareLaunchArgument(
        "avoid_flange_forearm_clamp",
        default_value="true",
        description="Reject trajectories entering the UR flange-to-forearm clamp zone.",
    )
    pose_goal_planning_time_arg = DeclareLaunchArgument(
        "pose_goal_planning_time",
        default_value="5.0",
        description="Planning timeout in seconds for pose-goal fallback.",
    )

    # ── Robot bringup: robot_state_publisher + ros2_control_node + controllers ──
    # Uses OpaqueFunction so the URDF Command can be built with concrete values at
    # launch time. Only spawns controllers that work with the fake hardware plugin.
    robot_bringup = OpaqueFunction(function=generate_robot_description)

    # ── MoveIt: move_group with OMPL, SRDF, kinematics ──────────────────────
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "onrobot_type": LaunchConfiguration("onrobot_type"),
            "use_fake_hardware": "true",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "launch_rviz": "false",
            "launch_servo": "false",
            "base_yaw_rad": LaunchConfiguration("robot_base_yaw_rad"),
        }.items(),
    )

    # ── Static workspace TF (base_link → workspace_frame) ────────────────────
    workspace_tf = Node(
        package="moveit_robot_control",
        executable="workspace_frame_tf",
        name="holoassist_workspace_frame_tf",
        output="screen",
        parameters=[LaunchConfiguration("full_sim_config")],
    )

    # ── Workspace scene (trolley mesh visual + collision objects) ─────────────
    workspace_scene = Node(
        package="moveit_robot_control",
        executable="workspace_scene_manager",
        name="workspace_scene_manager",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("full_sim_config")],
    )

    # ── Coordinate listener (topic-driven MoveIt goals + trajectory publisher) ─
    # trajectory_topic points at joint_trajectory_controller, which is the controller
    # active for fake hardware (not the UR-specific scaled controller).
    coordinate_listener = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(coordinate_listener_launch),
        launch_arguments={
            "move_group_name": LaunchConfiguration("move_group_name"),
            "ee_link": LaunchConfiguration("ee_link"),
            "frame": LaunchConfiguration("frame"),
            "require_robot_status": LaunchConfiguration("require_robot_status"),
            "require_controller_check": "false",
            "allow_pose_goal_fallback": "true",
            "orientation_mode": LaunchConfiguration("orientation_mode"),
            "avoid_flange_forearm_clamp": LaunchConfiguration("avoid_flange_forearm_clamp"),
            "pose_goal_planning_time": LaunchConfiguration("pose_goal_planning_time"),
            "velocity_scale": LaunchConfiguration("velocity_scale"),
            "trajectory_topic": "/joint_trajectory_controller/joint_trajectory",
        }.items(),
    )

    # ── Perception sim (truth cubes, fake D435i camera, visibility-based perception) ─
    perception_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch),
        launch_arguments={
            "use_rviz": "false",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "sim_scene_config": LaunchConfiguration("sim_scene_config"),
            "sim_camera_config": LaunchConfiguration("sim_camera_config"),
            "sim_cubes_config": LaunchConfiguration("sim_cubes_config"),
            # Suppress the standalone scene URDF publisher; robot_state_publisher above
            # already publishes world→base_link and workspace_frame_tf adds base_link→workspace_frame.
            "publish_scene_state_publisher": "false",
        }.items(),
    )

    # ── MoveIt planning scene bridge (perceived cubes → collision objects) ────
    moveit_bridge = Node(
        package="holo_assist_depth_tracker_sim",
        executable="sim_cube_moveit_bridge_node",
        name="holoassist_sim_cube_moveit_bridge",
        output="screen",
        parameters=[
            LaunchConfiguration("sim_scene_config"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # ── Selected-cube → MoveIt target adapter ────────────────────────────────
    # Reads perceived cube pose in workspace_frame, TF-transforms to base_link,
    # applies hover offset, publishes target_point and target_pose.
    selected_cube_adapter = Node(
        package="holo_assist_depth_tracker_sim",
        executable="selected_cube_to_moveit_target_node",
        name="holoassist_selected_cube_to_moveit_target",
        output="screen",
        parameters=[
            LaunchConfiguration("sim_scene_config"),
            LaunchConfiguration("full_sim_config"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "target_frame": LaunchConfiguration("frame"),
                "output_point_topic": "/moveit_robot_control/target_point",
                "output_pose_topic": "/moveit_robot_control/target_pose",
            },
        ],
    )

    # ── Pick-and-place sequencer (Ollie's bin-aware pick/drop state machine) ──
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
            # Sim cubes have their center at Z=0.020 m (half of 4 cm cube).
            # Offsets are added to the reported cube center Z, so these are
            # calibrated for center-reported positions rather than top-face AprilTags.
            "pregrasp_z_offset": 0.10,   # approach 10 cm above cube centre
            "grasp_z_offset": 0.0,       # descend to cube centre height
            "place_above_z_offset": 0.15,
            "place_z_offset": 0.05,
            "place_descent_enabled": True,
        }],
    )

    # ── Pick-cube-to-bin service (service → sequencer command bridge) ─────────
    pick_place_service = Node(
        package="holo_assist_depth_tracker_sim",
        executable="pick_place_service_node",
        name="holoassist_pick_place_service",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_pick_place")),
        parameters=[{"cube_pose_topic_prefix": "/holoassist/sim/truth"}],
    )

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="holoassist_moveit_full_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # Startup sequencing (all delays relative to launch start):
    #  t=0s  robot_bringup  → robot_state_publisher, ros2_control_node
    #                          controller spawner starts immediately but may take a few seconds
    #  t=0s  moveit_stack   → move_group needs robot_description and joint_states
    #  t=3s  workspace_tf   → needs base_link in TF (robot_state_publisher should be up by now)
    #  t=3s  workspace_scene
    #  t=4s  coordinate_listener → needs move_group planning service to be up
    #  t=4s  pick_place_sequencer → needs move_group and coordinate_listener
    #  t=5s  perception_stack   → independent; fake camera/cube truth sim
    #  t=6s  moveit_bridge + adapter → need perception topics
    #  t=6s  pick_place_service   → needs TF, cube truth poses, and sequencer
    #  t=7s  rviz               → all TF and topics should be settled

    return LaunchDescription(
        [
            start_moveit_arg,
            start_pick_place_arg,
            robot_base_yaw_rad_arg,
            moveit_launch_file_arg,
            ur_type_arg,
            onrobot_type_arg,
            use_sim_time_arg,
            use_rviz_arg,
            rviz_config_arg,
            full_sim_config_arg,
            sim_scene_arg,
            sim_camera_arg,
            sim_cubes_arg,
            move_group_name_arg,
            ee_link_arg,
            frame_arg,
            require_robot_status_arg,
            velocity_scale_arg,
            orientation_mode_arg,
            avoid_flange_forearm_clamp_arg,
            pose_goal_planning_time_arg,
            robot_bringup,
            moveit_stack,
            TimerAction(period=3.0, actions=[workspace_tf, workspace_scene]),
            TimerAction(period=4.0, actions=[coordinate_listener, pick_place_sequencer]),
            TimerAction(period=5.0, actions=[perception_stack]),
            TimerAction(period=6.0, actions=[moveit_bridge, selected_cube_adapter, pick_place_service]),
            TimerAction(period=7.0, actions=[
                LogInfo(msg=["[full_sim] t=7s: launching RViz, use_rviz=", LaunchConfiguration("use_rviz")]),
                rviz,
            ]),
        ]
    )
