from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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


def generate_gazebo_bringup(context, *args, **kwargs):
    """Build robot_description with sim_gazebo=true and start robot_state_publisher."""
    ur_type = LaunchConfiguration("ur_type").perform(context)
    onrobot_type = LaunchConfiguration("onrobot_type").perform(context)
    robot_base_yaw_rad = LaunchConfiguration("robot_base_yaw_rad").perform(context)

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("ur_onrobot_description"), "urdf", "ur_onrobot.urdf.xacro"]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ", xacro_file, " ",
            "ur_type:=", ur_type, " ",
            "onrobot_type:=", onrobot_type, " ",
            "name:=ur_onrobot", " ",
            "sim_gazebo:=true", " ",
            "robot_ip:=0.0.0.0", " ",
            "base_yaw_rad:=", robot_base_yaw_rad,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    return [robot_state_publisher]


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
    world_default = PathJoinSubstitution(
        [sim_pkg, "worlds", "holoassist_gazebo_sim.world"]
    )

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
    start_pick_place_arg = DeclareLaunchArgument(
        "start_pick_place",
        default_value="true",
        description="Launch pick_place_sequencer and pick_place_service_node.",
    )
    robot_base_yaw_rad_arg = DeclareLaunchArgument(
        "robot_base_yaw_rad",
        default_value="3.14159",
        description="Yaw of the world→base mounting joint in radians.",
    )
    moveit_launch_file_arg = DeclareLaunchArgument(
        "moveit_launch_file",
        default_value="ur_onrobot_moveit.launch.py",
    )
    ur_type_arg = DeclareLaunchArgument("ur_type", default_value="ur3e")
    onrobot_type_arg = DeclareLaunchArgument("onrobot_type", default_value="rg2")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value=rviz_default)

    use_gazebo_gui_arg = DeclareLaunchArgument(
        "use_gazebo_gui",
        default_value="true",
        description="Launch the Gazebo GUI (gzclient). Set false for headless.",
    )
    gazebo_world_arg = DeclareLaunchArgument(
        "gazebo_world",
        default_value=world_default,
        description="Path to the Gazebo .world file.",
    )

    full_sim_config_arg = DeclareLaunchArgument(
        "full_sim_config",
        default_value=full_sim_config_default,
    )
    sim_scene_arg = DeclareLaunchArgument("sim_scene_config", default_value=sim_scene_default)
    sim_camera_arg = DeclareLaunchArgument("sim_camera_config", default_value=sim_camera_default)
    sim_cubes_arg = DeclareLaunchArgument("sim_cubes_config", default_value=sim_cubes_default)

    move_group_name_arg = DeclareLaunchArgument(
        "move_group_name", default_value="ur_onrobot_manipulator"
    )
    ee_link_arg = DeclareLaunchArgument("ee_link", default_value="gripper_tcp")
    frame_arg = DeclareLaunchArgument("frame", default_value="base_link")
    require_robot_status_arg = DeclareLaunchArgument(
        "require_robot_status", default_value="false"
    )
    velocity_scale_arg = DeclareLaunchArgument("velocity_scale", default_value="0.05")
    orientation_mode_arg = DeclareLaunchArgument(
        "orientation_mode", default_value="auto"
    )
    avoid_flange_forearm_clamp_arg = DeclareLaunchArgument(
        "avoid_flange_forearm_clamp", default_value="true"
    )
    pose_goal_planning_time_arg = DeclareLaunchArgument(
        "pose_goal_planning_time", default_value="5.0"
    )

    # ── Gazebo ───────────────────────────────────────────────────────────────
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver", "--verbose",
            LaunchConfiguration("gazebo_world"),
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gazebo_gui")),
    )

    # ── Robot bringup ─────────────────────────────────────────────────────────
    robot_bringup = OpaqueFunction(function=generate_gazebo_bringup)

    # Spawn the robot URDF into Gazebo from the /robot_description topic.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur_onrobot",
        arguments=["-topic", "/robot_description", "-entity", "ur_onrobot"],
        output="screen",
    )

    # Spawn controllers once Gazebo's controller_manager plugin is up.
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "joint_state_broadcaster",
            "joint_trajectory_controller",
            "finger_width_trajectory_controller",
        ],
        output="screen",
    )

    # ── MoveIt ───────────────────────────────────────────────────────────────
    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "onrobot_type": LaunchConfiguration("onrobot_type"),
            # MoveIt only needs the kinematic structure; fake_hardware flag
            # doesn't affect planning, and avoids Gazebo plugin duplication.
            "use_fake_hardware": "true",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "launch_rviz": "false",
            "launch_servo": "false",
            "base_yaw_rad": LaunchConfiguration("robot_base_yaw_rad"),
        }.items(),
    )

    # ── Static workspace TF ───────────────────────────────────────────────────
    workspace_tf = Node(
        package="moveit_robot_control",
        executable="workspace_frame_tf",
        name="holoassist_workspace_frame_tf",
        output="screen",
        parameters=[LaunchConfiguration("full_sim_config")],
    )

    # ── Workspace scene ───────────────────────────────────────────────────────
    workspace_scene = Node(
        package="moveit_robot_control",
        executable="workspace_scene_manager",
        name="workspace_scene_manager",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("full_sim_config")],
    )

    # ── Coordinate listener ───────────────────────────────────────────────────
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
            # Gazebo uses joint_trajectory_controller (same as fake hardware sim).
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

    # ── Pick-cube-to-bin service ──────────────────────────────────────────────
    pick_place_service = Node(
        package="holo_assist_depth_tracker_sim",
        executable="pick_place_service_node",
        name="holoassist_pick_place_service",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_pick_place")),
    )

    # ── Perception sim (truth cubes + fake camera) ────────────────────────────
    perception_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch),
        launch_arguments={
            "use_rviz": "false",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "sim_scene_config": LaunchConfiguration("sim_scene_config"),
            "sim_camera_config": LaunchConfiguration("sim_camera_config"),
            "sim_cubes_config": LaunchConfiguration("sim_cubes_config"),
            "publish_scene_state_publisher": "false",
        }.items(),
    )

    # ── MoveIt planning scene bridge ──────────────────────────────────────────
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

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="holoassist_moveit_full_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # Startup sequencing:
    #  t=0s  gzserver + gzclient + robot_state_publisher
    #  t=5s  spawn_entity      — wait for Gazebo to be ready
    #  t=7s  controller_spawner — spawner blocks until /controller_manager is up (30s timeout)
    #  t=8s  moveit_stack       — needs joint_states (from Gazebo) + RSP
    #  t=11s workspace_tf + workspace_scene
    #  t=12s coordinate_listener + pick_place_sequencer
    #  t=14s perception_stack
    #  t=15s moveit_bridge + selected_cube_adapter + pick_place_service
    #  t=16s rviz

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
            use_gazebo_gui_arg,
            gazebo_world_arg,
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
            gzserver,
            gzclient,
            robot_bringup,
            TimerAction(period=5.0, actions=[spawn_entity]),
            TimerAction(period=7.0, actions=[controller_spawner]),
            TimerAction(period=8.0, actions=[moveit_stack]),
            TimerAction(period=11.0, actions=[workspace_tf, workspace_scene]),
            TimerAction(period=12.0, actions=[coordinate_listener, pick_place_sequencer]),
            TimerAction(period=14.0, actions=[perception_stack]),
            TimerAction(period=15.0, actions=[moveit_bridge, selected_cube_adapter, pick_place_service]),
            TimerAction(period=16.0, actions=[
                LogInfo(msg="[gazebo_sim] t=16s: launching RViz"),
                rviz,
            ]),
        ]
    )
