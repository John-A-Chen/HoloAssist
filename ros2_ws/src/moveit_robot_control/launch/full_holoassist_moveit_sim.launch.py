from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    start_moveit_arg = DeclareLaunchArgument("start_moveit", default_value="true")
    moveit_launch_file_arg = DeclareLaunchArgument(
        "moveit_launch_file",
        default_value="ur_onrobot_moveit.launch.py",
        description="Launch file inside ur_onrobot_moveit_config used for fake hardware MoveIt bringup.",
    )
    ur_type_arg = DeclareLaunchArgument("ur_type", default_value="ur3e")
    onrobot_type_arg = DeclareLaunchArgument("onrobot_type", default_value="rg2")
    use_fake_hardware_arg = DeclareLaunchArgument("use_fake_hardware", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value=rviz_default)

    full_sim_config_arg = DeclareLaunchArgument(
        "full_sim_config",
        default_value=full_sim_config_default,
        description="Full-stack sim tuning file (workspace TF, trolley mesh pose, target hover offsets).",
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

    moveit_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "onrobot_type": LaunchConfiguration("onrobot_type"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "launch_rviz": "false",
            "rviz": "false",
        }.items(),
    )

    workspace_tf = Node(
        package="moveit_robot_control",
        executable="workspace_frame_tf",
        name="holoassist_workspace_frame_tf",
        output="screen",
        parameters=[LaunchConfiguration("full_sim_config")],
    )

    workspace_scene = Node(
        package="moveit_robot_control",
        executable="workspace_scene_manager",
        name="workspace_scene_manager",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("full_sim_config")],
    )

    coordinate_listener = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(coordinate_listener_launch),
        launch_arguments={
            "move_group_name": LaunchConfiguration("move_group_name"),
            "ee_link": LaunchConfiguration("ee_link"),
            "frame": LaunchConfiguration("frame"),
            "require_robot_status": LaunchConfiguration("require_robot_status"),
            "allow_pose_goal_fallback": "true",
            "orientation_mode": "auto",
            "avoid_flange_forearm_clamp": "true",
        }.items(),
    )

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

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="holoassist_moveit_full_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            start_moveit_arg,
            moveit_launch_file_arg,
            ur_type_arg,
            onrobot_type_arg,
            use_fake_hardware_arg,
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
            moveit_stack,
            workspace_tf,
            workspace_scene,
            TimerAction(period=1.0, actions=[coordinate_listener]),
            TimerAction(period=1.5, actions=[perception_stack]),
            TimerAction(period=2.0, actions=[moveit_bridge, selected_cube_adapter]),
            TimerAction(period=2.5, actions=[rviz]),
        ]
    )
