from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    sim_pkg = FindPackageShare("holo_assist_depth_tracker_sim")

    perception_launch = PathJoinSubstitution([sim_pkg, "launch", "sim_april_cube_perception.launch.py"])
    sim_scene_default = PathJoinSubstitution([sim_pkg, "config", "sim_scene.yaml"])
    sim_camera_default = PathJoinSubstitution([sim_pkg, "config", "sim_camera.yaml"])
    sim_cubes_default = PathJoinSubstitution([sim_pkg, "config", "sim_cubes.yaml"])

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    sim_scene_arg = DeclareLaunchArgument("sim_scene_config", default_value=sim_scene_default)
    sim_camera_arg = DeclareLaunchArgument("sim_camera_config", default_value=sim_camera_default)
    sim_cubes_arg = DeclareLaunchArgument("sim_cubes_config", default_value=sim_cubes_default)
    publish_scene_state_publisher_arg = DeclareLaunchArgument(
        "publish_scene_state_publisher",
        default_value="true",
        description=(
            "Publish the standalone holoassist scene robot_description. "
            "Set false when a MoveIt robot_description is already running."
        ),
    )
    moveit_target_frame_arg = DeclareLaunchArgument("moveit_target_frame", default_value="base_link")
    moveit_target_topic_arg = DeclareLaunchArgument(
        "moveit_target_topic", default_value="/moveit_robot_control/target_point"
    )
    moveit_target_pose_topic_arg = DeclareLaunchArgument(
        "moveit_target_pose_topic", default_value="/moveit_robot_control/target_pose"
    )
    moveit_target_topic_legacy_arg = DeclareLaunchArgument(
        "moveit_target_topic_legacy",
        default_value="",
    )

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "sim_scene_config": LaunchConfiguration("sim_scene_config"),
            "sim_camera_config": LaunchConfiguration("sim_camera_config"),
            "sim_cubes_config": LaunchConfiguration("sim_cubes_config"),
            "publish_scene_state_publisher": LaunchConfiguration(
                "publish_scene_state_publisher"
            ),
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
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "target_frame": LaunchConfiguration("moveit_target_frame"),
                "output_point_topic": LaunchConfiguration("moveit_target_topic"),
                "output_pose_topic": LaunchConfiguration("moveit_target_pose_topic"),
                "output_point_topic_legacy": LaunchConfiguration("moveit_target_topic_legacy"),
            },
        ],
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            use_sim_time_arg,
            sim_scene_arg,
            sim_camera_arg,
            sim_cubes_arg,
            publish_scene_state_publisher_arg,
            moveit_target_frame_arg,
            moveit_target_topic_arg,
            moveit_target_pose_topic_arg,
            moveit_target_topic_legacy_arg,
            base_stack,
            moveit_bridge,
            selected_cube_adapter,
        ]
    )
