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

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    sim_scene_arg = DeclareLaunchArgument("sim_scene_config", default_value=sim_scene_default)
    sim_camera_arg = DeclareLaunchArgument("sim_camera_config", default_value=sim_camera_default)

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "sim_scene_config": LaunchConfiguration("sim_scene_config"),
            "sim_camera_config": LaunchConfiguration("sim_camera_config"),
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

    return LaunchDescription(
        [
            use_rviz_arg,
            use_sim_time_arg,
            sim_scene_arg,
            sim_camera_arg,
            base_stack,
            moveit_bridge,
        ]
    )
