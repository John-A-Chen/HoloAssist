from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    scene_pkg = FindPackageShare("holo_assist_depth_tracker")
    sim_pkg = FindPackageShare("holo_assist_depth_tracker_sim")

    default_model = PathJoinSubstitution([scene_pkg, "worlds", "urdf", "holoassist_scene.urdf.xacro"])
    default_rviz = PathJoinSubstitution([sim_pkg, "rviz", "holoassist_sim.rviz"])
    sim_scene_default = PathJoinSubstitution([sim_pkg, "config", "sim_scene.yaml"])
    sim_camera_default = PathJoinSubstitution([sim_pkg, "config", "sim_camera.yaml"])
    sim_cubes_default = PathJoinSubstitution([sim_pkg, "config", "sim_cubes.yaml"])

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    publish_scene_state_publisher_arg = DeclareLaunchArgument(
        "publish_scene_state_publisher",
        default_value="true",
        description=(
            "Publish the standalone holoassist scene robot_description. "
            "Set false when a MoveIt robot_description is already running."
        ),
    )
    urdf_model_arg = DeclareLaunchArgument("urdf_model", default_value=default_model)
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value=default_rviz)
    sim_scene_arg = DeclareLaunchArgument("sim_scene_config", default_value=sim_scene_default)
    sim_camera_arg = DeclareLaunchArgument("sim_camera_config", default_value=sim_camera_default)
    sim_cubes_arg = DeclareLaunchArgument("sim_cubes_config", default_value=sim_cubes_default)

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            LaunchConfiguration("urdf_model"),
            " enable_cubes:=false",
            " enable_camera:=false",
        ]
    )

    state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="holoassist_scene_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(LaunchConfiguration("publish_scene_state_publisher")),
    )

    truth_node = Node(
        package="holo_assist_depth_tracker_sim",
        executable="sim_cube_truth_node",
        name="holoassist_sim_cube_truth",
        output="screen",
        parameters=[
            LaunchConfiguration("sim_scene_config"),
            LaunchConfiguration("sim_camera_config"),
            LaunchConfiguration("sim_cubes_config"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="holoassist_sim_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            use_sim_time_arg,
            publish_scene_state_publisher_arg,
            urdf_model_arg,
            rviz_config_arg,
            sim_scene_arg,
            sim_camera_arg,
            sim_cubes_arg,
            state_pub,
            truth_node,
            rviz,
        ]
    )
