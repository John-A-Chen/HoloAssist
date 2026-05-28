from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("holo_assist_depth_tracker")
    default_model = PathJoinSubstitution(
        [pkg_share, "worlds", "urdf", "holoassist_scene.urdf.xacro"]
    )
    default_rviz = PathJoinSubstitution(
        [pkg_share, "worlds", "rviz", "holoassist_scene.rviz"]
    )

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    urdf_model_arg = DeclareLaunchArgument("urdf_model", default_value=default_model)
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value=default_rviz)
    enable_cubes_arg = DeclareLaunchArgument("enable_cubes", default_value="true")
    enable_camera_arg = DeclareLaunchArgument("enable_camera", default_value="true")

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            LaunchConfiguration("urdf_model"),
            " ",
            "enable_cubes:=",
            LaunchConfiguration("enable_cubes"),
            " ",
            "enable_camera:=",
            LaunchConfiguration("enable_camera"),
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
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="holoassist_scene_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            use_sim_time_arg,
            urdf_model_arg,
            rviz_config_arg,
            enable_cubes_arg,
            enable_camera_arg,
            state_pub,
            rviz,
        ]
    )
