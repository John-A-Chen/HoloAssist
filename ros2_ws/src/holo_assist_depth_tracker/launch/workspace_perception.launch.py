from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory("holo_assist_depth_tracker"),
        "config",
        "workspace_perception_params.yaml",
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to workspace perception parameter YAML",
    )

    workspace_node = Node(
        package="holo_assist_depth_tracker",
        executable="workspace_perception_node",
        name="holoassist_workspace_perception",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription(
        [
            params_arg,
            workspace_node,
        ]
    )
