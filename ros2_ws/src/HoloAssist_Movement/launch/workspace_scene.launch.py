from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "frame_id",
                default_value="base_link",
                description="Frame used for the table mesh and default block commands.",
            ),
            DeclareLaunchArgument(
                "publish_table_mesh",
                default_value="true",
                description="Publish the trolley/table mesh as an RViz marker.",
            ),
            DeclareLaunchArgument(
                "apply_table_collision",
                default_value="false",
                description=(
                    "Add the table collision box to MoveIt. Tune the collision "
                    "pose and size before enabling on the real robot."
                ),
            ),
            DeclareLaunchArgument(
                "table_mesh_resource",
                default_value=(
                    "package://moveit_robot_control/meshes/"
                    "UR3eTrolley_decimated.dae"
                ),
                description="Mesh resource URI for the visible trolley/table.",
            ),
            DeclareLaunchArgument(
                "table_mesh_xyz",
                default_value="[0.0, 0.0, 0.0]",
                description="Visible table mesh position as [x, y, z].",
            ),
            DeclareLaunchArgument(
                "table_mesh_rpy_deg",
                default_value="[0.0, 0.0, 0.0]",
                description="Visible table mesh orientation as [roll, pitch, yaw].",
            ),
            DeclareLaunchArgument(
                "table_mesh_scale",
                default_value="[1.0, 1.0, 1.0]",
                description="Visible table mesh scale as [x, y, z].",
            ),
            DeclareLaunchArgument(
                "table_collision_xyz",
                default_value="[0.0, 0.0, 1.04]",
                description="MoveIt table collision box center as [x, y, z].",
            ),
            DeclareLaunchArgument(
                "table_collision_size",
                default_value="[0.80, 0.70, 0.04]",
                description="MoveIt table collision box dimensions as [x, y, z].",
            ),
            DeclareLaunchArgument(
                "default_block_size",
                default_value="[0.05, 0.05, 0.05]",
                description="Default block dimensions as [x, y, z].",
            ),
            Node(
                package="moveit_robot_control",
                executable="workspace_scene_manager",
                name="workspace_scene_manager",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "frame_id": LaunchConfiguration("frame_id"),
                        "publish_table_mesh": ParameterValue(
                            LaunchConfiguration("publish_table_mesh"),
                            value_type=bool,
                        ),
                        "apply_table_collision": ParameterValue(
                            LaunchConfiguration("apply_table_collision"),
                            value_type=bool,
                        ),
                        "table_mesh_resource": LaunchConfiguration(
                            "table_mesh_resource"
                        ),
                        "table_mesh_xyz": LaunchConfiguration("table_mesh_xyz"),
                        "table_mesh_rpy_deg": LaunchConfiguration(
                            "table_mesh_rpy_deg"
                        ),
                        "table_mesh_scale": LaunchConfiguration("table_mesh_scale"),
                        "table_collision_xyz": LaunchConfiguration(
                            "table_collision_xyz"
                        ),
                        "table_collision_size": LaunchConfiguration(
                            "table_collision_size"
                        ),
                        "default_block_size": LaunchConfiguration(
                            "default_block_size"
                        ),
                    }
                ],
            ),
        ]
    )
