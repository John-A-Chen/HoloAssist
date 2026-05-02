from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "moveit_robot_control"


def default_bin_config_path() -> str:
    share_dir = Path(get_package_share_directory(PACKAGE_NAME))
    return str(share_dir / "config" / "bin_poses.json")


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "frame_id",
                default_value="base_link",
                description="Frame used for incoming block poses.",
            ),
            DeclareLaunchArgument(
                "block_pose_topic",
                default_value="/pick_place/block_pose",
                description="geometry_msgs/msg/PoseStamped block pose input.",
            ),
            DeclareLaunchArgument(
                "target_point_topic",
                default_value="/moveit_robot_control/target_point",
                description="geometry_msgs/msg/Point output for auto orientation mode.",
            ),
            DeclareLaunchArgument(
                "command_topic",
                default_value="/pick_place/command",
                description="std_msgs/msg/String JSON pick-place command input.",
            ),
            DeclareLaunchArgument(
                "mode_topic",
                default_value="/pick_place/mode",
                description="std_msgs/msg/String run or stop mode input.",
            ),
            DeclareLaunchArgument(
                "place_xyz",
                default_value="[0.45, -0.10, 0.05]",
                description="Default drop block center as [x, y, z].",
            ),
            DeclareLaunchArgument(
                "bin_config_path",
                default_value=default_bin_config_path(),
                description="JSON file containing bin xyz/rpy_deg definitions.",
            ),
            DeclareLaunchArgument(
                "default_bin_id",
                default_value="",
                description="Default bin for PoseStamped inputs. Empty uses place_xyz.",
            ),
            DeclareLaunchArgument(
                "item_bin_map",
                default_value="{}",
                description="JSON map from item names to bin ids.",
            ),
            DeclareLaunchArgument(
                "block_id",
                default_value="block_1",
                description="Collision/marker object id for the block.",
            ),
            DeclareLaunchArgument(
                "block_size",
                default_value="[0.05, 0.05, 0.05]",
                description="Block dimensions as [x, y, z].",
            ),
            DeclareLaunchArgument(
                "pregrasp_z_offset",
                default_value="0.11",
                description="Height above block center for the approach pose.",
            ),
            DeclareLaunchArgument(
                "grasp_z_offset",
                default_value="0.05",
                description="Height above block center for the grasp pose.",
            ),
            DeclareLaunchArgument(
                "place_above_z_offset",
                default_value="0.17",
                description="Height above place center before releasing.",
            ),
            DeclareLaunchArgument(
                "place_z_offset",
                default_value="0.05",
                description="Height above place center for release.",
            ),
            DeclareLaunchArgument(
                "place_descent_enabled",
                default_value="false",
                description="Move down from above the bin before opening the gripper.",
            ),
            DeclareLaunchArgument(
                "orientation_mode",
                default_value="auto",
                description="Use auto or fixed end-effector orientation for moves.",
            ),
            DeclareLaunchArgument(
                "initial_mode",
                default_value="stop",
                description="Initial sequencer mode: run or stop.",
            ),
            DeclareLaunchArgument(
                "open_width",
                default_value="0.08",
                description="RG2 opening width in meters.",
            ),
            DeclareLaunchArgument(
                "close_width",
                default_value="0.0",
                description="RG2 closing width in meters.",
            ),
            Node(
                package="moveit_robot_control",
                executable="pick_place_sequencer",
                name="pick_place_sequencer",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "frame_id": LaunchConfiguration("frame_id"),
                        "block_pose_topic": LaunchConfiguration("block_pose_topic"),
                        "target_point_topic": LaunchConfiguration("target_point_topic"),
                        "command_topic": LaunchConfiguration("command_topic"),
                        "mode_topic": LaunchConfiguration("mode_topic"),
                        "place_xyz": LaunchConfiguration("place_xyz"),
                        "bin_config_path": LaunchConfiguration("bin_config_path"),
                        "default_bin_id": LaunchConfiguration("default_bin_id"),
                        "item_bin_map": ParameterValue(
                            LaunchConfiguration("item_bin_map"),
                            value_type=str,
                        ),
                        "block_id": LaunchConfiguration("block_id"),
                        "block_size": LaunchConfiguration("block_size"),
                        "pregrasp_z_offset": ParameterValue(
                            LaunchConfiguration("pregrasp_z_offset"),
                            value_type=float,
                        ),
                        "grasp_z_offset": ParameterValue(
                            LaunchConfiguration("grasp_z_offset"),
                            value_type=float,
                        ),
                        "place_above_z_offset": ParameterValue(
                            LaunchConfiguration("place_above_z_offset"),
                            value_type=float,
                        ),
                        "place_z_offset": ParameterValue(
                            LaunchConfiguration("place_z_offset"),
                            value_type=float,
                        ),
                        "place_descent_enabled": ParameterValue(
                            LaunchConfiguration("place_descent_enabled"),
                            value_type=bool,
                        ),
                        "orientation_mode": LaunchConfiguration("orientation_mode"),
                        "initial_mode": LaunchConfiguration("initial_mode"),
                        "open_width": ParameterValue(
                            LaunchConfiguration("open_width"),
                            value_type=float,
                        ),
                        "close_width": ParameterValue(
                            LaunchConfiguration("close_width"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
