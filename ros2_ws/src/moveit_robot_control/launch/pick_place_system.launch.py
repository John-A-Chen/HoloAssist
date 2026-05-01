from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    workspace_scene_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), "/workspace_scene.launch.py"]
        ),
        launch_arguments={
            "frame_id": LaunchConfiguration("frame_id"),
            "publish_table_mesh": LaunchConfiguration("publish_table_mesh"),
            "apply_table_collision": LaunchConfiguration("apply_table_collision"),
        }.items(),
    )

    coordinate_listener_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), "/coordinate_listener.launch.py"]
        ),
        launch_arguments={
            "move_group_name": LaunchConfiguration("move_group_name"),
            "ee_link": LaunchConfiguration("ee_link"),
            "frame": LaunchConfiguration("frame"),
            "allow_pose_goal_fallback": LaunchConfiguration(
                "allow_pose_goal_fallback"
            ),
            "orientation_mode": LaunchConfiguration("orientation_mode"),
            "avoid_flange_forearm_clamp": LaunchConfiguration(
                "avoid_flange_forearm_clamp"
            ),
        }.items(),
    )

    pick_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/pick_place.launch.py"]),
        launch_arguments={
            "frame_id": LaunchConfiguration("frame_id"),
            "initial_mode": LaunchConfiguration("initial_mode"),
            "orientation_mode": LaunchConfiguration("orientation_mode"),
            "block_id": LaunchConfiguration("block_id"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "frame_id",
                default_value="base_link",
                description="Frame used by the workspace scene and pick-place sequencer.",
            ),
            DeclareLaunchArgument(
                "publish_table_mesh",
                default_value="true",
                description="Publish the trolley/table mesh as an RViz marker.",
            ),
            DeclareLaunchArgument(
                "apply_table_collision",
                default_value="false",
                description="Add the table collision box to MoveIt.",
            ),
            DeclareLaunchArgument(
                "move_group_name",
                default_value="ur_onrobot_manipulator",
                description="MoveIt planning group used for coordinate goals.",
            ),
            DeclareLaunchArgument(
                "ee_link",
                default_value="gripper_tcp",
                description="End-effector link used for coordinate goals.",
            ),
            DeclareLaunchArgument(
                "frame",
                default_value="base_link",
                description="Planning frame for coordinate goals.",
            ),
            DeclareLaunchArgument(
                "allow_pose_goal_fallback",
                default_value="true",
                description="Allow pose-goal planning fallback when Cartesian planning fails.",
            ),
            DeclareLaunchArgument(
                "orientation_mode",
                default_value="auto",
                description="Orientation mode for the coordinate listener and pick-place sequencer.",
            ),
            DeclareLaunchArgument(
                "avoid_flange_forearm_clamp",
                default_value="true",
                description="Reject predicted UR flange-to-forearm clamp routes.",
            ),
            DeclareLaunchArgument(
                "initial_mode",
                default_value="run",
                description="Initial pick-place sequencer mode: run or stop.",
            ),
            DeclareLaunchArgument(
                "block_id",
                default_value="block_1",
                description="Collision/marker object id for the block.",
            ),
            workspace_scene_launch,
            coordinate_listener_launch,
            pick_place_launch,
        ]
    )
