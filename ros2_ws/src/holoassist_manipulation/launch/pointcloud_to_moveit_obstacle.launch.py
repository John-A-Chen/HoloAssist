from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/holo_assist_depth_tracker/pointcloud",
            ),
            DeclareLaunchArgument("planning_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "collision_object_id",
                default_value="depth_pointcloud_obstacle",
            ),
            DeclareLaunchArgument("max_range_m", default_value="2.0"),
            DeclareLaunchArgument("padding_m", default_value="0.05"),
            DeclareLaunchArgument("min_size_m", default_value="0.08"),
            DeclareLaunchArgument("min_points", default_value="150"),
            DeclareLaunchArgument("update_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("sample_step", default_value="1"),
            Node(
                package="holoassist_manipulation",
                executable="pointcloud_to_moveit_obstacle",
                name="pointcloud_to_moveit_obstacle",
                output="screen",
                parameters=[
                    {
                        "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                        "planning_frame": LaunchConfiguration("planning_frame"),
                        "collision_object_id": LaunchConfiguration(
                            "collision_object_id"
                        ),
                        "max_range_m": LaunchConfiguration("max_range_m"),
                        "padding_m": LaunchConfiguration("padding_m"),
                        "min_size_m": LaunchConfiguration("min_size_m"),
                        "min_points": LaunchConfiguration("min_points"),
                        "update_rate_hz": LaunchConfiguration("update_rate_hz"),
                        "sample_step": LaunchConfiguration("sample_step"),
                    }
                ],
            ),
        ]
    )
