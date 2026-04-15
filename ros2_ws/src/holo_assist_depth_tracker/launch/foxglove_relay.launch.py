from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "[holo_assist_depth_tracker] foxglove_relay.launch.py is the "
                    "forward-looking alias for dashboard_bridge.launch.py."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("holo_assist_depth_tracker"),
                            "launch",
                            "dashboard_bridge.launch.py",
                        ]
                    )
                )
            ),
        ]
    )
