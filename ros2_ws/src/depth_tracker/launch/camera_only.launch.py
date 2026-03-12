from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # NOTE:
    # RealSense launch arguments can vary across realsense2_camera versions.
    # On ROS 2 Humble, these common keys are widely used, but older/newer builds
    # may rename or ignore some options. If needed, run:
    #   ros2 launch realsense2_camera rs_launch.py --show-args
    # and adapt this file accordingly.

    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="camera",
        description="RealSense camera node name/prefix",
    )

    depth_profile_arg = DeclareLaunchArgument(
        "depth_profile",
        default_value="424,240,15",
        description="Depth stream profile W,H,FPS (low bandwidth default)",
    )

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": LaunchConfiguration("camera_name"),
            "enable_depth": "true",
            "enable_color": "false",
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "pointcloud.enable": "false",
            "align_depth.enable": "false",
            "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
        }.items(),
    )

    return LaunchDescription([
        camera_name_arg,
        depth_profile_arg,
        rs_launch,
    ])
