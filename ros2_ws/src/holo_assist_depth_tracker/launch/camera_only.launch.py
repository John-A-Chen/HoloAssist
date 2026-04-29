from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
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
        default_value="848,480,15",
        description="Depth stream profile W,H,FPS (higher resolution default)",
    )
    color_profile_arg = DeclareLaunchArgument(
        "color_profile",
        default_value="640,480,15",
        description="Color stream profile W,H,FPS",
    )
    enable_color_arg = DeclareLaunchArgument(
        "enable_color",
        default_value="true",
        description="Enable RGB color stream (used for RViz debug overlay).",
    )
    align_depth_arg = DeclareLaunchArgument(
        "align_depth",
        default_value="false",
        description="Enable depth alignment filter.",
    )

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
    )

    scoped_rs_launch = GroupAction(
        scoped=True,
        forwarding=False,
        launch_configurations={
            "camera_name": LaunchConfiguration("camera_name"),
            "enable_color": LaunchConfiguration("enable_color"),
            "enable_depth": "true",
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",
            "pointcloud.enable": "false",
            "align_depth.enable": LaunchConfiguration("align_depth"),
            "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
            "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
        },
        actions=[rs_launch],
    )

    return LaunchDescription([
        camera_name_arg,
        depth_profile_arg,
        color_profile_arg,
        enable_color_arg,
        align_depth_arg,
        scoped_rs_launch,
    ])
