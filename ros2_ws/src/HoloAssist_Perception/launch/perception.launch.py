"""AprilTag cube perception pipeline.

Starts:
  1. RealSense camera          (skip with start_camera:=false for webcam path)
  2. apriltag_ros detector     (publishes /detections)
  3. aprilcube_tracker_node    (RGB debug overlay reading cube poses + tag detections)
  4. RViz                      (optional)

Webcam path (launch.py starts the webcam publisher before calling this):
  ros2 launch holoassist_perception perception.launch.py start_camera:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("holoassist_perception")

    start_camera_arg = DeclareLaunchArgument(
        "start_camera", default_value="true",
        description="Launch the RealSense camera. Set false when camera is already running.",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz", default_value="true",
        description="Open RViz2 with the perception debug display.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([pkg_share, "rviz", "holoassist_full.rviz"]),
    )
    apriltag_config_arg = DeclareLaunchArgument(
        "apriltag_config",
        default_value=PathJoinSubstitution([pkg_share, "config", "apriltag_cubes.yaml"]),
        description="AprilTag family/size config passed to apriltag_ros.",
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "camera.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("start_camera")),
    )

    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_ros",
        remappings=[
            ("image_rect", "/camera/camera/color/image_raw"),
            ("camera_info", "/camera/camera/color/camera_info"),
        ],
        parameters=[LaunchConfiguration("apriltag_config")],
        output="screen",
    )

    tracker = Node(
        package="holoassist_perception",
        executable="aprilcube_tracker_node",
        name="holoassist_perception",
        output="screen",
        emulate_tty=True,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return LaunchDescription([
        start_camera_arg,
        start_rviz_arg,
        rviz_config_arg,
        apriltag_config_arg,
        camera,
        apriltag,
        tracker,
        rviz,
    ])
