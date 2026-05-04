from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_webcam_arg = DeclareLaunchArgument(
        "use_webcam",
        default_value="true",
        description="Start webcam image + camera_info publisher.",
    )
    webcam_device_index_arg = DeclareLaunchArgument(
        "webcam_device_index",
        default_value="0",
        description="OpenCV webcam device index.",
    )
    webcam_fps_arg = DeclareLaunchArgument(
        "webcam_fps",
        default_value="15.0",
        description="Webcam frame rate.",
    )
    webcam_width_arg = DeclareLaunchArgument(
        "webcam_width",
        default_value="640",
        description="Webcam width.",
    )
    webcam_height_arg = DeclareLaunchArgument(
        "webcam_height",
        default_value="480",
        description="Webcam height.",
    )
    webcam_use_sensor_data_qos_arg = DeclareLaunchArgument(
        "webcam_use_sensor_data_qos",
        default_value="true",
        description="Publish webcam topics using sensor-data QoS (best-effort).",
    )
    webcam_qos_depth_arg = DeclareLaunchArgument(
        "webcam_qos_depth",
        default_value="5",
        description="QoS history depth for webcam image/camera_info topics.",
    )
    webcam_frame_id_arg = DeclareLaunchArgument(
        "webcam_frame_id",
        default_value="webcam_frame",
        description="Frame id for webcam image/camera_info.",
    )
    webcam_hfov_deg_arg = DeclareLaunchArgument(
        "webcam_hfov_deg",
        default_value="69.4",
        description="Approximate webcam HFOV used for synthetic intrinsics.",
    )

    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/camera/color/image_raw",
        description="Input RGB topic for apriltag detector.",
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/camera/color/camera_info",
        description="CameraInfo topic paired with image_topic.",
    )
    detections_topic_arg = DeclareLaunchArgument(
        "detections_topic",
        default_value="/detections",
        description="AprilTag detection topic from apriltag_ros.",
    )

    apriltag_start_camera_arg = DeclareLaunchArgument(
        "apriltag_start_camera",
        default_value="false",
        description="Start RealSense inside apriltag_workbench (set false for webcam).",
    )
    apriltag_params_file_arg = DeclareLaunchArgument(
        "apriltag_params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("holoassist_foxglove"),
                "config",
                "apriltag_cube_36h11_32mm.yaml",
            ]
        ),
        description="apriltag_ros parameter file for cube tags.",
    )
    start_overlay_arg = DeclareLaunchArgument(
        "start_overlay",
        default_value="true",
        description="Enable apriltag RGB overlay image output.",
    )
    overlay_output_topic_arg = DeclareLaunchArgument(
        "overlay_output_topic",
        default_value="/holoassist/apriltag/cube_debug_image",
        description="Overlay image topic.",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Start RViz2 with AprilTag debug image display.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("holoassist_foxglove"),
                "config",
                "apriltag_cube_pose_debug.rviz",
            ]
        ),
        description="RViz2 config path for cube debug visualization.",
    )
    rviz_fixed_frame_arg = DeclareLaunchArgument(
        "rviz_fixed_frame",
        default_value="camera_color_optical_frame",
        description=(
            "RViz fixed frame. For webcam runs use webcam_frame; for RealSense use camera_color_optical_frame."
        ),
    )

    cube_pose_topic_arg = DeclareLaunchArgument(
        "cube_pose_topic",
        default_value="/holoassist/perception/april_cube_pose",
        description="Published AprilCube pose topic.",
    )
    cube_marker_topic_arg = DeclareLaunchArgument(
        "cube_marker_topic",
        default_value="/holoassist/perception/april_cube_marker",
        description="Published AprilCube marker topic.",
    )
    cube_status_topic_arg = DeclareLaunchArgument(
        "cube_status_topic",
        default_value="/holoassist/perception/april_cube_status",
        description="Published AprilCube status text topic.",
    )
    cube_tf_child_frame_arg = DeclareLaunchArgument(
        "cube_tf_child_frame",
        default_value="apriltag_cube",
        description="TF child frame for estimated cube pose.",
    )
    cube_size_m_arg = DeclareLaunchArgument(
        "cube_size_m",
        default_value="0.075",
        description="Physical cube size in meters (edge length).",
    )
    cube_detections_timeout_s_arg = DeclareLaunchArgument(
        "cube_detections_timeout_s",
        default_value="2.0",
        description="How long to keep last detections before cube pose is marked stale.",
    )
    cube_update_rate_hz_arg = DeclareLaunchArgument(
        "cube_update_rate_hz",
        default_value="10.0",
        description="Cube pose solver timer frequency.",
    )
    cube_camera_frame_arg = DeclareLaunchArgument(
        "cube_camera_frame",
        default_value="",
        description="Reference frame for cube pose (empty=use detections header frame).",
    )
    cube_obstacle_marker_topic_arg = DeclareLaunchArgument(
        "cube_obstacle_marker_topic",
        default_value="/holo_assist_depth_tracker/obstacle_marker",
        description="Obstacle marker topic used for center fusion/fallback pose.",
    )
    cube_use_obstacle_center_fusion_arg = DeclareLaunchArgument(
        "cube_use_obstacle_center_fusion",
        default_value="false",
        description="Fuse obstacle marker center with tag-based cube center.",
    )
    cube_publish_obstacle_only_pose_arg = DeclareLaunchArgument(
        "cube_publish_obstacle_only_pose",
        default_value="false",
        description="Publish approximate cube pose from obstacle marker when tags are stale/missing.",
    )
    cube_obstacle_timeout_s_arg = DeclareLaunchArgument(
        "cube_obstacle_timeout_s",
        default_value="0.75",
        description="Obstacle marker freshness timeout for fusion/fallback.",
    )
    cube_obstacle_weight_min_arg = DeclareLaunchArgument(
        "cube_obstacle_weight_min",
        default_value="0.20",
        description="Minimum obstacle-center fusion weight when many tags are visible.",
    )
    cube_obstacle_weight_max_arg = DeclareLaunchArgument(
        "cube_obstacle_weight_max",
        default_value="0.65",
        description="Maximum obstacle-center fusion weight when few tags are visible.",
    )
    cube_obstacle_max_center_delta_m_arg = DeclareLaunchArgument(
        "cube_obstacle_max_center_delta_m",
        default_value="0.15",
        description="Reject obstacle-center fusion when tag/obstacle centers differ by more than this distance.",
    )

    webcam_node = Node(
        package="holo_assist_depth_tracker",
        executable="holo_assist_webcam_image_publisher",
        name="holo_assist_webcam_image_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_webcam")),
        parameters=[
            {
                "device_index": LaunchConfiguration("webcam_device_index"),
                "image_topic": LaunchConfiguration("image_topic"),
                "frame_id": LaunchConfiguration("webcam_frame_id"),
                "fps": LaunchConfiguration("webcam_fps"),
                "width": LaunchConfiguration("webcam_width"),
                "height": LaunchConfiguration("webcam_height"),
                "use_sensor_data_qos": LaunchConfiguration(
                    "webcam_use_sensor_data_qos"
                ),
                "qos_depth": LaunchConfiguration("webcam_qos_depth"),
                "publish_camera_info": True,
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "camera_info_frame_id": LaunchConfiguration("webcam_frame_id"),
                "hfov_deg": LaunchConfiguration("webcam_hfov_deg"),
            }
        ],
    )

    apriltag_workbench = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("holoassist_foxglove"), "launch", "apriltag_workbench.launch.py"]
            )
        ),
        launch_arguments={
            "start_camera": LaunchConfiguration("apriltag_start_camera"),
            "image_topic": LaunchConfiguration("image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "params_file": LaunchConfiguration("apriltag_params_file"),
            "start_overlay": LaunchConfiguration("start_overlay"),
            "overlay_output_topic": LaunchConfiguration("overlay_output_topic"),
            "detections_topic": LaunchConfiguration("detections_topic"),
            "workspace_outline_enabled": "false",
        }.items(),
    )

    cube_pose_node = Node(
        package="holoassist_foxglove",
        executable="apriltag_cube_pose_node",
        name="apriltag_cube_pose",
        output="screen",
        parameters=[
            {
                "detections_topic": LaunchConfiguration("detections_topic"),
                "camera_frame": LaunchConfiguration("cube_camera_frame"),
                "cube_pose_topic": LaunchConfiguration("cube_pose_topic"),
                "cube_marker_topic": LaunchConfiguration("cube_marker_topic"),
                "cube_status_topic": LaunchConfiguration("cube_status_topic"),
                "cube_tf_child_frame": LaunchConfiguration("cube_tf_child_frame"),
                "cube_size_m": LaunchConfiguration("cube_size_m"),
                "detections_timeout_s": LaunchConfiguration("cube_detections_timeout_s"),
                "update_rate_hz": LaunchConfiguration("cube_update_rate_hz"),
                "obstacle_marker_topic": LaunchConfiguration("cube_obstacle_marker_topic"),
                "use_obstacle_marker_center_fusion": LaunchConfiguration(
                    "cube_use_obstacle_center_fusion"
                ),
                "publish_obstacle_only_pose": LaunchConfiguration(
                    "cube_publish_obstacle_only_pose"
                ),
                "obstacle_timeout_s": LaunchConfiguration("cube_obstacle_timeout_s"),
                "obstacle_center_weight_min": LaunchConfiguration(
                    "cube_obstacle_weight_min"
                ),
                "obstacle_center_weight_max": LaunchConfiguration(
                    "cube_obstacle_weight_max"
                ),
                "obstacle_max_center_delta_m": LaunchConfiguration(
                    "cube_obstacle_max_center_delta_m"
                ),
            }
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="holoassist_apriltag_cube_rviz",
        output="screen",
        arguments=[
            "-d",
            LaunchConfiguration("rviz_config"),
            "-f",
            LaunchConfiguration("rviz_fixed_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return LaunchDescription(
        [
            use_webcam_arg,
            webcam_device_index_arg,
            webcam_fps_arg,
            webcam_width_arg,
            webcam_height_arg,
            webcam_use_sensor_data_qos_arg,
            webcam_qos_depth_arg,
            webcam_frame_id_arg,
            webcam_hfov_deg_arg,
            image_topic_arg,
            camera_info_topic_arg,
            detections_topic_arg,
            apriltag_start_camera_arg,
            apriltag_params_file_arg,
            start_overlay_arg,
            overlay_output_topic_arg,
            start_rviz_arg,
            rviz_config_arg,
            rviz_fixed_frame_arg,
            cube_pose_topic_arg,
            cube_marker_topic_arg,
            cube_status_topic_arg,
            cube_tf_child_frame_arg,
            cube_size_m_arg,
            cube_detections_timeout_s_arg,
            cube_update_rate_hz_arg,
            cube_camera_frame_arg,
            cube_obstacle_marker_topic_arg,
            cube_use_obstacle_center_fusion_arg,
            cube_publish_obstacle_only_pose_arg,
            cube_obstacle_timeout_s_arg,
            cube_obstacle_weight_min_arg,
            cube_obstacle_weight_max_arg,
            cube_obstacle_max_center_delta_m_arg,
            webcam_node,
            apriltag_workbench,
            cube_pose_node,
            rviz_node,
        ]
    )
