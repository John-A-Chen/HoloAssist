from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Standalone pick-place launch: sequencer + pick_place_service_node only.

    Assumes move_group + coordinate_listener are already running (launched by
    full_holoassist_hardware.launch.py or equivalent).
    """
    moveit_robot_control_pkg = FindPackageShare("moveit_robot_control")

    pick_place_launch = PathJoinSubstitution(
        [moveit_robot_control_pkg, "launch", "pick_place.launch.py"]
    )

    start_pick_place_arg = DeclareLaunchArgument(
        "start_pick_place",
        default_value="true",
    )
    cube_pose_topic_prefix_arg = DeclareLaunchArgument(
        "cube_pose_topic_prefix",
        default_value="/holoassist/perception",
        description="Prefix for april_cube_N_pose topics used by the pick-place service.",
    )
    grasp_z_absolute_arg = DeclareLaunchArgument(
        "grasp_z_absolute",
        default_value="0.05",
        description=(
            "Absolute Z height for the pick descent pose. "
            "Use -1.0 to fall back to cube_z + grasp_z_offset."
        ),
    )
    arm_trajectory_topic_arg = DeclareLaunchArgument(
        "arm_trajectory_topic",
        default_value="/scaled_joint_trajectory_controller/joint_trajectory",
        description="JointTrajectory topic for the pick-place sequencer.",
    )

    pick_place_sequencer = Node(
        package="moveit_robot_control",
        executable="pick_place_sequencer",
        name="pick_place_sequencer",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_pick_place")),
        parameters=[{
            "initial_mode": "run",
            "orientation_mode": "auto",
            "pregrasp_z_offset": 0.10,
            "grasp_z_offset": 0.0,
            "grasp_z_absolute": LaunchConfiguration("grasp_z_absolute"),
            "place_above_z_offset": 0.15,
            "place_z_offset": 0.05,
            "place_descent_enabled": True,
            "arm_trajectory_topic": LaunchConfiguration("arm_trajectory_topic"),
        }],
    )

    pick_place_service = Node(
        package="holo_assist_depth_tracker_sim",
        executable="pick_place_service_node",
        name="holoassist_pick_place_service",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_pick_place")),
        parameters=[{"cube_pose_topic_prefix": LaunchConfiguration("cube_pose_topic_prefix")}],
    )

    return LaunchDescription(
        [
            start_pick_place_arg,
            cube_pose_topic_prefix_arg,
            grasp_z_absolute_arg,
            arm_trajectory_topic_arg,
            pick_place_sequencer,
            pick_place_service,
        ]
    )
