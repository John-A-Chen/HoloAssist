from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter


def _start_gazebo_processes(context, *args):
    del args
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    verbose = LaunchConfiguration("verbose").perform(context).lower() == "true"
    paused = LaunchConfiguration("paused").perform(context).lower() == "true"

    server_cmd = ["gzserver", world, "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so"]
    if verbose:
        server_cmd.append("--verbose")
    if paused:
        server_cmd.append("-u")

    actions = [ExecuteProcess(cmd=server_cmd, output="screen")]
    if gui:
        actions.append(ExecuteProcess(cmd=["gzclient"], output="screen"))
    return actions


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("holo_assist_depth_tracker")

    world_default = PathJoinSubstitution([pkg_share, "worlds", "holoassist_trolley_board.world"])
    sim_tag_params_default = PathJoinSubstitution([pkg_share, "config", "sim_apriltag_params.yaml"])
    workspace_params_default = PathJoinSubstitution([pkg_share, "config", "workspace.yaml"])
    cube_pose_params_default = PathJoinSubstitution([pkg_share, "config", "cubes.yaml"])
    tracker_params_default = PathJoinSubstitution([pkg_share, "config", "tracker_params.yaml"])
    workspace_perception_params_default = PathJoinSubstitution(
        [pkg_share, "config", "workspace_perception_params.yaml"]
    )

    gui_arg = DeclareLaunchArgument("gui", default_value="true")
    verbose_arg = DeclareLaunchArgument("verbose", default_value="false")
    pause_arg = DeclareLaunchArgument("paused", default_value="false")
    world_arg = DeclareLaunchArgument("world", default_value=world_default)

    start_tracker_arg = DeclareLaunchArgument("start_tracker", default_value="true")
    start_overlay_arg = DeclareLaunchArgument("start_overlay", default_value="true")
    start_workspace_perception_arg = DeclareLaunchArgument(
        "start_workspace_perception", default_value="false"
    )

    sim_tag_params_arg = DeclareLaunchArgument(
        "sim_tag_params_file",
        default_value=sim_tag_params_default,
        description="Synthetic AprilTag publisher params",
    )
    workspace_params_arg = DeclareLaunchArgument(
        "workspace_params_file",
        default_value=workspace_params_default,
    )
    cube_pose_params_arg = DeclareLaunchArgument(
        "cube_pose_params_file",
        default_value=cube_pose_params_default,
    )
    tracker_params_arg = DeclareLaunchArgument(
        "tracker_params_file",
        default_value=tracker_params_default,
    )
    workspace_perception_params_arg = DeclareLaunchArgument(
        "workspace_perception_params_file",
        default_value=workspace_perception_params_default,
    )

    gazebo = OpaqueFunction(function=_start_gazebo_processes)

    synthetic_tags = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_sim_apriltag_publisher_node",
        name="holoassist_sim_apriltag_publisher",
        output="screen",
        parameters=[LaunchConfiguration("sim_tag_params_file")],
    )

    workspace_board = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_workspace_board_node",
        name="holoassist_workspace_board",
        output="screen",
        parameters=[LaunchConfiguration("workspace_params_file")],
    )

    cube_pose = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_cube_pose_node",
        name="holoassist_cube_pose",
        output="screen",
        parameters=[LaunchConfiguration("cube_pose_params_file")],
    )

    overlay = Node(
        package="holo_assist_depth_tracker",
        executable="holoassist_overlay_node",
        name="holoassist_overlay",
        output="screen",
        parameters=[
            {
                "input_image_topic": "/camera/camera/color/image_raw",
                "detections_topic": "/detections_all",
                "output_image_topic": "/holoassist/perception/apriltag_overlay",
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_overlay")),
    )

    tracker = Node(
        package="holo_assist_depth_tracker",
        executable="holo_assist_depth_tracker_node",
        name="holo_assist_depth_tracker",
        output="screen",
        parameters=[LaunchConfiguration("tracker_params_file")],
        condition=IfCondition(LaunchConfiguration("start_tracker")),
    )

    workspace_perception = Node(
        package="holo_assist_depth_tracker",
        executable="workspace_perception_node",
        name="holoassist_workspace_perception",
        output="screen",
        parameters=[LaunchConfiguration("workspace_perception_params_file")],
        condition=IfCondition(LaunchConfiguration("start_workspace_perception")),
    )

    ros_stack = GroupAction(
        actions=[
            SetParameter(name="use_sim_time", value=True),
            synthetic_tags,
            workspace_board,
            cube_pose,
            overlay,
            tracker,
            workspace_perception,
        ]
    )

    return LaunchDescription(
        [
            gui_arg,
            verbose_arg,
            pause_arg,
            world_arg,
            start_tracker_arg,
            start_overlay_arg,
            start_workspace_perception_arg,
            sim_tag_params_arg,
            workspace_params_arg,
            cube_pose_params_arg,
            tracker_params_arg,
            workspace_perception_params_arg,
            gazebo,
            ros_stack,
        ]
    )
