from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "move_group_name",
                default_value="ur_onrobot_manipulator",
                description="MoveIt planning group used for coordinate goals.",
            ),
            DeclareLaunchArgument(
                "frame",
                default_value="base_link",
                description="Planning frame for coordinate goals.",
            ),
            DeclareLaunchArgument(
                "ee_link",
                default_value="tool0",
                description="End-effector link to move to coordinate goals.",
            ),
            DeclareLaunchArgument(
                "trajectory_topic",
                default_value="/scaled_joint_trajectory_controller/joint_trajectory",
                description=(
                    "JointTrajectory topic to publish planned trajectories on. "
                    "Use /joint_trajectory_controller/joint_trajectory for fake hardware."
                ),
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/moveit_robot_control/target_pose",
                description="Topic for geometry_msgs/msg/Pose goals.",
            ),
            DeclareLaunchArgument(
                "coordinate_topic",
                default_value="/moveit_robot_control/target",
                description="Topic for moveit_robot_control_msgs/msg/TargetRPY goals.",
            ),
            DeclareLaunchArgument(
                "point_topic",
                default_value="/moveit_robot_control/target_point",
                description="Topic for legacy geometry_msgs/msg/Point goals.",
            ),
            DeclareLaunchArgument(
                "require_robot_status",
                default_value="true",
                description=(
                    "Require UR driver status topics before executing. Set false "
                    "for fake hardware simulation."
                ),
            ),
            DeclareLaunchArgument(
                "require_controller_check",
                default_value="true",
                description=(
                    "Check and activate scaled_joint_trajectory_controller before "
                    "executing. Set false for fake hardware simulation."
                ),
            ),
            DeclareLaunchArgument(
                "allow_pose_goal_fallback",
                default_value="true",
                description=(
                    "Allow non-straight MoveIt pose planning if the straight "
                    "Cartesian path fails."
                ),
            ),
            DeclareLaunchArgument(
                "orientation_mode",
                default_value="auto",
                description=(
                    "Orientation policy for point goals: auto, current, or fixed."
                ),
            ),
            DeclareLaunchArgument(
                "use_current_orientation",
                default_value="false",
                description=(
                    "Legacy setting for point goals when orientation_mode is not set. "
                    "Kept for backwards compatibility."
                ),
            ),
            DeclareLaunchArgument(
                "roll_deg",
                default_value="180.0",
                description="Target end-effector roll in degrees.",
            ),
            DeclareLaunchArgument(
                "pitch_deg",
                default_value="0.0",
                description="Target end-effector pitch in degrees.",
            ),
            DeclareLaunchArgument(
                "yaw_deg",
                default_value="0.0",
                description="Target end-effector yaw in degrees.",
            ),
            DeclareLaunchArgument(
                "auto_orientation_roll_deg",
                default_value="180.0",
                description="Base roll used for auto orientation candidates.",
            ),
            DeclareLaunchArgument(
                "auto_orientation_max_pitch_deg",
                default_value="55.0",
                description="Maximum inward tilt angle for auto orientation.",
            ),
            DeclareLaunchArgument(
                "auto_orientation_pitch_step_deg",
                default_value="15.0",
                description="Pitch step between auto orientation candidates.",
            ),
            DeclareLaunchArgument(
                "auto_orientation_radius_start_m",
                default_value="0.20",
                description="Radius where auto tilt starts increasing.",
            ),
            DeclareLaunchArgument(
                "auto_orientation_radius_full_m",
                default_value="0.55",
                description="Radius where auto tilt reaches full effect.",
            ),
            DeclareLaunchArgument(
                "auto_orientation_height_start_m",
                default_value="0.10",
                description="Height where extra auto tilt starts increasing.",
            ),
            DeclareLaunchArgument(
                "auto_orientation_height_full_m",
                default_value="0.45",
                description="Height where extra auto tilt reaches full effect.",
            ),
            DeclareLaunchArgument(
                "auto_orientation_height_bonus_deg",
                default_value="10.0",
                description="Extra tilt added for higher targets.",
            ),
            DeclareLaunchArgument(
                "pose_goal_orientation_tolerance",
                default_value="0.05",
                description=(
                    "Orientation tolerance in radians when using pose-goal "
                    "fallback planning."
                ),
            ),
            DeclareLaunchArgument(
                "pose_goal_planning_time",
                default_value="5.0",
                description="Planning timeout in seconds for pose-goal fallback.",
            ),
            DeclareLaunchArgument(
                "pose_goal_route_candidates",
                default_value="5",
                description="Number of candidate fallback routes to try.",
            ),
            DeclareLaunchArgument(
                "avoid_flange_forearm_clamp",
                default_value="true",
                description=(
                    "Reject trajectories that enter the predicted UR "
                    "flange-to-forearm protective-stop zone."
                ),
            ),
            DeclareLaunchArgument(
                "forearm_clamp_surface_clearance_m",
                default_value="0.028",
                description=(
                    "Minimum predicted surface clearance in meters between the "
                    "forearm clamp cylinder and the tool flange safety sphere."
                ),
            ),
            DeclareLaunchArgument(
                "collision_check_stride",
                default_value="1",
                description="Check every Nth trajectory point for collision.",
            ),
            DeclareLaunchArgument(
                "velocity_scale",
                default_value="0.05",
                description="Trajectory velocity scale from 0.0 to 1.0.",
            ),
            DeclareLaunchArgument(
                "joint_goal_tolerance",
                default_value="0.1",
                description=(
                    "Allowed final angular joint error in radians after "
                    "trajectory execution."
                ),
            ),
            DeclareLaunchArgument(
                "execution_timeout_scale",
                default_value="20.0",
                description=(
                    "Multiplier for nominal trajectory duration while waiting "
                    "for speed-scaled real robot execution to settle."
                ),
            ),
            DeclareLaunchArgument(
                "execution_timeout_padding",
                default_value="5.0",
                description=(
                    "Extra seconds added to the scaled execution settling timeout."
                ),
            ),
            Node(
                package="moveit_robot_control",
                executable="coordinate_listener",
                name="moveit_robot_control",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "move_group_name": LaunchConfiguration("move_group_name"),
                        "trajectory_topic": LaunchConfiguration("trajectory_topic"),
                        "frame": LaunchConfiguration("frame"),
                        "ee_link": LaunchConfiguration("ee_link"),
                        "coordinate_topic": LaunchConfiguration("coordinate_topic"),
                        "point_topic": LaunchConfiguration("point_topic"),
                        "pose_topic": LaunchConfiguration("pose_topic"),
                        "require_robot_status": LaunchConfiguration(
                            "require_robot_status"
                        ),
                        "require_controller_check": ParameterValue(
                            LaunchConfiguration("require_controller_check"),
                            value_type=bool,
                        ),
                        "allow_pose_goal_fallback": LaunchConfiguration(
                            "allow_pose_goal_fallback"
                        ),
                        "orientation_mode": LaunchConfiguration("orientation_mode"),
                        "use_current_orientation": LaunchConfiguration(
                            "use_current_orientation"
                        ),
                        "roll_deg": ParameterValue(
                            LaunchConfiguration("roll_deg"), value_type=float
                        ),
                        "pitch_deg": ParameterValue(
                            LaunchConfiguration("pitch_deg"), value_type=float
                        ),
                        "yaw_deg": ParameterValue(
                            LaunchConfiguration("yaw_deg"), value_type=float
                        ),
                        "auto_orientation_roll_deg": ParameterValue(
                            LaunchConfiguration("auto_orientation_roll_deg"),
                            value_type=float,
                        ),
                        "auto_orientation_max_pitch_deg": ParameterValue(
                            LaunchConfiguration("auto_orientation_max_pitch_deg"),
                            value_type=float,
                        ),
                        "auto_orientation_pitch_step_deg": ParameterValue(
                            LaunchConfiguration("auto_orientation_pitch_step_deg"),
                            value_type=float,
                        ),
                        "auto_orientation_radius_start_m": ParameterValue(
                            LaunchConfiguration("auto_orientation_radius_start_m"),
                            value_type=float,
                        ),
                        "auto_orientation_radius_full_m": ParameterValue(
                            LaunchConfiguration("auto_orientation_radius_full_m"),
                            value_type=float,
                        ),
                        "auto_orientation_height_start_m": ParameterValue(
                            LaunchConfiguration("auto_orientation_height_start_m"),
                            value_type=float,
                        ),
                        "auto_orientation_height_full_m": ParameterValue(
                            LaunchConfiguration("auto_orientation_height_full_m"),
                            value_type=float,
                        ),
                        "auto_orientation_height_bonus_deg": ParameterValue(
                            LaunchConfiguration("auto_orientation_height_bonus_deg"),
                            value_type=float,
                        ),
                        "pose_goal_orientation_tolerance": ParameterValue(
                            LaunchConfiguration("pose_goal_orientation_tolerance"),
                            value_type=float,
                        ),
                        "pose_goal_planning_time": ParameterValue(
                            LaunchConfiguration("pose_goal_planning_time"),
                            value_type=float,
                        ),
                        "pose_goal_route_candidates": ParameterValue(
                            LaunchConfiguration("pose_goal_route_candidates"),
                            value_type=int,
                        ),
                        "avoid_flange_forearm_clamp": ParameterValue(
                            LaunchConfiguration("avoid_flange_forearm_clamp"),
                            value_type=bool,
                        ),
                        "forearm_clamp_surface_clearance_m": ParameterValue(
                            LaunchConfiguration("forearm_clamp_surface_clearance_m"),
                            value_type=float,
                        ),
                        "collision_check_stride": ParameterValue(
                            LaunchConfiguration("collision_check_stride"),
                            value_type=int,
                        ),
                        "velocity_scale": ParameterValue(
                            LaunchConfiguration("velocity_scale"),
                            value_type=float,
                        ),
                        "joint_goal_tolerance": ParameterValue(
                            LaunchConfiguration("joint_goal_tolerance"),
                            value_type=float,
                        ),
                        "execution_timeout_scale": ParameterValue(
                            LaunchConfiguration("execution_timeout_scale"),
                            value_type=float,
                        ),
                        "execution_timeout_padding": ParameterValue(
                            LaunchConfiguration("execution_timeout_padding"),
                            value_type=float,
                        ),
                    }
                ],
            )
        ]
    )
