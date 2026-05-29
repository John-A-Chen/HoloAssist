#!/usr/bin/env python3
"""Launch automated real-camera hand-eye calibration through MoveIt."""

import argparse
import os
import pathlib
import signal
import subprocess
import sys
import time

import yaml


ROOT = pathlib.Path(__file__).resolve().parent
DEMO_SETUP = ROOT / "ros2_ws" / "install" / "setup.bash"
MAIN_SETUP = ROOT.parent / "main" / "ros2_ws" / "install" / "setup.bash"


def shell_command(command):
    setup = "source /opt/ros/humble/setup.bash"
    if MAIN_SETUP.exists():
        setup += f" && source {MAIN_SETUP}"
    if DEMO_SETUP.exists():
        setup += f" && source {DEMO_SETUP}"
    return ["bash", "-lc", f"{setup} && {command}"]


def validate_poses_file(path):
    with path.open() as stream:
        data = yaml.safe_load(stream) or {}
    joint_names = data.get("joint_names", [])
    poses = data.get("poses_deg", [])
    expected = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    if joint_names != expected:
        raise ValueError("joint_names must list the six UR joints in controller order")
    if len(poses) < 8 or any(len(pose) != len(expected) for pose in poses):
        raise ValueError("poses_deg must contain at least eight varied six-joint poses")
    return len(poses)


def main():
    parser = argparse.ArgumentParser(
        description="Automated real-camera hand-eye calibration using physical tag36h11:1"
    )
    parser.add_argument("--robot-ip", required=True, help="IP address of the physical UR robot")
    parser.add_argument(
        "--poses-file",
        type=pathlib.Path,
        required=True,
        help="Reviewed joint-pose YAML for the physical robot workspace",
    )
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--no-dashboard", action="store_true")
    parser.add_argument("--fake-gripper", action="store_true")
    parser.add_argument(
        "--velocity-scale",
        type=float,
        default=0.02,
        help="MoveIt trajectory velocity scale, default 0.02 for initial hardware trials",
    )
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Begin the pose/sample sequence when the services become ready",
    )
    parser.add_argument(
        "--acknowledge-real-motion",
        action="store_true",
        help="Required with --auto-start: confirms reviewed poses and a cleared workspace",
    )
    parser.add_argument(
        "--algorithm",
        default="Park",
        choices=["Park", "OpenCV/Tsai-Lenz", "OpenCV/Andreff", "OpenCV/Daniilidis", "OpenCV/Horaud"],
        help="Hand-eye calibration algorithm (default: Park)",
    )
    args = parser.parse_args()

    poses_file = args.poses_file.expanduser().resolve()
    if not poses_file.exists():
        parser.error(f"pose file not found: {poses_file}")
    try:
        pose_count = validate_poses_file(poses_file)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.error(f"invalid pose file {poses_file}: {exc}")
    if not 0.0 < args.velocity_scale <= 0.10:
        parser.error("--velocity-scale must be greater than 0 and no more than 0.10")
    if args.auto_start and not args.acknowledge_real_motion:
        parser.error("--auto-start requires --acknowledge-real-motion")

    processes = []

    def start(name, command):
        print(f"\n>>> Starting {name}\n    {command}", flush=True)
        process = subprocess.Popen(shell_command(command), cwd=ROOT, start_new_session=True)
        processes.append((name, process))
        return process

    def stop_all():
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            for name, process in reversed(processes):
                if process.poll() is None:
                    print(f">>> Stopping {name}", flush=True)
                    os.killpg(os.getpgid(process.pid), signal.SIGINT)
            deadline = time.monotonic() + 8.0
            for _, process in reversed(processes):
                try:
                    process.wait(timeout=max(0.0, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        finally:
            signal.signal(signal.SIGINT, previous)

    print("=" * 72)
    print("  Automated Physical Hand-Eye Calibration")
    print("=" * 72)
    print(f"  Robot:        {args.robot_ip}")
    print("  Observation:  RealSense + physical tag36h11:1 (no generated tag TF)")
    print(f"  Poses:        {pose_count} from {poses_file}")
    print(f"  Speed scale:  {args.velocity_scale:.3f}")
    print(f"  Dashboard:    {'off' if args.no_dashboard else 'on'}")
    print("  Previous TF:  not broadcast during sampling; saved file is retained on SAVE")
    print()
    print("  Required before any motion:")
    print("    - tag36h11:1 is rigidly attached to the tool and visible to the camera")
    print("    - no other physical tag ID 1 is visible in the workspace")
    print("    - every pose has been reviewed for trolley/bin/cable clearance")
    print("    - UR External Control is running and the workcell is clear")
    print("    - launch.py --perception is stopped so no saved camera TF is broadcast")
    print("=" * 72)

    try:
        if not args.no_dashboard:
            start("Dashboard", "python3 dashboard/main.py")
            time.sleep(1.0)
        driver_extra = " use_fake_gripper_hardware:=true" if args.fake_gripper else ""
        start(
            "UR + OnRobot driver",
            "ros2 launch ur_onrobot_control start_robot.launch.py"
            " ur_type:=ur3e onrobot_type:=rg2"
            f" robot_ip:={args.robot_ip} launch_rviz:=false"
            " initial_joint_controller:=scaled_joint_trajectory_controller"
            " activate_joint_controller:=true"
            + driver_extra,
        )
        time.sleep(12.0)
        start("RealSense camera", "ros2 launch holoassist_perception camera.launch.py")
        time.sleep(5.0)
        start(
            "Physical AprilTag detector",
            "ros2 run apriltag_ros apriltag_node --ros-args"
            " -p families:=36h11 -p size:=0.032 -p publish_tf:=true"
            " --remap image_rect:=/camera/camera/color/image_raw"
            " --remap camera_info:=/camera/camera/color/camera_info",
        )
        start(
            "MoveIt calibration motion",
            "ros2 launch holoassist_movement movement.launch.py"
            f" robot_ip:={args.robot_ip} start_pick_place:=false"
            " use_calibrated_workspace:=false"
            f" velocity_scale:={args.velocity_scale}"
            f" cartesian_retime_velocity_scale:={args.velocity_scale}"
            f" use_rviz:={'false' if args.no_rviz else 'true'}"
            + driver_extra,
        )
        time.sleep(10.0)
        start(
            "easy_handeye2 server",
            "ros2 run easy_handeye2 handeye_server --ros-args"
            " -p name:=holoassist_calibration"
            " -p calibration_type:=eye_on_base"
            " -p robot_base_frame:=base_link"
            " -p robot_effector_frame:=tool0"
            " -p tracking_base_frame:=camera_link"
            " -p tracking_marker_frame:=tag36h11:1",
        )
        start(
            "Calibration coordinator",
            "python3 calibration/coordinator_node.py --ros-args"
            f" -p poses_file:={poses_file}"
            f" -p auto_start:={'true' if args.auto_start else 'false'}"
            " -p marker_frame:=tag36h11:1"
            f" -p algorithm:={args.algorithm}",
        )
        start(
            "Waypoint publisher",
            "python3 calibration/waypoint_publisher_node.py --ros-args"
            f" -p poses_file:={poses_file}"
            " -p base_frame:=base_link"
            " -p ee_link:=tool0",
        )
        print("\n>>> Calibration stack is starting.")
        if args.auto_start:
            print(">>> Automatic physical motion will begin once tag and services are ready.")
        else:
            print(">>> Motion is idle. Use the dashboard CALIBRATION tab after verification.")
            print(">>> The equivalent command is:")
            print("    ros2 topic pub --once /holoassist/calibration/command std_msgs/msg/String")
            print("      \"{data: '{\\\"action\\\": \\\"start\\\"}'}\"")
        print(">>> Ctrl+C stops the stack.")
        while True:
            time.sleep(0.5)
            for name, process in processes:
                code = process.poll()
                if code is not None:
                    print(f">>> {name} exited with code {code}", file=sys.stderr)
                    return code or 1
    except KeyboardInterrupt:
        return 0
    finally:
        stop_all()


if __name__ == "__main__":
    sys.exit(main())
