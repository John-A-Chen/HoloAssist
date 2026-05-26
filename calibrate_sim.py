#!/usr/bin/env python3
"""Launch MoveIt fake hardware plus automated easy_handeye2 calibration."""

import argparse
import os
import pathlib
import signal
import subprocess
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parent
MAIN_SETUP = ROOT.parent / "main" / "ros2_ws" / "install" / "setup.bash"
DEMO_SETUP = ROOT / "ros2_ws" / "install" / "setup.bash"


def shell_command(command):
    setup = f"source /opt/ros/humble/setup.bash && source {MAIN_SETUP}"
    if DEMO_SETUP.exists():
        setup += f" && source {DEMO_SETUP}"
    return ["bash", "-lc", f"{setup} && {command}"]


def main():
    parser = argparse.ArgumentParser(description="Automated hand-eye calibration simulation")
    parser.add_argument("--no-dashboard", action="store_true")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--auto-start", action="store_true")
    args = parser.parse_args()

    if not MAIN_SETUP.exists():
        print(f"Missing ROS overlay: {MAIN_SETUP}", file=sys.stderr)
        return 1
    if not DEMO_SETUP.exists():
        print(
            "The demo ROS overlay has not been built. Run:\n"
            "  cd ros2_ws && source /opt/ros/humble/setup.bash && "
            f"source {MAIN_SETUP} && colcon build --symlink-install "
            "--packages-select moveit_robot_control",
            file=sys.stderr,
        )
        return 1

    pose_file = ROOT / "calibration" / "sim_poses.yaml"
    processes = []

    def start(name, command):
        print(f"[calibrate_sim] starting {name}: {command}", flush=True)
        proc = subprocess.Popen(
            shell_command(command),
            cwd=ROOT,
            preexec_fn=os.setsid,
        )
        processes.append((name, proc))

    def stop_all():
        for _, proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        deadline = time.monotonic() + 5.0
        for _, proc in reversed(processes):
            remaining = max(0.0, deadline - time.monotonic())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)

    try:
        rviz = "false" if args.no_rviz else "true"
        start(
            "MoveIt simulation",
            "ros2 launch moveit_robot_control full_holoassist_moveit_sim.launch.py "
            f"use_rviz:={rviz} start_perception:=false start_pick_place:=false",
        )
        time.sleep(5.0)
        start("synthetic tag 1", "python3 calibration/sim_marker_node.py")
        start(
            "easy_handeye2 server",
            "ros2 run easy_handeye2 handeye_server --ros-args "
            "-p name:=holoassist_calibration "
            "-p calibration_type:=eye_on_base "
            "-p robot_base_frame:=base_link "
            "-p robot_effector_frame:=tool0 "
            "-p tracking_base_frame:=camera_link "
            "-p tracking_marker_frame:=tag36h11:1",
        )
        auto_start = "true" if args.auto_start else "false"
        start(
            "calibration coordinator",
            "python3 calibration/coordinator_node.py --ros-args "
            f"-p poses_file:={pose_file} -p auto_start:={auto_start}",
        )
        if not args.no_dashboard:
            start("dashboard", "python3 dashboard/main.py")
        while True:
            time.sleep(0.5)
            for name, proc in processes:
                return_code = proc.poll()
                if return_code is not None:
                    print(f"[calibrate_sim] {name} exited with {return_code}", file=sys.stderr)
                    return return_code or 1
    except KeyboardInterrupt:
        return 0
    finally:
        stop_all()


if __name__ == "__main__":
    sys.exit(main())
