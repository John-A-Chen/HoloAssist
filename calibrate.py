#!/usr/bin/env python3
"""
HoloAssist eye-to-hand calibration launcher.

Starts everything needed to recalibrate the RealSense camera position
relative to the robot base in a single command:

  1. UR3e + OnRobot driver (real hardware)
  2. Controller switch (trajectory controller for manual posing)
  3. RealSense camera
  4. AprilTag detector (tag 36h11, 32mm, publish_tf)
  5. easy_handeye2 calibrate GUI (eye_on_base, tracking_base_frame=camera_link)

Usage:
  ./calibrate.sh --robot-ip 192.168.0.192
  ./calibrate.sh --robot-ip 192.168.0.192 --no-rviz
"""

import argparse
import os
import subprocess
import signal
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROS2_WS = os.path.join(SCRIPT_DIR, "ros2_ws")
MAIN_WS = os.path.join(os.path.dirname(SCRIPT_DIR), "main", "ros2_ws")
SOURCE_CMD = (
    f"source /opt/ros/humble/setup.bash"
    f" && source {ROS2_WS}/install/setup.bash"
    f" && [ -f {MAIN_WS}/install/setup.bash ] && source {MAIN_WS}/install/setup.bash || true"
)

processes = []


def run(name, cmd):
    print(f"\n>>> Starting {name}")
    print(f"    {cmd}\n")
    proc = subprocess.Popen(
        ["bash", "-c", f"{SOURCE_CMD} && {cmd}"],
        preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
    )
    processes.append((name, proc))
    return proc


def run_once(name, cmd, retries=1, delay=5):
    for attempt in range(1 + retries):
        result = subprocess.run(
            ["bash", "-c", f"{SOURCE_CMD} && {cmd}"],
            capture_output=True, text=True,
        )
        if result.returncode == 0:
            print(f">>> {name}: OK")
            return True
        if attempt < retries:
            print(f">>> {name}: failed, retrying in {delay}s...")
            time.sleep(delay)
    print(f">>> {name}: FAILED — {result.stderr.strip()}")
    return False


def cleanup(*_):
    print("\n\n>>> Shutting down all processes...")
    for name, proc in reversed(processes):
        if proc.poll() is None:
            print(f"    Stopping {name} (pid {proc.pid})")
            proc.terminate()
    for name, proc in processes:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print(f"    Force killing {name}")
            proc.kill()
    print(">>> All stopped.")
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description="HoloAssist Calibration Launcher")
    parser.add_argument(
        "--robot-ip", type=str, required=True,
        help="Real robot IP (e.g. 192.168.0.192)",
    )
    parser.add_argument("--no-rviz", action="store_true", help="Disable RViz")
    args = parser.parse_args()

    print("=" * 65)
    print("  HoloAssist Eye-to-Hand Calibration")
    print("=" * 65)
    print(f"  Robot IP:    {args.robot_ip}")
    print(f"  RViz:        {'off' if args.no_rviz else 'on'}")
    print()
    print("  Calibration params:")
    print("    calibration_type:    eye_on_base")
    print("    robot_base_frame:    base_link")
    print("    robot_effector_frame: tool0")
    print("    tracking_base_frame: camera_link        <-- IMPORTANT")
    print("    tracking_marker_frame: tag36h11:0")
    print("=" * 65)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ── Phase 1: UR driver ──────────────────────────────────────────
    rviz_flag = "false" if args.no_rviz else "true"
    run(
        "UR + OnRobot Driver",
        f"ros2 launch ur_robot_driver ur_control.launch.py"
        f" ur_type:=ur3e"
        f" robot_ip:={args.robot_ip} launch_rviz:={rviz_flag}",
    )

    print("\n>>> Waiting 12s for UR driver to initialize...")
    time.sleep(12)

    # ── Phase 2: RealSense camera ───────────────────────────────────
    run(
        "RealSense Camera",
        "ros2 launch holo_assist_depth_tracker camera_only.launch.py",
    )

    print("\n>>> Waiting 5s for camera to start...")
    time.sleep(5)

    # ── Phase 3: AprilTag detector ──────────────────────────────────
    run(
        "AprilTag Detector",
        'ros2 run apriltag_ros apriltag_node --ros-args'
        ' -p "families:=36h11" -p "size:=0.032" -p "publish_tf:=true"'
        ' --remap image_rect:=/camera/camera/color/image_raw'
        ' --remap camera_info:=/camera/camera/color/camera_info',
    )

    print("\n>>> Waiting 3s for AprilTag detector...")
    time.sleep(3)

    # ── Phase 4: easy_handeye2 calibration GUI ──────────────────────
    run(
        "easy_handeye2 Calibrate",
        "ros2 launch easy_handeye2 calibrate.launch.py"
        " name:=holoassist_calibration"
        " calibration_type:=eye_on_base"
        " robot_base_frame:=base_link"
        " robot_effector_frame:=tool0"
        " tracking_base_frame:=camera_link"
        " tracking_marker_frame:=tag36h11:0",
    )

    # ── Done ────────────────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("  All running. Ctrl+C to stop everything.")
    print()
    print("  Steps:")
    print("    1. On teach pendant: load + run External Control")
    print("       (Host IP: 192.168.0.100)")
    print("    2. Move robot to different poses using teach pendant")
    print("    3. At each pose, click 'Take Sample' in the GUI")
    print("    4. Collect 15-25 samples with varied orientations")
    print("    5. Click 'Save' — calibration saved to:")
    print("       ~/.ros2/easy_handeye2/calibrations/")
    print("       holoassist_calibration.calib")
    print()
    print("  Verify in RViz: camera model should appear at the")
    print("  correct physical location relative to the robot.")
    print("=" * 65)

    critical = {"UR + OnRobot Driver"}
    while True:
        for name, proc in list(processes):
            ret = proc.poll()
            if ret is not None:
                print(f"\n>>> {name} exited (code {ret})")
                if name in critical:
                    print(f"    Critical process died — shutting down.")
                    cleanup()
                else:
                    processes.remove((name, proc))
        time.sleep(1)


if __name__ == "__main__":
    main()
