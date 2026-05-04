#!/usr/bin/env python3
"""HoloAssist ROS 2 launcher — starts UR driver, controller switch, and TCP endpoint."""

import argparse
import os
import subprocess
import signal
import sys
import time
import socket

# Resolve ros2_ws relative to this script so the launcher works on any machine.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROS2_WS = os.path.join(SCRIPT_DIR, "ros2_ws")
SOURCE_CMD = f"source /opt/ros/humble/setup.bash && source {ROS2_WS}/install/setup.bash"

DEFAULT_WIFI_IP = "172.19.115.104"
DEFAULT_ROBOT_IP = "192.168.0.194"

processes = []


def get_wifi_ip():
    """Try to get current WiFi IP, fall back to default."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return DEFAULT_WIFI_IP


def run(name, cmd):
    """Launch a bash subprocess and track it."""
    print(f"\n>>> Starting {name}")
    print(f"    {cmd}\n")
    proc = subprocess.Popen(
        ["bash", "-c", f"{SOURCE_CMD} && {cmd}"],
        preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
    )
    processes.append((name, proc))
    return proc


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
    parser = argparse.ArgumentParser(description="HoloAssist ROS 2 Launcher")
    parser.add_argument(
        "--robot-ip",
        type=str,
        default=None,
        help=f"Real robot IP (e.g. {DEFAULT_ROBOT_IP}). If omitted, uses fake hardware.",
    )
    parser.add_argument(
        "--ros-ip",
        type=str,
        default="0.0.0.0",
        help="ROS TCP endpoint bind IP (default: 0.0.0.0 = all interfaces)",
    )
    parser.add_argument(
        "--no-rviz",
        action="store_true",
        help="Disable RViz",
    )
    args = parser.parse_args()

    fake = args.robot_ip is None
    robot_ip = "0.0.0.0" if fake else args.robot_ip
    wifi_ip = get_wifi_ip()

    print("=" * 60)
    print("  HoloAssist ROS 2 Launcher")
    print("=" * 60)
    print(f"  Mode:       {'FAKE HARDWARE' if fake else 'REAL ROBOT'}")
    if not fake:
        print(f"  Robot IP:   {robot_ip}")
    print(f"  ROS IP:     {args.ros_ip}")
    print(f"  WiFi IP:    {wifi_ip}  <-- set this in Unity ROS Settings")
    print(f"  RViz:       {'off' if args.no_rviz else 'on'}")
    print("=" * 60)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # 1. UR + OnRobot driver (combined)
    rviz = "false" if args.no_rviz else "true"
    driver_cmd = (
        f"ros2 launch ur_onrobot_control start_robot.launch.py"
        f" ur_type:=ur3e onrobot_type:=rg2 launch_rviz:={rviz}"
    )
    if fake:
        driver_cmd += " use_fake_hardware:=true"
    else:
        driver_cmd += f" robot_ip:={robot_ip}"
    run("UR + OnRobot Driver", driver_cmd)

    # Wait for driver to initialize before switching controllers
    print("\n>>> Waiting 8s for UR driver to start...")
    time.sleep(8)

    # 2. Switch controllers (activate velocity + gripper, deactivate trajectory)
    # Use `ros2 service call` instead of `ros2 control switch_controllers` so we
    # don't depend on the ros2controlcli apt package being installed.
    switch_cmd = (
        "ros2 service call /controller_manager/switch_controller"
        " controller_manager_msgs/srv/SwitchController"
        " \"{activate_controllers: ['forward_velocity_controller', 'finger_width_controller'],"
        " deactivate_controllers: ['scaled_joint_trajectory_controller', 'finger_width_trajectory_controller'],"
        " strictness: 2}\""
    )
    switch_proc = subprocess.run(
        ["bash", "-c", f"{SOURCE_CMD} && {switch_cmd}"],
        capture_output=True,
        text=True,
    )
    if switch_proc.returncode == 0:
        print(">>> Controllers switched (forward_velocity_controller + finger_width_controller active)")
    else:
        print(f">>> Controller switch failed: {switch_proc.stderr.strip()}")
        print("    Retrying in 5s...")
        time.sleep(5)
        subprocess.run(["bash", "-c", f"{SOURCE_CMD} && {switch_cmd}"])

    # 3. ROS-TCP endpoint
    tcp_cmd = (
        f"ros2 run ros_tcp_endpoint default_server_endpoint"
        f" --ros-args -p ROS_IP:={args.ros_ip}"
    )
    run("ROS-TCP Endpoint", tcp_cmd)

    print("\n" + "=" * 60)
    print("  All running. Ctrl+C to stop everything.")
    if not fake:
        print("  Don't forget: run External Control on teach pendant")
    print("  Then hit Play in Unity.")
    print("=" * 60)

    # Wait for any process to exit
    while True:
        for name, proc in processes:
            ret = proc.poll()
            if ret is not None:
                print(f"\n>>> {name} exited (code {ret})")
                cleanup()
        time.sleep(1)


if __name__ == "__main__":
    main()
