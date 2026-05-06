#!/usr/bin/env python3
"""
HoloAssist ROS 2 launcher.

Starts:
  - UR3e + OnRobot RG2 driver (fake or real hardware)
  - Controller switch (velocity + gripper for teleop)
  - ROS-TCP endpoint (Unity bridge, port 10000)
  - UDP beacon (Quest auto-discovery)
  - [optional] Perception sim + cube pose relay (--perception)
"""

import argparse
import os
import subprocess
import signal
import sys
import time
import socket

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROS2_WS = os.path.join(SCRIPT_DIR, "ros2_ws")
SOURCE_CMD = f"source /opt/ros/humble/setup.bash && source {ROS2_WS}/install/setup.bash"

DEFAULT_WIFI_IP = "172.19.115.104"
DEFAULT_ROBOT_IP = "192.168.0.194"
PREFERRED_SUBNET = "192.168.0."

processes = []


def get_wifi_ip():
    """Return the best local IP — prefer the 192.168.0.x subnet."""
    try:
        import netifaces
        for iface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(iface).get(netifaces.AF_INET, [])
            for a in addrs:
                if a["addr"].startswith(PREFERRED_SUBNET):
                    return a["addr"]
    except ImportError:
        pass
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


def run_once(name, cmd, retries=1, delay=5):
    """Run a command once (blocking), retry on failure."""
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
    parser = argparse.ArgumentParser(description="HoloAssist ROS 2 Launcher")
    parser.add_argument(
        "--robot-ip", type=str, default=None,
        help=f"Real robot IP (e.g. {DEFAULT_ROBOT_IP}). Omit for fake hardware.",
    )
    parser.add_argument(
        "--ros-ip", type=str, default="0.0.0.0",
        help="ROS TCP endpoint bind IP (default: 0.0.0.0)",
    )
    parser.add_argument("--no-rviz", action="store_true", help="Disable RViz")
    parser.add_argument(
        "--perception", action="store_true",
        help="Start perception pipeline + cube pose relay (AprilTag cubes in Unity)",
    )
    args = parser.parse_args()

    fake = args.robot_ip is None
    robot_ip = "0.0.0.0" if fake else args.robot_ip
    wifi_ip = get_wifi_ip()

    print("=" * 65)
    print("  HoloAssist ROS 2 Launcher")
    print("=" * 65)
    print(f"  Mode:        {'FAKE HARDWARE' if fake else 'REAL ROBOT'}")
    if not fake:
        print(f"  Robot IP:    {robot_ip}")
    print(f"  ROS IP:      {args.ros_ip}")
    print(f"  WiFi IP:     {wifi_ip}  <-- set this in Unity ROS Settings")
    print(f"  RViz:        {'off' if args.no_rviz else 'on'}")
    print(f"  Perception:  {'on' if args.perception else 'off (use --perception to enable)'}")
    print("=" * 65)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ── Phase 1: UR + OnRobot driver ─────────────────────────────────
    rviz_flag = "false" if args.no_rviz else "true"
    driver_cmd = (
        f"ros2 launch ur_onrobot_control start_robot.launch.py"
        f" ur_type:=ur3e onrobot_type:=rg2 launch_rviz:={rviz_flag}"
    )
    if fake:
        driver_cmd += " use_fake_hardware:=true"
    else:
        driver_cmd += f" robot_ip:={robot_ip}"
    run("UR + OnRobot Driver", driver_cmd)

    print("\n>>> Waiting 10s for UR driver to initialize...")
    time.sleep(10)

    # ── Phase 2: Controller switch ───────────────────────────────────
    switch_cmd = (
        "ros2 service call /controller_manager/switch_controller"
        " controller_manager_msgs/srv/SwitchController"
        " \"{activate_controllers: ['forward_velocity_controller', 'finger_width_controller'],"
        " deactivate_controllers: ['scaled_joint_trajectory_controller', 'finger_width_trajectory_controller'],"
        " strictness: 1}\""
    )
    run_once("Controller Switch (teleop)", switch_cmd, retries=5, delay=3)

    # ── Phase 3: Communication bridges ───────────────────────────────
    subprocess.run(["bash", "-c", "fuser -k 10000/tcp 2>/dev/null"], capture_output=True)
    time.sleep(0.5)
    run(
        "ROS-TCP Endpoint",
        f"ros2 run ros_tcp_endpoint default_server_endpoint"
        f" --ros-args -p ROS_IP:={args.ros_ip}",
    )

    beacon_path = os.path.join(SCRIPT_DIR, "beacon.py")
    if os.path.exists(beacon_path):
        run("IP Beacon", f"python3 {beacon_path} --ip {wifi_ip}")

    # ── Phase 4: Perception + cube relay (optional) ──────────────────
    if args.perception:
        print("\n>>> Starting perception pipeline...")

        if fake:
            run(
                "Perception (sim)",
                "ros2 launch holo_assist_depth_tracker_sim"
                " sim_april_cube_perception.launch.py"
                " use_rviz:=false publish_scene_state_publisher:=false",
            )
        else:
            run(
                "Perception (hardware)",
                "ros2 launch holo_assist_depth_tracker"
                " holoassist_4tag_board_4cube.launch.py",
            )
        time.sleep(2)

        # workspace_frame TF: base_link → workspace_frame
        # Sim values: (0, -0.315, 0.02) — workspace is 31.5cm in front of robot base
        run(
            "Workspace Frame TF",
            "ros2 run tf2_ros static_transform_publisher"
            " 0 -0.315 0.02 0 0 0 base_link workspace_frame",
        )
        time.sleep(1)

        # Cube pose relay: transforms perception poses (workspace_frame) to base_link for Unity
        relay_prefix = "/holoassist/sim/perception" if fake else "/holoassist/perception"
        run(
            "Cube Pose Relay",
            f"python3 {os.path.join(SCRIPT_DIR, 'cube_pose_relay.py')}"
            f" --ros-args -p input_prefix:={relay_prefix}",
        )

    # ── Done ─────────────────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("  All running. Ctrl+C to stop everything.")
    print(f"  ROS-TCP:     port 10000 (Unity)")
    if args.perception:
        print(f"  Perception:  cube poses relayed to /holoassist/unity/cube_{{1-4}}_pose")
    print()
    if not fake:
        print("  Don't forget: run External Control on teach pendant")
    print("  Then hit Play in Unity.")
    print("=" * 65)

    critical = {"UR + OnRobot Driver", "ROS-TCP Endpoint"}
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
