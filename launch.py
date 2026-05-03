#!/usr/bin/env python3
"""
HoloAssist unified ROS 2 launcher.

Starts the full integrated stack:
  - UR3e + OnRobot RG2 driver
  - Controller switch (teleop mode by default)
  - ROS-TCP endpoint (Unity bridge, port 10000)
  - MoveIt 2 (move_group for autonomous mode)
  - Perception pipeline (RealSense + AprilTag + cube poses)
  - Cube pose relay (workspace_frame -> base_link for Unity)
  - Workspace scene manager (trolley collision mesh)
  - Coordinate listener (MoveIt goal execution)
  - Pick-place sequencer + service (autonomous sorting)
  - Auto-sort orchestrator (triggers picks when in MOVEIT mode)
  - rosbridge WebSocket server (port 9090)
  - UDP beacon (Quest auto-discovery)

Default mode is TELEOP (forward_velocity_controller active).
Switch to MOVEIT via dashboard buttons or Quest 3 radial menu.
"""

import argparse
import subprocess
import signal
import sys
import time
import socket

ROS2_WS = "/home/nic/git/RS2-HoloAssist/nic/ros2_ws"
NIC_DIR = "/home/nic/git/RS2-HoloAssist/nic"
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
    parser = argparse.ArgumentParser(description="HoloAssist Unified ROS 2 Launcher")
    parser.add_argument(
        "--robot-ip", type=str, default=None,
        help=f"Real robot IP (e.g. {DEFAULT_ROBOT_IP}). Omit for fake hardware.",
    )
    parser.add_argument(
        "--ros-ip", type=str, default="0.0.0.0",
        help="ROS TCP endpoint bind IP (default: 0.0.0.0)",
    )
    parser.add_argument("--no-rviz", action="store_true", help="Disable RViz")
    parser.add_argument("--no-perception", action="store_true", help="Skip perception pipeline")
    parser.add_argument("--no-moveit", action="store_true", help="Skip MoveIt + autonomous sorting")
    parser.add_argument("--no-rosbridge", action="store_true", help="Skip rosbridge WebSocket")
    parser.add_argument("--no-autosort", action="store_true", help="Skip auto-sort orchestrator")
    parser.add_argument(
        "--rosbridge-port", type=int, default=9090,
        help="rosbridge WebSocket port (default: 9090)",
    )
    args = parser.parse_args()

    fake = args.robot_ip is None
    robot_ip = "0.0.0.0" if fake else args.robot_ip
    wifi_ip = get_wifi_ip()

    print("=" * 65)
    print("  HoloAssist Unified Launcher")
    print("=" * 65)
    print(f"  Mode:        {'FAKE HARDWARE' if fake else 'REAL ROBOT'}")
    if not fake:
        print(f"  Robot IP:    {robot_ip}")
    print(f"  ROS IP:      {args.ros_ip}")
    print(f"  WiFi IP:     {wifi_ip}  <-- set this in Unity ROS Settings")
    print(f"  RViz:        {'off' if args.no_rviz else 'on'}")
    print(f"  Perception:  {'off' if args.no_perception else 'on'}")
    print(f"  MoveIt:      {'off' if args.no_moveit else 'on'}")
    print(f"  rosbridge:   {'off' if args.no_rosbridge else f'port {args.rosbridge_port}'}")
    print(f"  Auto-sort:   {'off' if args.no_autosort else 'on'}")
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

    # ── Phase 2: Switch to teleop controllers (default mode) ─────────
    run_once(
        "Controller switch (TELEOP)",
        "ros2 control switch_controllers"
        " --activate forward_velocity_controller finger_width_controller"
        " --deactivate scaled_joint_trajectory_controller finger_width_trajectory_controller",
        retries=2, delay=5,
    )

    # ── Phase 3: Communication bridges ───────────────────────────────
    run(
        "ROS-TCP Endpoint",
        f"ros2 run ros_tcp_endpoint default_server_endpoint"
        f" --ros-args -p ROS_IP:={args.ros_ip}",
    )

    run("IP Beacon", f"python3 {NIC_DIR}/beacon.py --ip {wifi_ip}")

    if not args.no_rosbridge:
        run(
            "rosbridge WebSocket",
            f"ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
            f" port:={args.rosbridge_port}",
        )

    # ── Phase 4: MoveIt (needed for autonomous mode) ─────────────────
    if not args.no_moveit:
        print("\n>>> Starting MoveIt stack...")
        moveit_cmd = (
            f"ros2 launch ur_onrobot_moveit_config ur_onrobot_moveit.launch.py"
            f" ur_type:=ur3e onrobot_type:=rg2"
            f" launch_rviz:=false launch_servo:=false"
        )
        if fake:
            moveit_cmd += " use_fake_hardware:=true"
        else:
            moveit_cmd += f" robot_ip:={robot_ip}"
        run("MoveIt 2", moveit_cmd)
        time.sleep(3)

    # ── Phase 5: Perception pipeline ─────────────────────────────────
    if not args.no_perception:
        print("\n>>> Starting perception pipeline...")
        if fake:
            run(
                "Perception (sim)",
                "ros2 launch moveit_robot_control full_holoassist_moveit_sim.launch.py"
                " --launch-arguments start_robot:=false start_moveit:=false"
                " start_coordinate_listener:=false",
            )
        else:
            run(
                "Perception (hardware)",
                "ros2 launch holo_assist_depth_tracker holoassist_4tag_board_4cube.launch.py",
            )
        time.sleep(2)

        # Cube pose relay (workspace_frame -> base_link for Unity)
        run("Cube Pose Relay", f"python3 {NIC_DIR}/cube_pose_relay.py")

    # ── Phase 6: MoveIt execution nodes ──────────────────────────────
    if not args.no_moveit:
        print("\n>>> Starting MoveIt execution nodes...")

        # Workspace scene manager (trolley collision mesh)
        hw_config = f"{ROS2_WS}/install/moveit_robot_control/share/moveit_robot_control/config"
        if fake:
            config_file = f"{hw_config}/full_holoassist_sim.yaml"
        else:
            config_file = f"{hw_config}/full_holoassist_hw.yaml"

        run(
            "Workspace Scene Manager",
            f"ros2 run moveit_robot_control workspace_scene_manager"
            f" --ros-args --params-file {config_file}",
        )

        time.sleep(2)

        # Coordinate listener (receives MoveIt goals, executes trajectories)
        traj_topic = "/scaled_joint_trajectory_controller/joint_trajectory"
        run(
            "Coordinate Listener",
            f"ros2 launch moveit_robot_control coordinate_listener.launch.py"
            f" move_group_name:=ur_onrobot_manipulator"
            f" ee_link:=gripper_tcp frame:=base_link"
            f" trajectory_topic:={traj_topic}"
            f" velocity_scale:=0.05"
            f" require_robot_status:={'true' if not fake else 'false'}"
            f" require_controller_check:={'true' if not fake else 'false'}",
        )

        time.sleep(2)

        # Pick-place sequencer (state machine for autonomous sorting)
        run(
            "Pick-Place Sequencer",
            "ros2 run moveit_robot_control pick_place_sequencer"
            " --ros-args -p initial_mode:=stop -p orientation_mode:=auto"
            " -p pregrasp_z_offset:=0.10 -p grasp_z_offset:=0.0"
            " -p place_above_z_offset:=0.15 -p place_z_offset:=0.05",
        )

        # Pick-place service (bridges service calls to sequencer commands)
        run(
            "Pick-Place Service",
            "ros2 run holo_assist_depth_tracker_sim pick_place_service_node"
            " --ros-args -p cube_pose_topic_prefix:=/holoassist/perception",
        )

        # Auto-sort orchestrator
        if not args.no_autosort:
            run("Auto-Sort Orchestrator", f"python3 {NIC_DIR}/auto_sort.py")

    # ── Done ─────────────────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("  All running. Ctrl+C to stop everything.")
    print(f"  ROS-TCP:     port 10000 (Unity)")
    if not args.no_rosbridge:
        print(f"  rosbridge:   port {args.rosbridge_port} (WebSocket)")
    print(f"  IP Beacon:   broadcasting {wifi_ip}:10000")
    print()
    print("  Default mode: TELEOP (velocity control)")
    print("  Switch to MOVEIT via dashboard or Quest 3 menu")
    print()
    if not fake:
        print("  Don't forget: run External Control on teach pendant")
    print("  Then hit Play in Unity.")
    print("=" * 65)

    while True:
        for name, proc in processes:
            ret = proc.poll()
            if ret is not None:
                print(f"\n>>> {name} exited (code {ret})")
                cleanup()
        time.sleep(1)


if __name__ == "__main__":
    main()
