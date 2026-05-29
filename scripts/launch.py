#!/usr/bin/env python3
"""
HoloAssist ROS 2 launcher.

Starts:
  - UR3e + OnRobot RG2 driver (fake or real hardware)
  - Controller switch (velocity + gripper for teleop)
  - ROS-TCP endpoint (Unity bridge, port 10000)
  - UDP beacon (Quest auto-discovery)
  - [optional] Perception pipeline (--perception)
  - [optional] MoveIt pick/place stack (--moveit)

Default mode: clean one-line status per process, logs to /tmp/holoassist_*.log
Verbose mode: full subprocess output in terminal (--verbose)
"""

import argparse
import os
import re
import subprocess
import signal
import sys
import time
import socket

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
ROS2_WS = os.path.join(REPO_ROOT, "ros2_ws")
SOURCE_CMD = (
    f"unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH"
    f" && source /opt/ros/humble/setup.bash"
    f" && source {ROS2_WS}/install/setup.bash"
)

DEFAULT_WIFI_IP = "10.84.45.11"
DEFAULT_ROBOT_IP = "192.168.0.194"
PREFERRED_SUBNET = "192.168.0."

_WEBCAM_PRESETS = {
    "brio":    (78.0,  848, 480,  25.0),
    "generic": (69.4,  640, 480,  25.0),
}

processes = []
_pgids: set = set()  # all process group IDs we've spawned, for final cleanup
_verbose = False  # set by --verbose in main()
_NW = 42          # name column width for aligned one-liners

# Lines filtered from verbose output — harmless FastDDS SHM init noise
_NOISE_PATTERNS = [
    "RTPS_TRANSPORT_SHM",
    "open_and_lock_file",
    "open_port_internal",
    "Failed init_port",
]


# ── Output helpers ────────────────────────────────────────────────────

def _log_slug(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def _pstart(name: str, log_path: str = "") -> None:
    if log_path:
        print(f"  ▸  {name:<{_NW}}  → {log_path}", flush=True)
    else:
        print(f"  ▸  {name}", flush=True)


def _pok(name: str, detail: str = "OK") -> None:
    print(f"  ✓  {name:<{_NW}}  {detail}", flush=True)


def _pfail(name: str, detail: str = "") -> None:
    suffix = f"  {detail}" if detail else ""
    print(f"  ✗  {name}{suffix}", flush=True)


def _pinfo(msg: str) -> None:
    print(f"       {msg}", flush=True)


# ── Camera detection ──────────────────────────────────────────────────

def _detect_camera() -> dict:
    """
    Probe plugged-in cameras. Returns best one found by priority:
      Intel RealSense > Logitech Brio > other UVC webcam > none
    """
    import glob

    try:
        import pyrealsense2 as rs
        ctx = rs.context()
        devs = ctx.query_devices()
        if len(devs) > 0:
            try:
                name = devs[0].get_info(rs.camera_info.name)
            except Exception:
                name = "Intel RealSense"
            return dict(type="realsense", index=None, label=name, preset=None)
    except Exception:
        pass

    v4l = []
    for path in sorted(glob.glob("/sys/class/video4linux/video*/name")):
        try:
            name = open(path).read().strip()
            idx = int(path.split("/")[4].replace("video", ""))
            v4l.append((idx, name))
        except Exception:
            pass

    for idx, name in v4l:
        if "realsense" in name.lower():
            return dict(type="realsense", index=None,
                        label=f"Intel RealSense ({name})", preset=None)
    for idx, name in v4l:
        if "brio" in name.lower():
            return dict(type="brio", index=idx,
                        label=f"Logitech Brio ({name}, /dev/video{idx})", preset="brio")
    for idx, name in v4l:
        if any(k in name.lower() for k in ("webcam", "cam", "uvc", "usb video")):
            return dict(type="webcam", index=idx,
                        label=f"{name} (/dev/video{idx})", preset="generic")

    return dict(type=None, index=None, label="none detected", preset=None)


# ── Network ───────────────────────────────────────────────────────────

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


# ── Process management ────────────────────────────────────────────────

def _wait_for_driver(timeout: int = 30) -> bool:
    """Poll until joint_state_broadcaster is active in controller_manager."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            r = subprocess.run(
                ["bash", "-c", f"{SOURCE_CMD} && ros2 control list_controllers 2>/dev/null"],
                capture_output=True, text=True, timeout=5,
            )
            if "joint_state_broadcaster" in r.stdout and "[active]" in r.stdout:
                return True
        except Exception:
            pass
        time.sleep(0.5)
    return False


def _preexec():
    os.setsid()  # new process group so cleanup() can kill the whole tree
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def run(name: str, cmd: str, always_quiet: bool = False) -> subprocess.Popen:
    """Launch a bash subprocess and track it.

    Default mode  — all output redirected to /tmp/holoassist_<name>.log.
    Verbose mode  — output goes to the terminal, with FastDDS SHM noise filtered.
    always_quiet  — redirect even in verbose mode (for trivial TF publishers etc).
    """
    show_inline = _verbose and not always_quiet

    if show_inline:
        print(f"\n>>> Starting {name}")
        print(f"    {cmd}\n")
        noise = " | ".join(f"grep -v '{p}'" for p in _NOISE_PATTERNS)
        full_cmd = f"({SOURCE_CMD} && {cmd}) 2>&1 | {noise}; exit ${{PIPESTATUS[0]}}"
        kwargs = {}
    else:
        log_path = f"/tmp/holoassist_{_log_slug(name)}.log"
        log_file = open(log_path, "w")
        kwargs = {"stdout": log_file, "stderr": log_file}
        if not _verbose:
            _pstart(name, log_path)
        full_cmd = f"{SOURCE_CMD} && {cmd}"

    proc = subprocess.Popen(
        ["bash", "-c", full_cmd],
        preexec_fn=_preexec,
        **kwargs,
    )
    processes.append((name, proc))
    try:
        _pgids.add(os.getpgid(proc.pid))
    except Exception:
        pass
    return proc


def run_once(name: str, cmd: str, retries: int = 1, delay: int = 5, timeout: int = 20) -> bool:
    """Run a command once (blocking), retry on failure."""
    result = None
    for attempt in range(1 + retries):
        try:
            result = subprocess.run(
                ["bash", "-c", f"{SOURCE_CMD} && {cmd}"],
                capture_output=True, text=True, timeout=timeout,
            )
        except subprocess.TimeoutExpired:
            if attempt < retries:
                if _verbose:
                    print(f">>> {name}: timed out, retrying in {delay}s...")
                else:
                    _pinfo(f"timed out, retrying in {delay}s…")
                time.sleep(delay)
                continue
            _pfail(name, f"timed out after {timeout}s")
            return False
        if result.returncode == 0:
            if _verbose:
                print(f">>> {name}: OK")
            else:
                _pok(name)
            return True
        if attempt < retries:
            if _verbose:
                print(f">>> {name}: failed, retrying in {delay}s...")
            else:
                _pinfo(f"retrying in {delay}s…")
            time.sleep(delay)
    err_line = ((result.stderr.strip().splitlines() or ["unknown error"])[-1]) if result else "no result"
    if _verbose:
        print(f">>> {name}: FAILED — {(result.stderr.strip() if result else 'no result')}")
    else:
        _pfail(name, err_line[:72])
    return False


def _kill_group(proc, sig):
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, sig)
    except ProcessLookupError:
        pass
    except Exception:
        try:
            proc.send_signal(sig)
        except Exception:
            pass


def cleanup(*_):
    if _verbose:
        print("\n\n>>> Shutting down all processes...")
    else:
        print("\n")
        print("=" * 65)
        print("  Shutting down...")
        print("=" * 65)

    # Phase 1: SIGTERM to all tracked process groups simultaneously
    for name, proc in reversed(processes):
        if proc.poll() is None:
            if _verbose:
                try:
                    pgid = os.getpgid(proc.pid)
                    print(f"    Stopping {name} (pgid {pgid})")
                except Exception:
                    print(f"    Stopping {name}")
            else:
                _pinfo(f"stopping  {name}")
            _kill_group(proc, signal.SIGTERM)

    # Phase 2: wait up to 3s total for all processes, then SIGKILL stragglers
    deadline = time.time() + 3.0
    for name, proc in processes:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        try:
            proc.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            pass

    # Phase 3: SIGKILL everything still alive — both tracked procs and any
    # grandchildren ros2 launch left behind in their own process groups
    for pgid in list(_pgids):
        try:
            os.killpg(pgid, signal.SIGKILL)
        except Exception:
            pass
    for name, proc in processes:
        try:
            proc.kill()
        except Exception:
            pass

    # Kill any lingering RViz windows (ros2 launch children may survive pgid kill)
    subprocess.run(["pkill", "-9", "-f", "rviz2"], capture_output=True)

    # Kill any calibration processes spawned independently (e.g. via LAUNCH STACK button)
    for pattern in ["coordinator_node.py", "waypoint_publisher_node.py", "handeye_server"]:
        subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)

    if _verbose:
        print(">>> All stopped.")
    else:
        print()
        print("  All stopped.")
        print("=" * 65)
    sys.exit(0)


# ── Entry point ───────────────────────────────────────────────────────

def main():
    global _verbose

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
        "--no-perception", dest="perception", action="store_false",
        help="Disable the perception pipeline (camera + AprilTag + cube pose).",
    )
    parser.add_argument(
        "--no-moveit", dest="moveit", action="store_false",
        help="Disable MoveIt pick/place stack.",
    )
    parser.set_defaults(perception=True, moveit=True)
    parser.add_argument(
        "--verbose", action="store_true",
        help="Print full subprocess output to the terminal (ROS driver logs, node output, etc).",
    )
    parser.add_argument(
        "--no-dashboard", dest="dashboard", action="store_false",
        help="Disable the e-stop dashboard.",
    )
    parser.add_argument(
        "--dashboard-fullscreen", action="store_true",
        help="Open the dashboard in fullscreen mode.",
    )
    parser.set_defaults(dashboard=True)
    parser.add_argument(
        "--fake-gripper", action="store_true",
        help="With a real/URSim arm, use fake OnRobot gripper hardware.",
    )
    args = parser.parse_args()
    _verbose = args.verbose

    open_dashboard = args.dashboard_fullscreen or args.dashboard
    dashboard_fullscreen = args.dashboard_fullscreen

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
    if not fake:
        print(f"  Gripper:     {'fake' if args.fake_gripper else 'real'}")
    print(f"  RViz:        {'off' if args.no_rviz else 'on'}")
    if args.perception:
        cam = _detect_camera()
        if cam["type"] is None and fake:
            cam_label = f"on (sim — {cam['label']})"
        elif cam["type"] is None:
            cam_label = "on — WARNING: no camera detected"
        else:
            cam_label = f"on ({cam['label']})"
    else:
        cam = dict(type=None, index=None, label="", preset=None)
        cam_label = "off"
    print(f"  Perception:  {cam_label}")
    print(f"  MoveIt:      {'on' if args.moveit else 'off'}")
    print(f"  Dashboard:   {'on (fullscreen)' if dashboard_fullscreen else 'on  (bridge on :9090)' if open_dashboard else 'off  (--no-dashboard to hide)'}")
    print(f"  Verbose:     {'on' if _verbose else 'off  (--verbose for full node logs)'}")
    print("=" * 65)
    if not _verbose:
        print()

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ── Dashboard (first, so it's live when everything else comes up) ──
    if open_dashboard:
        subprocess.run(["bash", "-c", "fuser -k 9090/tcp 2>/dev/null"], capture_output=True)
        time.sleep(0.3)
        bridge_path = os.path.join(REPO_ROOT, "dashboard", "bridge_server.py")
        run("Bridge", f"python3 {bridge_path}")
        dashboard_path = os.path.join(REPO_ROOT, "dashboard", "main.py")
        dashboard_cmd = f"python3 {dashboard_path}"
        if dashboard_fullscreen:
            dashboard_cmd += " --fullscreen"
        run("Dashboard", dashboard_cmd)
        time.sleep(1)  # give Qt a moment to open before the terminal fills up

    # ── Phase 1: UR + OnRobot driver ─────────────────────────────────
    # Release UR reverse/script/trajectory ports from any previous run before binding.
    subprocess.run(
        ["bash", "-c", "fuser -k 50001/tcp 50003/tcp 50004/tcp 2>/dev/null"],
        capture_output=True,
    )
    time.sleep(0.5)

    # Only one RViz instance at a time:
    #   --moveit     → MoveIt stack owns RViz (has SRDF pre-loaded)
    #   --perception → perception launch owns RViz (has cube/camera displays)
    #   neither      → driver opens RViz
    driver_rviz = "false" if (args.no_rviz or args.moveit or args.perception) else "true"
    driver_cmd = (
        f"ros2 launch ur_onrobot_control start_robot.launch.py"
        f" ur_type:=ur3e onrobot_type:=rg2 launch_rviz:={driver_rviz}"
        f" base_yaw_rad:=0.0"
    )
    if fake:
        driver_cmd += " use_fake_hardware:=true"
    else:
        driver_cmd += f" robot_ip:={robot_ip}"
        if args.fake_gripper:
            driver_cmd += " use_fake_gripper_hardware:=true"
    run("UR + OnRobot Driver", driver_cmd)

    run(
        "Trolley Scene Publisher",
        "ros2 run holoassist_perception holoassist_trolley_scene_publisher",
    )

    if _verbose:
        print("\n>>> Waiting for UR driver (joint_state_broadcaster active)...")
    else:
        _pinfo("waiting for UR driver…")
    if not _wait_for_driver(timeout=30):
        _pinfo("warning: UR driver readiness check timed out (30s), continuing anyway")

    # ── Phase 2: Controller switch (real hardware, teleop-only start) ──
    # When --moveit is set, the driver already starts with scaled_joint_trajectory_controller
    # active (the default), so MoveIt can use it immediately.  Switching to velocity
    # controllers here would break MoveIt until the dashboard MOVEIT button is pressed.
    # When --moveit is NOT set, switch to velocity + gripper controllers for teleop.
    if args.moveit:
        _pinfo("moveit mode — keeping trajectory controllers (scaled_joint + finger_width_traj)")
    else:
        switch_cmd = (
            "ros2 service call /controller_manager/switch_controller"
            " controller_manager_msgs/srv/SwitchController"
            " \"{activate_controllers: ['forward_velocity_controller', 'finger_width_controller'],"
            " deactivate_controllers: ['scaled_joint_trajectory_controller', 'finger_width_trajectory_controller'],"
            " strictness: 1}\""
        )
        run_once("Controller Switch", switch_cmd, retries=5, delay=3, timeout=20)

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

    # ── Phase 4: Perception (optional) ───────────────────────────────
    if args.perception:
        if _verbose:
            print(f"\n>>> Starting perception pipeline...")
        else:
            print()
            _pinfo("── perception ──────────────────────────────────────────")

        if cam["type"] is None:
            _pfail("Perception", "no camera detected — skipping")
        else:
            rviz_flag_perception = "false" if args.no_rviz else "true"

            if cam["type"] in ("brio", "webcam"):
                hfov, w, h, fps = _WEBCAM_PRESETS[cam["preset"]]
                run(
                    f"Camera ({cam['label']})",
                    f"ros2 run holoassist_perception holoassist_webcam_image_publisher"
                    f" --ros-args"
                    f" -p device_index:={cam['index']}"
                    f" -p image_topic:=/camera/camera/color/image_raw"
                    f" -p camera_info_topic:=/camera/camera/color/camera_info"
                    f" -p frame_id:=camera_color_optical_frame"
                    f" -p hfov_deg:={hfov}"
                    f" -p width:={w} -p height:={h} -p fps:={fps}",
                )
                time.sleep(3)

            calib_file = os.path.expanduser(
                "~/.ros2/easy_handeye2/calibrations/holoassist_calibration.calib"
            )
            if os.path.exists(calib_file):
                try:
                    import yaml
                    with open(calib_file) as f:
                        calib = yaml.safe_load(f)
                    t = calib["transform"]["translation"]
                    r = calib["transform"]["rotation"]
                    calib_target = (
                        str(calib.get("parameters", {}).get("tracking_base_frame", ""))
                        or "camera_link"
                    )
                    run(
                        "Camera Calibration TF",
                        f"ros2 run tf2_ros static_transform_publisher"
                        f" --x {t['x']} --y {t['y']} --z {t['z']}"
                        f" --qx {r['x']} --qy {r['y']} --qz {r['z']} --qw {r['w']}"
                        f" --frame-id base_link --child-frame-id {calib_target}",
                    )
                    if not _verbose:
                        _pinfo(f"calibration: base_link → {calib_target}")
                    # Webcam: RealSense driver publishes camera_link→optical TF automatically.
                    if cam["type"] != "realsense" and calib_target == "camera_link":
                        run(
                            "Camera Link → Optical TF",
                            "ros2 run tf2_ros static_transform_publisher"
                            " --x 0 --y 0 --z 0"
                            " --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5"
                            " --frame-id camera_link"
                            " --child-frame-id camera_color_optical_frame",
                            always_quiet=True,
                        )
                    time.sleep(1)
                except Exception as e:
                    _pfail("Camera Calibration TF", str(e))
            else:
                _pinfo("no calibration found — run ./scripts/calibrate.sh to calibrate")

            start_camera = "false" if cam["type"] in ("brio", "webcam") else "true"
            run(
                f"Perception ({cam['type']})",
                f"ros2 launch holoassist_perception perception.launch.py"
                f" start_camera:={start_camera}"
                f" start_rviz:={rviz_flag_perception}",
            )
            time.sleep(5)

    # ── Phase 5: MoveIt (optional) ───────────────────────────────────
    if args.moveit:
        if _verbose:
            print("\n>>> Starting MoveIt stack...")
        else:
            print()
            _pinfo("── moveit ──────────────────────────────────────────────")

        # scaled_joint_trajectory_controller is active on both real and fake hardware
        # (the UR driver spawner activates it in both modes).  Only the safety
        # checks need to be skipped for fake hardware.
        moveit_extra = " require_robot_status:=false require_controller_check:=false" if fake else ""

        # Perception owns RViz when both --perception and --moveit are active.
        # MoveIt only opens its own RViz when running standalone (no --perception).
        moveit_rviz = "false" if (args.no_rviz or args.perception) else "true"
        view_robot_rviz = os.path.join(
            ROS2_WS,
            "install/holoassist_movement/share"
            "/holoassist_movement/rviz/view_robot.rviz",
        )
        moveit_cmd = (
            "ros2 launch holoassist_movement"
            " movement.launch.py"
            f" robot_ip:={robot_ip}"
            f" use_rviz:={moveit_rviz}"
            + (f" rviz_config:={view_robot_rviz}" if moveit_rviz == "true" else "")
            + moveit_extra
        )
        run("MoveIt", moveit_cmd)
        if not args.perception:
            _pinfo("warning: perception is off; pick_cube_to_bin will wait for cube poses")

    # ── Done ─────────────────────────────────────────────────────────
    print()
    print("=" * 65)
    print("  All running. Ctrl+C to stop everything.")
    print(f"  ROS-TCP:     port 10000 (Unity)")
    if args.perception:
        print(f"  Perception:  cube poses on /holoassist/perception/april_cube_{{1-4}}_pose")
    if args.moveit:
        print(f"  MoveIt:      /holoassist/pick_cube_to_bin  (reads real cube poses from perception)")
    print()
    if not fake:
        print("  Don't forget: run External Control on teach pendant")
    print("  Then hit Play in Unity.")
    if not _verbose:
        print()
        print("  Logs: /tmp/holoassist_*.log")
    print("=" * 65)

    # Wait for rviz2 to start, then monitor it — closing RViz shuts everything down.
    _rviz_pid = None
    if not args.no_rviz:
        for _ in range(20):
            r = subprocess.run(["pgrep", "-f", "rviz2"], capture_output=True, text=True)
            if r.returncode == 0:
                _rviz_pid = int(r.stdout.strip().split()[0])
                break
            time.sleep(1)

    critical = {"UR + OnRobot Driver", "ROS-TCP Endpoint"}
    while True:
        for name, proc in list(processes):
            ret = proc.poll()
            if ret is not None:
                if _verbose:
                    print(f"\n>>> {name} exited (code {ret})")
                else:
                    print()
                    _pfail(f"{name} has stopped", f"(code {ret})")
                if name in critical:
                    if _verbose:
                        print(f"    Critical process died — shutting down.")
                    else:
                        _pinfo("critical — shutting down")
                    cleanup()
                else:
                    processes.remove((name, proc))

        if _rviz_pid is not None:
            try:
                os.kill(_rviz_pid, 0)
            except ProcessLookupError:
                _pinfo("RViz closed — shutting down")
                cleanup()

        time.sleep(1)


if __name__ == "__main__":
    main()
