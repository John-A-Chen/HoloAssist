#!/usr/bin/env python3
"""
WebSocket bridge server — runs on the laptop alongside ROS 2.
Forwards dashboard ROS data to the Steam Deck over USB-C Ethernet.

Usage:
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    python3 dashboard/bridge_server.py [--port 9090] [--push-hz 30]

Protocol:
    Laptop → Deck:
        Text frames:  JSON status snapshot (DashboardStatus fields)
        Binary frames: 0x01 + JPEG payload (headset image)
    Deck → Laptop:
        Text frames:  JSON commands {"cmd": "estop"/"resume"/...}
"""

import asyncio
import json
import sys
import time

try:
    import websockets
    from websockets import serve
except ImportError:
    print("ERROR: 'websockets' package required.  pip install websockets")
    sys.exit(1)

from ros_interface import RosInterface, ROS_AVAILABLE

BINARY_HEADSET = 0x01


class BridgeServer:
    def __init__(self, ros: RosInterface, host: str = "0.0.0.0",
                 port: int = 9090, push_hz: float = 30):
        self.ros = ros
        self.host = host
        self.port = port
        self.push_interval = 1.0 / push_hz
        self.clients: set = set()

    async def _handler(self, ws):
        self.clients.add(ws)
        addr = ws.remote_address
        print(f"[bridge] client connected: {addr}")
        try:
            async for message in ws:
                try:
                    self._handle_command(json.loads(message))
                except json.JSONDecodeError:
                    pass
        except websockets.ConnectionClosed:
            pass
        finally:
            self.clients.discard(ws)
            print(f"[bridge] client disconnected: {addr}")

    def _handle_command(self, msg: dict):
        cmd = msg.get("cmd")
        if cmd == "estop":
            self.ros.emergency_stop()
        elif cmd == "resume":
            self.ros.resume()
        elif cmd == "switch_teleop":
            self.ros.switch_to_teleop()
        elif cmd == "switch_moveit":
            self.ros.switch_to_moveit()
        elif cmd == "pick_cube":
            self.ros.pick_cube_to_bin(msg.get("cube_id"), msg.get("bin_id"))
        elif cmd == "calibration":
            self.ros.calibration_command(msg.get("action", ""))
        elif cmd == "reconfigure_camera":
            self.ros.reconfigure_camera(
                int(msg.get("width", 640)),
                int(msg.get("height", 480)),
                float(msg.get("fps", 30)),
            )

    async def _push_loop(self):
        while True:
            if self.clients:
                status = self.ros.get_status()

                text = json.dumps(self._serialize(status))
                headset_frame = None
                if status.headset_jpeg:
                    headset_frame = bytes([BINARY_HEADSET]) + status.headset_jpeg

                dead = set()
                for ws in list(self.clients):
                    try:
                        await ws.send(text)
                        if headset_frame:
                            await ws.send(headset_frame)
                    except websockets.ConnectionClosed:
                        dead.add(ws)
                self.clients -= dead

            await asyncio.sleep(self.push_interval)

    @staticmethod
    def _serialize(status) -> dict:
        topic_rates = {}
        for name, ts in status.topic_rates.items():
            topic_rates[name] = {
                "rx_count": ts.rx_count,
                "last_rx_time": ts.last_rx_time,
                "hz": ts.hz,
            }

        return {
            "t": "s",
            "robot_state": status.robot_state.name,
            "ros_connected": status.ros_connected,
            "controller_active": status.controller_active,
            "joint_names": status.joint_names,
            "joint_positions": status.joint_positions,
            "joint_velocities": status.joint_velocities,
            "last_joint_time": status.last_joint_time,
            "joint_hz": status.joint_hz,
            "events": status.events,
            "topic_rates": topic_rates,
            "camera_width": status.camera_width,
            "camera_height": status.camera_height,
            "eef_pose": status.eef_pose,
            "target_pose": status.target_pose,
            "last_target_age_s": status.last_target_age_s,
            "last_twist_age_s": status.last_twist_age_s,
            "unity_map_loaded": status.unity_map_loaded,
            "session_info": status.session_info,
            "gripper_value": status.gripper_value,
            "gripper_grips": status.gripper_grips,
            "collision_scale": status.collision_scale,
            "collision_blocked": status.collision_blocked,
            "collision_events": status.collision_events,
            "ee_locked": status.ee_locked,
            "ee_lock_count": status.ee_lock_count,
            "operating_mode": status.operating_mode,
            "pick_service_ready": status.pick_service_ready,
            "pick_request_pending": status.pick_request_pending,
            "last_pick_cube": status.last_pick_cube,
            "last_pick_success": status.last_pick_success,
            "last_pick_message": status.last_pick_message,
            "pick_place_status": status.pick_place_status,
            "pick_place_status_lines": status.pick_place_status_lines,
            "pick_place_block_id": status.pick_place_block_id,
            "pick_place_destination": status.pick_place_destination,
            "pick_place_step": status.pick_place_step,
            "pick_place_step_label": status.pick_place_step_label,
            "pick_place_step_index": status.pick_place_step_index,
            "pick_place_step_total": status.pick_place_step_total,
            "pick_place_state": status.pick_place_state,
            "pick_place_error": status.pick_place_error,
            "pick_place_error_detail": status.pick_place_error_detail,
            "calibration_ready": status.calibration_ready,
            "calibration_running": status.calibration_running,
            "calibration_state": status.calibration_state,
            "calibration_message": status.calibration_message,
            "calibration_sample_count": status.calibration_sample_count,
            "calibration_pose_index": status.calibration_pose_index,
            "calibration_pose_total": status.calibration_pose_total,
            "calibration_computed": status.calibration_computed,
            "calibration_result": status.calibration_result,
            "calibration_latest_path": status.calibration_latest_path,
            "calibration_archive_path": status.calibration_archive_path,
            "calibration_error": status.calibration_error,
            "calibration_marker_frame": status.calibration_marker_frame,
            "camera_type": status.camera_type,
            "headset_type": status.headset_type,
            "velocity_history": status.velocity_history,
            "rate_history": status.rate_history,
            "latency_history": status.latency_history,
        }

    async def run(self):
        try:
            async with serve(self._handler, self.host, self.port,
                             max_size=10 * 1024 * 1024):
                print(f"[bridge] listening on ws://{self.host}:{self.port}", flush=True)
                await self._push_loop()
        except OSError as e:
            if e.errno == 98:
                print(f"[bridge] ERROR: port {self.port} already in use — "
                      f"run: fuser -k {self.port}/tcp", flush=True)
            else:
                print(f"[bridge] ERROR: {e}", flush=True)
            raise


def main():
    import argparse
    parser = argparse.ArgumentParser(description="HoloAssist WebSocket bridge")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--push-hz", type=float, default=30)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--no-ros", action="store_true",
                        help="Run without ROS (serves empty status)")
    args = parser.parse_args()

    ros = RosInterface()
    if not args.no_ros:
        if ROS_AVAILABLE:
            ros.start()
        else:
            print("[bridge] WARNING: rclpy not available — serving empty status")
    else:
        print("[bridge] --no-ros: serving empty status")

    bridge = BridgeServer(ros, host=args.host, port=args.port,
                          push_hz=args.push_hz)

    try:
        asyncio.run(bridge.run())
    except KeyboardInterrupt:
        print("\n[bridge] shutting down")
    finally:
        ros.shutdown()


if __name__ == "__main__":
    main()
