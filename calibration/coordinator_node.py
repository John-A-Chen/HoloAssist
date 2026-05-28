#!/usr/bin/env python3
"""Automate MoveIt calibration poses and easy_handeye2 sampling/saving."""

import json
import math
import pathlib
import shutil
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml

import easy_handeye2 as hec
from easy_handeye2_msgs import srv as eh_srv


COMMAND_TOPIC = "/holoassist/calibration/command"
STATUS_TOPIC = "/holoassist/calibration/status"
MOVE_STATE_TOPIC = "/moveit_robot_control/state"
MOVE_STOP_TOPIC = "/moveit_robot_control/stop"
JOINT_TARGET_TOPIC = "/moveit_robot_control/target_joint_state"


class CalibrationCoordinator(Node):
    def __init__(self):
        super().__init__("holoassist_calibration_coordinator")
        self.declare_parameter("poses_file", "")
        self.declare_parameter("algorithm", "OpenCV/Tsai-Lenz")
        self.declare_parameter("settle_seconds", 0.8)
        self.declare_parameter("move_timeout_seconds", 60.0)
        self.declare_parameter("auto_save", True)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("calibration_name", "holoassist_calibration")
        self.declare_parameter("marker_frame", "tag36h11:1")

        poses_value = str(self.get_parameter("poses_file").value).strip()
        if not poses_value:
            raise ValueError("poses_file parameter is required; no automatic pose default is permitted")
        self.poses_file = pathlib.Path(poses_value)
        self.algorithm = str(self.get_parameter("algorithm").value)
        self.settle_seconds = float(self.get_parameter("settle_seconds").value)
        self.move_timeout = float(self.get_parameter("move_timeout_seconds").value)
        self.auto_save = bool(self.get_parameter("auto_save").value)
        self.calibration_name = str(self.get_parameter("calibration_name").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.joint_names, self.poses = self._load_poses()

        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.target_pub = self.create_publisher(JointState, JOINT_TARGET_TOPIC, 10)
        self.stop_pub = self.create_publisher(String, MOVE_STOP_TOPIC, 10)
        self.create_subscription(String, COMMAND_TOPIC, self._command_cb, 10)
        self.create_subscription(String, MOVE_STATE_TOPIC, self._move_state_cb, 10)

        self.sample_client = self.create_client(eh_srv.TakeSample, hec.TAKE_SAMPLE_TOPIC)
        self.sample_list_client = self.create_client(
            eh_srv.TakeSample, hec.GET_SAMPLE_LIST_TOPIC
        )
        self.compute_client = self.create_client(
            eh_srv.ComputeCalibration, hec.COMPUTE_CALIBRATION_TOPIC
        )
        self.save_client = self.create_client(
            eh_srv.SaveCalibration, hec.SAVE_CALIBRATION_TOPIC
        )
        self.algorithm_client = self.create_client(
            eh_srv.SetAlgorithm, hec.SET_ALGORITHM_TOPIC
        )

        self._lock = threading.Lock()
        self._motion_condition = threading.Condition()
        self._move_serial = 0
        self._last_move_state = ""
        self._running = False
        self._cancel = threading.Event()
        self._state = "waiting"
        self._message = "Waiting for calibration server and MoveIt"
        self._sample_count = 0
        self._pose_index = 0
        self._computed = False
        self._result = {}
        self._latest_path = ""
        self._archive_path = ""
        self._error = ""
        self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            f"Calibration automation configured for {len(self.poses)} poses and "
            f"marker {self.marker_frame}"
        )
        if bool(self.get_parameter("auto_start").value):
            self.create_timer(2.0, self._auto_start_once)

    def _load_poses(self):
        with self.poses_file.open() as stream:
            content = yaml.safe_load(stream)
        names = list(content["joint_names"])
        poses = [
            [math.radians(float(value)) for value in pose]
            for pose in content["poses_deg"]
        ]
        if len(poses) < 8 or any(len(pose) != len(names) for pose in poses):
            raise ValueError(
                f"Calibration needs at least eight varied poses in {self.poses_file}"
            )
        return names, poses

    def _services_ready(self):
        return all(
            client.service_is_ready()
            for client in (
                self.sample_client,
                self.sample_list_client,
                self.compute_client,
                self.save_client,
                self.algorithm_client,
            )
        )

    def _auto_start_once(self):
        if self._services_ready() and not self._running and self._state == "waiting":
            self._start_sequence()

    def _publish_status(self):
        ready = self._services_ready()
        with self._lock:
            payload = {
                "ready": ready,
                "state": self._state,
                "message": self._message,
                "running": self._running,
                "sample_count": self._sample_count,
                "pose_index": self._pose_index,
                "pose_total": len(self.poses),
                "computed": self._computed,
                "result": self._result,
                "latest_path": self._latest_path,
                "archive_path": self._archive_path,
                "error": self._error,
                "marker_frame": self.marker_frame,
            }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

    def _set_status(self, state=None, message=None, error=None, **updates):
        with self._lock:
            if state is not None:
                self._state = state
            if message is not None:
                self._message = message
            if error is not None:
                self._error = error
            for key, value in updates.items():
                setattr(self, f"_{key}", value)
        self._publish_status()

    def _command_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            action = str(payload.get("action", "")).strip().lower()
        except json.JSONDecodeError:
            action = msg.data.strip().lower()
        actions = {
            "start": self._start_sequence,
            "sample": self._manual_sample,
            "compute": self._compute_and_optionally_save,
            "save": self._save,
            "stop": self._stop,
        }
        operation = actions.get(action)
        if operation is None:
            self._set_status(message=f"Ignored unknown calibration command: {action}")
            return
        operation()

    def _start_worker(self, target):
        with self._lock:
            if self._running:
                self._message = "Calibration operation already running"
                return False
            self._running = True
            self._error = ""
        threading.Thread(target=self._worker_wrapper, args=(target,), daemon=True).start()
        return True

    def _worker_wrapper(self, target):
        try:
            target()
        except Exception as exc:
            self.get_logger().error(f"Calibration automation failed: {exc}")
            self._set_status(state="error", message="Calibration operation failed", error=str(exc))
        finally:
            with self._lock:
                self._running = False
            self._publish_status()

    def _start_sequence(self):
        if not self._services_ready():
            self._set_status(message="Waiting for easy_handeye2 services")
            return
        self._cancel.clear()
        self._start_worker(self._run_sequence)

    def _call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline and not self._cancel.is_set():
            time.sleep(0.02)
        if self._cancel.is_set():
            raise RuntimeError("Calibration stopped")
        if not future.done():
            raise RuntimeError("Timed out waiting for easy_handeye2 service")
        response = future.result()
        if response is None:
            raise RuntimeError("easy_handeye2 service call returned no response")
        return response

    def _set_algorithm(self):
        request = eh_srv.SetAlgorithm.Request()
        request.new_algorithm = self.algorithm
        response = self._call(self.algorithm_client, request)
        if not response.success:
            raise RuntimeError(f"Calibration algorithm rejected: {self.algorithm}")

    def _get_sample_count(self):
        response = self._call(self.sample_list_client, eh_srv.TakeSample.Request())
        return len(response.samples.samples)

    def _take_sample(self):
        before = self._get_sample_count()
        response = self._call(self.sample_client, eh_srv.TakeSample.Request())
        after = len(response.samples.samples)
        if after <= before:
            raise RuntimeError("easy_handeye2 did not add a sample; check tag/tool TFs")
        self._set_status(sample_count=after, message=f"Captured sample {after}")

    def _publish_pose(self, position):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = position
        with self._motion_condition:
            baseline = self._move_serial
        self.target_pub.publish(msg)
        return baseline

    def _wait_for_motion(self, baseline):
        deadline = time.monotonic() + self.move_timeout
        while time.monotonic() < deadline and not self._cancel.is_set():
            with self._motion_condition:
                state = self._last_move_state if self._move_serial > baseline else ""
                if state in {"COMPLETE", "DRY_RUN_COMPLETE"}:
                    return
                if state in {"FAILED", "INVALID", "REJECTED", "CANCELLED"}:
                    raise RuntimeError(f"MoveIt movement ended in {state}")
                self._motion_condition.wait(timeout=0.1)
        if self._cancel.is_set():
            raise RuntimeError("Calibration stopped")
        raise RuntimeError("Timed out waiting for MoveIt calibration pose")

    def _run_sequence(self):
        self._set_status(state="running", message="Starting automated calibration", computed=False)
        self._set_algorithm()
        existing_samples = self._get_sample_count()
        if existing_samples:
            raise RuntimeError(
                "Automatic sequence requires an empty sample set; restart the "
                "calibration stack or compute/save the current manual set"
            )
        self._set_status(sample_count=0)
        for index, pose in enumerate(self.poses, start=1):
            if self._cancel.is_set():
                raise RuntimeError("Calibration stopped")
            self._set_status(
                pose_index=index,
                message=f"Moving to calibration pose {index}/{len(self.poses)}",
            )
            baseline = self._publish_pose(pose)
            self._wait_for_motion(baseline)
            self._set_status(message=f"Settling at pose {index}/{len(self.poses)}")
            time.sleep(self.settle_seconds)
            self._take_sample()
        self._compute_impl()
        if self.auto_save:
            self._save_impl()
        else:
            self._set_status(state="computed", message="Calibration computed; press SAVE")

    def _move_state_cb(self, msg):
        with self._motion_condition:
            self._last_move_state = msg.data.strip()
            self._move_serial += 1
            self._motion_condition.notify_all()

    def _manual_sample(self):
        self._start_worker(self._manual_sample_impl)

    def _manual_sample_impl(self):
        self._take_sample()
        self._set_status(state="sampling", message="Manual sample captured")

    def _compute_and_optionally_save(self):
        self._start_worker(self._compute_impl)

    def _compute_impl(self):
        response = self._call(self.compute_client, eh_srv.ComputeCalibration.Request())
        if not response.valid:
            raise RuntimeError("No valid calibration result; capture more varied samples")
        tf = response.calibration.transform
        result = {
            "translation": [tf.translation.x, tf.translation.y, tf.translation.z],
            "rotation": [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w],
        }
        self._set_status(
            state="computed",
            message="Calibration computed",
            computed=True,
            result=result,
        )

    def _save(self):
        self._start_worker(self._save_impl)

    def _save_impl(self):
        with self._lock:
            if not self._computed:
                raise RuntimeError("Compute calibration before saving")
        calibration_dir = pathlib.Path.home() / ".ros2" / "easy_handeye2" / "calibrations"
        canonical = calibration_dir / f"{self.calibration_name}.calib"
        history = calibration_dir / "history"
        history.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        prior_path = ""
        if canonical.exists():
            prior = history / f"{self.calibration_name}_prior_{stamp}.calib"
            shutil.copy2(canonical, prior)
            prior_path = str(prior)
        response = self._call(self.save_client, eh_srv.SaveCalibration.Request())
        if not response.success:
            raise RuntimeError("easy_handeye2 failed to save calibration")
        latest = pathlib.Path(response.filepath.data)
        archive = history / f"{self.calibration_name}_{stamp}.calib"
        shutil.copy2(latest, archive)
        message = f"Saved newest calibration; history copy: {archive.name}"
        if prior_path:
            message += " (previous calibration retained)"
        self._set_status(
            state="saved",
            message=message,
            latest_path=str(latest),
            archive_path=str(archive),
        )

    def _stop(self):
        self._cancel.set()
        msg = String()
        msg.data = "stop"
        self.stop_pub.publish(msg)
        self._set_status(state="stopped", message="Calibration stopped")


def main():
    rclpy.init()
    node = CalibrationCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
