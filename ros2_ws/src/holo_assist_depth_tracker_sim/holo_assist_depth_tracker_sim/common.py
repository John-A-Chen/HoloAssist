from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Tuple

from geometry_msgs.msg import Pose, Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler


CUBE_NAMES = (
    "april_cube_1",
    "april_cube_2",
    "april_cube_3",
    "april_cube_4",
)

CUBE_COLORS: Dict[str, Tuple[float, float, float, float]] = {
    "april_cube_1": (1.0, 0.0, 0.0, 0.65),
    "april_cube_2": (0.0, 1.0, 0.0, 0.65),
    "april_cube_3": (0.1, 0.35, 1.0, 0.65),
    "april_cube_4": (1.0, 0.7, 0.0, 0.65),
}


@dataclass
class CubeState:
    name: str
    x: float
    y: float
    z: float
    yaw: float


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = quaternion_from_euler(0.0, 0.0, yaw)
    out = Quaternion()
    out.x = float(q[0])
    out.y = float(q[1])
    out.z = float(q[2])
    out.w = float(q[3])
    return out


def yaw_from_quaternion(q: Quaternion) -> float:
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return float(yaw)


def cube_state_to_pose(cube: CubeState) -> Pose:
    pose = Pose()
    pose.position.x = float(cube.x)
    pose.position.y = float(cube.y)
    pose.position.z = float(cube.z)
    pose.orientation = quaternion_from_yaw(cube.yaw)
    return pose


def normalize_angle(angle_rad: float) -> float:
    out = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if out < 0.0:
        out += 2.0 * math.pi
    return out - math.pi
