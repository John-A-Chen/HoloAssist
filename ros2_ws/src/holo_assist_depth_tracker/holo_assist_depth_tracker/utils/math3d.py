from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


EPS = 1e-9


def normalize(vec: np.ndarray) -> Optional[np.ndarray]:
    norm = float(np.linalg.norm(vec))
    if norm < EPS:
        return None
    return vec / norm


def rotation_matrix_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def quaternion_from_rotation_matrix(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    m00 = float(rotation[0, 0])
    m01 = float(rotation[0, 1])
    m02 = float(rotation[0, 2])
    m10 = float(rotation[1, 0])
    m11 = float(rotation[1, 1])
    m12 = float(rotation[1, 2])
    m20 = float(rotation[2, 0])
    m21 = float(rotation[2, 1])
    m22 = float(rotation[2, 2])

    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if q_norm < EPS:
        return 0.0, 0.0, 0.0, 1.0
    return qx / q_norm, qy / q_norm, qz / q_norm, qw / q_norm
