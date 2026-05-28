from __future__ import annotations

import math

import numpy as np
from apriltag_msgs.msg import AprilTagDetection

from holo_assist_depth_tracker.nodes.detection_merge_node import (
    CachedDetection,
    merge_detections_by_id,
    prune_stale_detections,
)
from holo_assist_depth_tracker.nodes.workspace_board_node import (
    WorkspaceBoardNode,
    solve_workspace_pose,
)


def _det(tag_id: int) -> AprilTagDetection:
    msg = AprilTagDetection()
    msg.id = int(tag_id)
    return msg


def test_merge_keeps_latest_per_id() -> None:
    cache: dict[int, CachedDetection] = {}

    merge_detections_by_id(cache, [_det(10), _det(11)], stamp_ns=100, frame_id="cam")
    merge_detections_by_id(cache, [_det(10)], stamp_ns=50, frame_id="cam")

    assert sorted(cache.keys()) == [10, 11]
    assert cache[10].stamp_ns == 100

    merge_detections_by_id(cache, [_det(10)], stamp_ns=200, frame_id="cam")
    assert cache[10].stamp_ns == 200


def test_prune_stale_detections() -> None:
    cache = {
        1: CachedDetection(stamp_ns=1_000_000_000, frame_id="cam", detection=_det(1)),
        2: CachedDetection(stamp_ns=3_000_000_000, frame_id="cam", detection=_det(2)),
    }
    prune_stale_detections(cache, now_ns=4_500_000_000, stale_timeout_s=2.0)
    assert sorted(cache.keys()) == [2]


def test_workspace_solver_recovers_transform() -> None:
    layout = WorkspaceBoardNode.TAG_LAYOUT_W
    dists = WorkspaceBoardNode.EXPECTED_CENTER_DISTS

    yaw = math.radians(20.0)
    pitch = math.radians(-10.0)
    roll = math.radians(5.0)

    rx = np.array(
        [[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]],
        dtype=np.float64,
    )
    ry = np.array(
        [[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]],
        dtype=np.float64,
    )
    rz = np.array(
        [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]],
        dtype=np.float64,
    )
    rot = rz @ ry @ rx
    origin = np.array([0.8, -0.2, 0.5], dtype=np.float64)

    observed = {}
    for tag_id, point_w in layout.items():
        observed[tag_id] = rot @ point_w + origin

    solved = solve_workspace_pose(
        observed_points=observed,
        model_points=layout,
        expected_dists=dists,
        rms_tolerance_m=1e-6,
        dimension_tolerance_m=1e-6,
    )

    assert solved is not None
    solved_origin, solved_rot, rms, dim_err = solved
    assert rms < 1e-7
    assert dim_err < 1e-7
    assert np.allclose(solved_origin, origin, atol=1e-6)
    assert np.allclose(solved_rot, rot, atol=1e-6)
