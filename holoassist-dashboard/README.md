# RS2 HoloAssist Dashboard (Legacy Compatibility Path)

This React dashboard is retained for compatibility, fallback debugging, and UI prototyping.

Primary runtime observability is now Foxglove-first.

## Runtime Positioning

Current preferred path:
- `ros2_ws/src/holoassist_foxglove`
- `foxglove_bridge` for live transport

Legacy/fallback path (this dashboard):
- Pulls perception and robot status from the dashboard relay HTTP API.
- Useful when Foxglove is unavailable or for quick local checks.

## Supported Relay Endpoints

Expected relay API routes:
- `GET /api/perception/status`
- `GET /api/perception/debug.jpg`

Relay launch options live in:
- `ros2_ws/src/holo_assist_depth_tracker/launch/dashboard_bridge.launch.py`
- `ros2_ws/src/holo_assist_depth_tracker/launch/foxglove_relay.launch.py`

## How It Relates to Foxglove

This dashboard is not the source of truth for runtime observability anymore.

When adding new runtime telemetry:
1. Publish ROS topics first.
2. Wire Foxglove panels to those topics.
3. Optionally expose reduced HTTP summaries for this dashboard.

## Run (Local)

Requires Node.js 18+.

```bash
cd ~/git/RS2-HoloAssist/john/holoassist-dashboard
npm install
npm run dev
```

Open `http://localhost:5173`.

If relay runs on same machine, Vite proxies `/api/*` to `http://127.0.0.1:8765`.

Optional override:

```bash
VITE_PERCEPTION_API_BASE=http://<relay-host>:8765 npm run dev
```

Build preview:

```bash
npm run build
npm run preview
```

## Foxglove-Equivalent Topics (Reference)

Most dashboard widgets map to topics already available in Foxglove runtime:
- `/holo_assist_depth_tracker/debug_image`
- `/holo_assist_depth_tracker/bbox`
- `/holo_assist_depth_tracker/pointcloud`
- `/holo_assist_depth_tracker/obstacle_marker`
- `/joint_states`
- `/servo_target_pose`
- `/servo_node/delta_twist_cmds`
- `/holoassist/diagnostics`
- `/holoassist/events`

## Recommendation

Use this dashboard as fallback only.

For operations, demos, validation, and recordings, use Foxglove Studio connected through `foxglove_bridge`.
