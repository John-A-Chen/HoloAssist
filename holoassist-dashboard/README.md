# RS2 HoloAssist Dashboard (Legacy Path)

Local web dashboard for RS2 HoloAssist.

Foxglove migration status:

- Primary runtime observability path: `ros2_ws/src/holoassist_foxglove`
- This dashboard is retained for compatibility and fallback debugging.
- New runtime telemetry should be published as ROS topics for Foxglove first.

Current default page is a tabbed System Dashboard that reads from the depth tracker dashboard relay:
- `GET /api/perception/status`
- `GET /api/perception/debug.jpg`

Tabs:
- Overview
- Perception
- Robot State
- Unity Bridge

Relay modes:
- Depth tracker mode (RealSense depth pipeline active)
- RGB fallback mode (raw webcam image only, pointcloud/obstacle marked unsupported)

The old mock robot-operation panels are still present in `src/pages/DashboardPage.tsx` and related components for later reintegration.

## Stack

- React + TypeScript + Vite
- Tailwind CSS
- Zustand (global state)
- Recharts (latency sparkline)
- Lightweight local UI primitives (shadcn-style patterns)

## Run (Perception Monitor)

Requires Node.js `18+` (Vite 5).

```bash
cd holoassist-dashboard
npm install
npm run dev
```

Open `http://localhost:5173`.

If the relay runs on the same machine, Vite proxies `/api/*` to `http://127.0.0.1:8765`.

Optional override:

```bash
VITE_PERCEPTION_API_BASE=http://<relay-host>:8765 npm run dev
```

Production build:

```bash
npm run build
npm run preview
```

## Component map

- `src/pages/DashboardPage.tsx`: page layout and panel composition
- `src/components/layout/TopBar.tsx`: connection/mode/E-stop/latency controls
- `src/components/panels/ServoPanel.tsx`: servo enable, speed scaling, deadman, zero command
- `src/components/panels/PlanningPanel.tsx`: mock plan/execute and editable target pose
- `src/components/panels/SafetyPanel.tsx`: software safety toggles and warnings
- `src/components/panels/DigitalTwinPanel.tsx`: centre visualisation placeholder + full-screen mode
- `src/components/panels/JointStateWidget.tsx`: 6-joint telemetry readout
- `src/components/panels/ToolPoseWidget.tsx`: tool pose display
- `src/components/panels/EventLogWidget.tsx`: filtered log list
- `src/components/ui/*`: reusable local UI primitives
- `src/components/visualisation/LatencySparkline.tsx`: Recharts sparkline

## State and mock backend

- `src/store/useRobotStore.ts`: global Zustand store, actions, UI behaviour rules, event logging
- `src/services/robotApi.ts`: async mock API abstraction (replace later with ROS/websocket bridge)
- `src/types/robot.ts`: payload and domain types
- `src/hooks/useDashboardPolling.ts`: 10 Hz polling loop (mock)
- `src/hooks/useKeyboardShortcuts.ts`: keyboard shortcuts (`Space`, `E`, `R`)

## Where to plug in ROS later

Replace the mock API implementations in `src/services/robotApi.ts` with your real transport layer while keeping the same function signatures where possible.

Suggested mapping:

- `pollTelemetry()` -> websocket/ROS bridge snapshot stream (`joint_states`, tool pose, latency, warnings)
- `setConnectionStatus()` -> transport connect/disconnect lifecycle
- `setRobotMode()` -> mode topic/service/action wrapper
- `setServoEnabled()` / `sendZeroCommand()` -> MoveIt Servo control endpoints
- `planTarget()` / `executePlan()` -> MoveIt planning/execution calls
- `setTargetFromClickedPointPlaceholder()` -> `/clicked_point` integration (or backend callback)

If you later move to streaming subscriptions, keep the store API unchanged and push updates into the store from a transport adapter hook.

## Notes

- UK spelling is used in UI copy where applicable (for example, "visualisation").
- The centre panel intentionally leaves a clean slot for a future 3D view (Three.js, RViz stream, or custom renderer).
