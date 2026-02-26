# RS2 HoloAssist Dashboard (Mock UI)

Local web dashboard scaffold for RS2 HoloAssist robot operations.

This build is UI-only and uses a mock API/state layer. No ROS pub/sub, websocket, or DDS integration is implemented yet.

## Stack

- React + TypeScript + Vite
- Tailwind CSS
- Zustand (global state)
- Recharts (latency sparkline)
- Lightweight local UI primitives (shadcn-style patterns)

## Run

Requires Node.js `18+` (Vite 5).

```bash
cd holoassist-dashboard
npm install
npm run dev
```

Open `http://localhost:5173`.

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
