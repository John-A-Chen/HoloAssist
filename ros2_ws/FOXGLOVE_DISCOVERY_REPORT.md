# Foxglove Integration Discovery Report

Date: 2026-04-15
Workspace root: `~/git/RS2-HoloAssist`

## Repositories Found

- `john` (active integration workspace, branch `john`)
- `nic` (branch `nic`)
- `ollie` (branch `main` locally, contains `origin/ollie`)
- `seb` (branch `seb`)
- Nested Unity repos:
  - `ROS-TCP-Connector`
  - `URDF-Importer`

No git submodules were detected.

## Relevant Branch Ownership (Source Review)

- `origin/john`
  - Core ROS packages:
    - `holo_assist_depth_tracker`
    - `holoassist_manipulation`
    - `holoassist_servo_tools`
    - `holoassist_unity_bridge`
  - React web dashboard (`holoassist-dashboard`) and depth-tracker relay path.

- `origin/nic`
  - Desktop dashboard app (`dashboard/main.py`, `dashboard/ros_interface.py`)
  - Steam Deck oriented launcher scripts (`launch.py`, `dashboard.sh`, `launch.sh`)
  - Unity-heavy assets and setup documents.

- `origin/ollie`
  - Movement and teleop packages:
    - `holoassist_movement`
    - `ur3_keyboard_teleop`
    - `ur3_joint_position_controller`
  - Additional manipulation/servo updates.

- `origin/seb`
  - `holoassist_manager` package
  - `tf_marker_bridge` and manager diagnostics/mode services
  - Unity marker/TF integration path.

## Runtime Interface Gaps Identified

- Multiple subsystems only logged status to terminal, not ROS topics.
- Diagnostics existed in `holoassist_manager`, but broader runtime telemetry was fragmented.
- No single Foxglove-centric launch path existed across perception/teleop/safety/planner/runtime state.
- Legacy dashboard naming (`holoassist-dashboard`) was not aligned with Foxglove-first direction.

## Overlaps and Duplicates

- `holoassist_manipulation`, `holoassist_servo_tools`, `holoassist_unity_bridge` appear across `john` and `ollie`.
- Dashboard functionality duplicated between:
  - `holoassist-dashboard` (React)
  - `dashboard` (PyQt + ROS interface)
  - `holo_assist_depth_tracker/dashboard_relay_node.py` (HTTP relay bridge)
- Manager/heartbeat logic existed only on `seb`.

## Source-of-Truth Decision

Chosen baseline: `origin/john` ROS stack, extended with selected branch assets:

- Integrate `holoassist_manager` from `seb`
- Integrate teleop/movement packages from `ollie`
- Add new Foxglove-first package `holoassist_foxglove` for unified observability
