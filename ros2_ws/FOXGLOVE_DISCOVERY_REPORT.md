# Foxglove Integration Discovery Report

Date: 2026-04-15
Workspace root: `~/git/RS2-HoloAssist`

## Discovery Scope

Goal of this pass:
- identify relevant repos/branches/packages
- map runtime interfaces (launches, nodes, topics)
- resolve overlap across dashboard/teleop/manager/unity work
- produce a Foxglove-first source-of-truth path in `john`

## Repositories Found

Top-level repos in workspace:
- `john` (active integration branch)
- `nic`
- `seb`
- `ollie`

Related nested Unity repos discovered:
- `ROS-TCP-Connector` (nested git repo)
- `URDF-Importer` (nested git repo)

No git submodules detected in `john`.

## Branch Ownership (Practical)

`origin/john` owned:
- core perception/manipulation/servo/unity bridge packages
- prior web dashboard path (`holoassist-dashboard`)

`origin/seb` owned:
- `holoassist_manager` (mode + heartbeat diagnostics)
- manager-focused runtime supervision

`origin/ollie` owned:
- `holoassist_movement`
- `ur3_keyboard_teleop`
- `ur3_joint_position_controller`

`origin/nic` owned:
- Unity-heavy XR project updates
- desktop dashboard scripts (`dashboard/`, `launch.py`, `launch.sh`, `dashboard.sh`)

## Overlaps and Fragmentation Found

Main overlap areas:
- Multiple dashboard paradigms: React (`holoassist-dashboard`), PyQt (`dashboard/`), HTTP relay (`dashboard_relay_node`), and now Foxglove.
- Runtime status previously split between terminal logs and package-specific topics.
- Perception evidence mostly exposed through custom HTTP relay rather than unified ROS observability topics.

Main fragmentation issues resolved:
- Added a single runtime aggregation node (`runtime_observability_node`) under `holoassist_foxglove`.
- Added consistent `/holoassist/*` topic namespace for diagnostics, events, state, and metrics.
- Wired `foxglove_bridge` into launch flow.

## Final Source-of-Truth Decision

Chosen source of truth:
- Base runtime in `john`
- Import/merge manager + motion/teleop from `seb` and `ollie`
- Introduce new Foxglove-focused package (`holoassist_foxglove`)
- Keep legacy dashboards as compatibility/fallback only

## Integration Commits on `john`

Key delivered commits:
- `98e132d` integrate manager and UR3 teleop packages from seb/ollie branches
- `e364f9e` add foxglove-first observability package and runtime launch flow
- `43cde86` tighten ignore rules for Unity-generated artifacts
- `6980535` sync Unity/XR source tree from `origin/nic` and clean ignore noise

## Unity Sync Outcome

`john` now includes `origin/nic` source-of-truth Unity folders:
- `Unity/My project/Assets`
- `Unity/My project/Packages`
- `Unity/My project/ProjectSettings`

Tracked build junk from `nic` was intentionally excluded via `.gitignore` cleanup, so `john` remains cleaner while retaining usable source content.

## Open Follow-Ups

- Convert any remaining terminal-only runtime signals to ROS topics.
- Add a concrete exported Foxglove `.json` layout file (current repo has panel spec YAML).
- Standardize launcher scripts that were synced from `nic` but still use absolute `nic` paths.
