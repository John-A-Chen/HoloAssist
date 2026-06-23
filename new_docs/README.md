# HoloAssist Website Update Pack

This folder is a documentation staging area for updating `/home/john/git/RS2-HoloAssist/site/docs`.
It was written from the current `main` checkout, not from the existing website text.

Use these files as source material:

| File | Purpose |
|---|---|
| `current-architecture.md` | Current subsystem relationships and runtime data flow. |
| `package-submodule-map.md` | First-party packages, upstream submodules, Unity packages, and apt dependencies. |
| `website-page-mapping.md` | Which current repo details belong on each website page. |
| `deprecated-and-wrong-details.md` | Exact stale or wrong website references with current replacements. |
| `run-guide-rewrite.md` | Updated run guide draft matching `scripts/launch.py` and current package names. |

Most important website corrections:

- Replace old ROS package names:
  - `holo_assist_depth_tracker` -> `holoassist_perception`
  - `holo_assist_depth_tracker_sim` -> folded into current perception/movement flow or absent
  - `holo_assist_depth_tracker_sim_interfaces` -> `holoassist_perception/srv/PickCubeToBin`
  - `moveit_robot_control` and `moveit_robot_control_msgs` -> `holoassist_movement`
- Update launch defaults:
  - `./scripts/launch.sh` now defaults to perception, MoveIt, and dashboard enabled.
  - Use `--no-perception`, `--no-moveit`, and `--no-dashboard` to opt out.
- Update current MoveIt topics:
  - `/moveit_robot_control/...` is stale.
  - Current topics are under `/holoassist/movement/...`.
- Update submodule status:
  - `.gitmodules` already contains the current submodules.
  - `Universal_Robots_ROS2_Driver` is not a direct submodule in this checkout.
- Keep a warning about the current perception cube-pose gap:
  - Current launch starts `apriltag_ros` and `aprilcube_tracker_node`.
  - No installed first-party executable currently publishes `/holoassist/perception/april_cube_N_pose`.
  - `aprilcube_tracker_node`, `pick_place_service_node`, and Unity expect those pose topics.

