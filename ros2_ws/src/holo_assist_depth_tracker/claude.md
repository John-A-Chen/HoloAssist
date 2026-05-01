# claude.md

Reviewed: 2026-04-30

Package: holo_assist_depth_tracker
Purpose: Depth-only object tracker using depth thresholding, morphology, and connected components.

## Repo Context Reviewed
The following shared docs were reviewed and are considered upstream context:
- `README.md`
- `holoassist-dashboard/README.md`
- `ros2_ws/FOXGLOVE_DISCOVERY_REPORT.md`
- `ros2_ws/FOXGLOVE_RUNTIME.md`
- `ros2_ws/HOLOASSIST_RUNTIME_UNIFICATION_RUNBOOK.md`
- `ros2_ws/PERCEPTION_PHASE4_VALIDATION.md`

## Local Markdown Reviewed
- `README.md`

## Local Doc Signals
### README.md
- # holo_assist_depth_tracker
- ## 1) System Overview
- ## 2) Architecture Diagram
- ## 3) Node Descriptions
- ### `holoassist_detection_merge_node`
- ### `holoassist_workspace_board_node`
- ### `holoassist_cube_pose_node`
- ### `holoassist_overlay_node`
- ### Existing `depth_tracker_node` (preserved)
- ## 4) Topics
- ### Detector/Merge
- ### Workspace/Robot

## Working Agreement
- Treat `/home/john/git/RS2-HoloAssist/john/claude.md` as the full master corpus.
- Keep this package file concise and package-specific.
- Preserve backward compatibility unless explicitly asked to break it.
