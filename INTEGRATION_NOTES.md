# Integration Notes: seb ↔ main

Reference for merging features between branches without breaking either.

## Scripts inventory

### Unique to main (Nic's, never overwrite from seb)
- `CollisionDebugVisualizer.cs` — visualizes collision normals/contacts
- `HeadsetStreamPublisher.cs` — streams headset camera frames to ROS
- `MeshCollisionGuard.cs` — runtime collision validation
- `SessionLogger.cs` — logs experiment sessions
- `SpatialMarkers.cs` — spatial anchor markers
- `UR3eKinematics.cs` — forward/inverse kinematics for UR3e

### Originated in seb (always pull seb's version on conflict)
- `BinDetector.cs`
- `BinStatusPanel.cs`
- `CoachingPanel.cs`
- `JointTFVisualizer.cs`
- `PanelPlacer.cs`
- `PassthroughToggle.cs`
- `RadialMenu.cs`
- `RobotDataPanel.cs`
- `ROSObjectPublisher.cs`
- `Editor/HoloAssistSetup.cs` — auto-setup menu item

### Modified by both branches (handle case-by-case)
These scripts originated in main (Nic) but seb has modifications too:
- `JointStateSubscriber.cs` — seb has changes for kinematic-only mode
- `RobotBasePlacer.cs` — same in both currently
- `RobotController.cs` — seb added `tfVisualizer` field, X-button binding, public `CurrentPositions`/`JointNames`/`SelectedJoint`
- `RobotHUD.cs` — same in both
- `RobotControlActions.inputactions` — same in both

When pulling seb's `RobotController.cs` into main, verify Nic's robot still works.

## Asset folders

### Unique to main
- `Assets/PortalMesh/` — Nic's portal effect

### Same in both
- `Assets/Environment_Models/` (bin FBX, Realsense FBX)
- `Assets/Materials/` (GroundMat, VRSkybox, TrolleyMat)
- `Assets/trolley model/` (159M — kept identical, do not modify)

### Different versions
- `Assets/URDF/` — main is **21M** (newer robot model + more meshes), seb is **12M** (older). **Always keep main's version** when merging.

## Scene file (SampleScene.unity)

Both branches have heavily diverged. **Never blind-merge** — always one of:
1. Use main's scene + manually add seb features via `GameObject → HoloAssist → Setup All UI Features`
2. Use seb's scene + manually replace robot/trolley with main's version

## Manifest.json

Both branches use **relative paths** for ROS-TCP-Connector and URDF-Importer:
- `file:../../../ROS-TCP-Connector/...`
- `file:../../../URDF-Importer/...`

These resolve to the repo root. Each clone needs the packages cloned/copied at the repo root level.

main also has `com.unity.device-simulator.devices` (added for editor XR testing). seb doesn't.

## Git LFS tracking

seb's `.gitattributes` tracks `*.fbx` via LFS (added when seb migrated environment models). main's `.gitattributes` only tracks `*.dae` and `*.apk`.

**When copying FBX between branches:**
1. Use `cp` of the actual file (not `git checkout`) to avoid LFS pointer corruption
2. Verify file size after copy — pointer files are ~130 bytes, real FBX files are 40K+
3. If you see 130-byte FBX after a git operation, run `git lfs pull` in the source branch and re-copy

## Known integration gotchas

1. **Meta XR Simulator 404 on Linux** — patch `Library/PackageCache/com.meta.xr.sdk.core@*/Editor/MetaXRSimulator/Installer.cs` to skip auto-install on `UNITY_EDITOR_LINUX`. Patch is per-project in Library/, must be re-applied if Library is wiped.
2. **Active Input Handler** — must be `Input System Package` only (1, not 2/Both) for Android builds. `ProjectSettings.asset` line `activeInputHandler: 1`.
3. **URDF-Importer assimp.dll** — set `Any: enabled: 0` in win/x86 and win/x86_64 `.meta` files for Android builds.
4. **Velocity controller** — UR driver starts in `scaled_joint_trajectory_controller`. Must switch to `forward_velocity_controller` after launch (see QUICKSTART.md).
5. **Quest signing keys** — different builds use different keys; `INSTALL_FAILED_UPDATE_INCOMPATIBLE` on cross-headset deploy. Click "Yes" to remove old install.

## Quick comparison commands

```bash
# Compare scripts between branches
diff -q "/home/sebastian/git/rs2/HoloAssist(Main)/Unity/My project/Assets/Scripts" \
        "/home/sebastian/git/rs2/HoloAssist/Unity/My project/Assets/Scripts"

# Check FBX is real file not LFS pointer
ls -lh "Unity/My project/Assets/Environment_Models/"
# Real FBX: 40K+ size. LFS pointer: ~130 bytes

# Diff scene files (warning: huge output)
diff "Unity/My project/Assets/Scenes/SampleScene.unity" \
     "/path/to/other/branch/SampleScene.unity" | head -50

# Check what's tracked by LFS
git lfs ls-files
```

## Recommended workflow

1. Work on your seb branch for new features.
2. When ready to integrate to main:
   - Cherry-pick script files via `git checkout origin/seb -- <file>`
   - Copy FBX/binary assets via `cp` (not git checkout) to avoid LFS pointer issues
   - Open main's scene in Unity and run `GameObject → HoloAssist → Setup All UI Features`
   - Manually verify and save scene
3. For URDF/trolley updates: always pull from main into seb, never the other way.
