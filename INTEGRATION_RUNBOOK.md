# Integration Runbook — Recreating Main from `nic` + `seb`

This runbook describes how to reproduce the current state of `main` on this machine, where Nic's teleoperation features and Seb's UI/environment features are integrated together with bug fixes applied. Follow it sequentially if you need to rebuild this state from scratch.

**End state:** A working `main` branch where:
- Nic's teleoperation system is intact (RMRC, Direct Joint, Hand Guide, gripper, dashboard)
- Seb's UI panels, environment models, and RobotDataPanel are integrated
- The `RobotDataPanel` correctly reflects live joint angles even before ROS connects
- No Unity asset files contain committed merge conflict markers
- Build artifacts (APKs) are not tracked in git
- `INTEGRATION_NOTES.md`, `SETUP.md`, `UNITY_GIT.md` document the gotchas

---

## 0. Source-of-truth references

Before starting, read these companion docs — they contain detail this runbook deliberately summarises:
- `INTEGRATION_NOTES.md` — script-level inventory of which branch each file belongs to
- `UNITY_GIT.md` — git-side problems and the conflict-marker resolution script
- `SETUP.md` — fresh-clone prerequisites and the ROS-TCP-Connector patch
- `CLAUDE.md` — system architecture and known issues

The remote is `github.com:John-A-Chen/HoloAssist` and these branches matter:

| Branch | Owner | Contains |
|---|---|---|
| `nic` | Nic | Teleoperation, kinematics, dashboard, collision guard, gripper, headset stream |
| `seb` | Seb | RobotDataPanel, BinStatusPanel, CoachingPanel, RadialMenu, JointTFVisualizer, PassthroughToggle, environment models, portal mesh |
| `john` | John | RealSense, ROS workspace, calibration, OnRobot driver |
| `ollie` | Oliver | MoveIt 2 autonomous sorting (planned) |
| `main` | shared | Integrated final state |

> **Important:** the current `main` is a **superset** of both `nic` and `seb` — it isn't equivalent to either branch. It contains all of nic's teleoperation runtime, all of seb's UI and environment work, **plus** modifications authored during integration that aren't in either source branch. Section 1 below enumerates what's new, what was modified, and where the deltas live. Re-doing the merge alone is not enough — Section 6 (RobotDataPanel/JointStateSubscriber fix) is essential and is *not* present in either source branch.

---

## 1. Feature delta — what main now has beyond either source branch

The integration produced more than the union of `nic + seb`. Three categories matter:

1. **Net-new features brought across** from each branch into main
2. **In-place modifications** authored during integration (not in either source branch)
3. **Removals / replacements** (older versions superseded)

### 1a. Net-new from `nic` (absent in seb)

**Teleoperation runtime**
- `UR3eKinematics.cs` — forward kinematics, geometric Jacobian, DLS pseudoinverse, manipulability measure
- `RobotController.cs` — three control modes (RMRC, Direct Joint, Hand Guide), translate/rotate sub-modes, gripper control, EE lock-down, output velocity smoothing
- `MeshCollisionGuard.cs` — table + self-collision protection via mesh proximity
- `CollisionDebugVisualizer.cs` — on-screen collision status label
- `RobotControlActions.inputactions` — Unity Input System bindings (replaces OVRInput, which doesn't work in the MR Template)

**HUD & spatial visualisation**
- `RobotHUD.cs` — colour-coded floating mode panel, joint names in Direct Joint mode, gripper bar, EE-lock indicator
- `SpatialMarkers.cs` — RGB axes on `tool0` + yellow velocity arrow

**Connectivity & telemetry**
- `ROSAutoConnect.cs` — auto-discovers `ros_tcp_endpoint` by TCP-scanning `192.168.0.101–109` (no beacon/multicast — works on Quest)
- `HeadsetStreamPublisher.cs` — captures Unity scene from XR camera, JPEG-encodes, publishes `/headset/image_compressed` to the dashboard at 15 FPS
- `SessionLogger.cs` — publishes session JSON to `/session/status` (2 Hz) + `/session/events`, saves session log on quit

**Dashboard (Python/PyQt5)**
- `dashboard/main.py`, `dashboard/ros_interface.py` — e-stop console with STATUS / HEADSET / CAMERA / STATS / LATENCY / SESSION tabs
- `launch.py`, `launch.sh`, `dashboard.sh` — one-command launchers (fake-hardware default, `--robot-ip` for real robot)
- `beacon.py` — UDP multicast IP beacon (legacy fallback)

**Robot model**
- `Assets/URDF/ur3e_rg2.urdf` — UR3e + OnRobot RG2 combined URDF with the 5 mimic joints
- `Assets/URDF/onrobot_rg2/` — RG2 STL meshes (visual renamed `*_visual.stl` to dodge URDF-Importer name-collision bug)
- `Assets/URDF/ur_description/meshes/ur3e/` — UR3e visual (DAE) + collision (STL) meshes (21 MB total)

### 1b. Net-new from `seb` (absent in nic)

These come across via the seb merge in Section 3b:

**UI panels**
- `RobotDataPanel.cs` — XR floating panel with joint angles, EE pose, control mode, gripper %, EE lock, passthrough state, TF-toggle button
- `BinStatusPanel.cs` — bin sorting status
- `CoachingPanel.cs` — coaching/instruction overlay
- `RadialMenu.cs` — radial selection menu
- `JointTFVisualizer.cs` — toggleable per-joint TF axes
- `PassthroughToggle.cs` — MR/VR switching
- `PanelPlacer.cs` — panel positioning helper

**Object detection & publishing**
- `BinDetector.cs` — bin classification
- `ROSObjectPublisher.cs` — publishes interactive cube TF + pose markers

**Editor tooling**
- `Editor/HoloAssistSetup.cs` — `GameObject → HoloAssist → Setup All UI Features` menu item

**Environment**
- `Assets/PortalMesh/` — portal test chamber (geometry, materials, textures)
- `Assets/Environment_Models/` — bin FBX, RealSense FBX (LFS-tracked — see `INTEGRATION_NOTES.md` "Git LFS tracking")

### 1c. Modifications authored during integration (NOT in either source branch)

The biggest reason main is a superset rather than just a union: these script changes were written *during* the integration to make the combined system work. None exist in `nic` or `seb` as-is.

| File | Change | Reason |
|---|---|---|
| `JointStateSubscriber.cs` | Added public `LastJointAnglesRad[]` cache + `HasReceivedJointState` flag + `rosNameToIndex` lookup; populated inside `OnJointState` after the angle clamp | Make joint state a single source of truth so other UI doesn't each subscribe and race `ROSConnection.Start()` |
| `RobotDataPanel.cs` | (1) Removed premature `SubscribeToJointStates()` from `OnEnable()`; (2) added auto-find for `JointStateSubscriber`, `RobotController`, `tool0`, `base_link` in `Start()`; (3) `UpdateData()` now reads `jointStateSubscriber.LastJointAnglesRad` rather than its own callback; (4) `LateUpdate()` self-heals if not subscribed | Panel was showing all-zero angles because `OnEnable` ran before `ROSConnection.Start()` registered subscribers; reading from `JointStateSubscriber` removes the race entirely |
| `RobotHUD.cs` | `LateUpdate` self-heal subscribe pattern (`if (!hudRosSubscribed) SubscribeJointStates();`) | Same lifecycle race; survives "Disable Domain Reload" Project Setting |
| `RobotController.cs` (main side) | Added `tfVisualizer` field, X-button RMRC sub-mode binding, public `CurrentPositions` / `JointNames` / `SelectedJoint` getters | Lets `RobotDataPanel`, `JointTFVisualizer`, and HUD read controller state without `FindObjectOfType` every frame |
| `.gitignore` | Added `Library/`, `Logs/`, `*.sln`, `*.csproj`, `Assets/_Recovery/`, `Unity/My project/.vscode/`, root-level stray Unity dirs (`/Library/`, `/Logs/`, `/Packages/`, `/ProjectSettings/`, `/UserSettings/`), generated `.slnx` | Stop tracking Unity build artefacts and IDE caches |
| `SETUP.md` | Added "Required patch: ROS-TCP-Connector reconnect fix" section | Package is gitignored, patch must be re-applied per fresh clone (otherwise topics stop arriving after a ROS restart) |
| Conflict markers in 21 Unity asset files | Resolved per the table in Section 5b | Previous merges committed `<<<<<<<` markers; Unity refuses to import while they exist |
| `xrtest.apk` | Untracked via `git rm --cached` | Build artefact tracked by accident; gitignore alone doesn't untrack files already in the index |

### 1d. UI feature comparison table

| Feature | `seb` branch | `nic` branch | `main` (current) |
|---|---|---|---|
| Floating data panel | ✅ basic | ❌ | ✅ + auto-find references + self-heal + live `LastJointAnglesRad` |
| Mode HUD | ❌ | ✅ | ✅ + EE-lock indicator + gripper bar + self-heal |
| Bin status panel | ✅ | ❌ | ✅ |
| Coaching overlay | ✅ | ❌ | ✅ |
| Radial menu | ✅ | ❌ | ✅ |
| TF axes toggle | ✅ | ❌ | ✅ wired through `RobotDataPanel` button |
| Passthrough toggle | ✅ | ❌ | ✅ + state shown in `RobotDataPanel` |
| Spatial axes on tool0 | ❌ | ✅ | ✅ |
| Velocity arrow viz | ❌ | ✅ | ✅ |
| Headset stream to dashboard | ❌ | ✅ | ✅ |
| Session-logging publisher | ❌ | ✅ | ✅ |
| Dashboard (PyQt5 e-stop) | ❌ | ✅ | ✅ |
| Gripper control (XR trigger) | ❌ | ✅ | ✅ |
| Three teleop modes | ❌ | ✅ | ✅ |
| EE lock-down (Y button) | ❌ | ✅ | ✅ + state indicator |
| Output velocity smoothing | ❌ | ✅ | ✅ |
| Auto-discover ROS endpoint | ❌ | ✅ | ✅ |
| OnRobot RG2 in URDF + animated mimic joints | ❌ | ✅ | ✅ |
| Environment / portal mesh / bins | ✅ | ❌ | ✅ |
| Interactive cube + pose markers | ✅ | ❌ | ✅ |

If a feature in this table doesn't behave as described after a fresh integration, the most likely cause is that Section 6 (RobotDataPanel/JointStateSubscriber fix) hasn't been applied yet — the panel renders but stays blank, or the HUD freezes after a ROS restart.

---

## 2. Prerequisites

```bash
# Clone the repo via SSH (HTTPS clone fails on this machine)
git clone git@github.com:John-A-Chen/HoloAssist.git HoloAssist_Main
cd HoloAssist_Main

# Clone the two Unity packages that are gitignored
git clone git@github.com:Unity-Technologies/ROS-TCP-Connector.git
git clone git@github.com:Unity-Technologies/URDF-Importer.git

# Apply the ROS-TCP-Connector reconnect patch
# See SETUP.md "Required patch: ROS-TCP-Connector reconnect fix"
# Edits ROS-TCP-Connector/.../Runtime/TcpConnector/RosTopicState.cs
# Removes the !SentSubscriberRegistration guard in OnConnectionEstablished

# Build the ROS workspace
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
cd ..
```

---

## 3. Establish the integration baseline

The integration sequence is `nic → main`, then `seb → main`. Always merge **into main**, never the reverse — main is the integration target.

```bash
# Start from a clean main
git checkout main
git pull origin main

# Step 3a: bring Nic's teleoperation work into main
git merge origin/nic
# Conflicts here are usually in:
#   - Unity/My project/Packages/manifest.json (keep main, then re-add anything nic-only)
#   - Assets/Scripts/RobotController.cs (take nic's version — it's the canonical one)
#   - Assets/Scenes/SampleScene.unity (see Section 5)
git commit  # accept the merge
```

**Key files Nic owns** (always take nic's version when in conflict — listed in `INTEGRATION_NOTES.md` "Unique to main"):

```
Assets/Scripts/CollisionDebugVisualizer.cs
Assets/Scripts/HeadsetStreamPublisher.cs
Assets/Scripts/MeshCollisionGuard.cs
Assets/Scripts/SessionLogger.cs
Assets/Scripts/SpatialMarkers.cs
Assets/Scripts/UR3eKinematics.cs
Assets/Scripts/RobotController.cs
Assets/Scripts/JointStateSubscriber.cs   (modified later — see Section 5)
Assets/Scripts/RobotHUD.cs
Assets/Scripts/RobotBasePlacer.cs
Assets/Scripts/ROSAutoConnect.cs
```

```bash
# Step 3b: bring Seb's UI and environment work into main
git merge origin/seb
# Expect heavy conflicts here — see Section 5
```

**Key files Seb owns** (always take seb's version when in conflict):

```
Assets/Scripts/BinDetector.cs
Assets/Scripts/BinStatusPanel.cs
Assets/Scripts/CoachingPanel.cs
Assets/Scripts/JointTFVisualizer.cs
Assets/Scripts/PanelPlacer.cs
Assets/Scripts/PassthroughToggle.cs
Assets/Scripts/RadialMenu.cs
Assets/Scripts/RobotDataPanel.cs        (modified later — see Section 5)
Assets/Scripts/ROSObjectPublisher.cs
Assets/Scripts/Editor/HoloAssistSetup.cs
Assets/PortalMesh/                       (entire folder)
Assets/Environment_Models/               (entire folder)
```

---

## 4. Manifest and project settings

```bash
# Packages/manifest.json must keep these entries from main:
#   "com.unity.device-simulator.devices": "..."   (added in main, missing in seb)
# And the relative file:// paths to ROS-TCP-Connector and URDF-Importer.

# ProjectSettings/ProjectSettings.asset:
#   activeInputHandler: 1   (Input System Package only — required for Android)
#   Take main's version on conflict.
```

If `URDF-Importer/.../assimp.dll.meta` files have been touched, set `Any: enabled: 0` in both `win/x86` and `win/x86_64` meta files (Android build requirement — see `INTEGRATION_NOTES.md` gotcha 3).

---

## 5. Unity asset merge conflicts

This is the longest step. Unity YAML files (`*.mat`, `*.unity`, `*.prefab`, `*.asset`, `*.meta`) regularly produce nonsensical conflicts because two unrelated assets share the internal fileID `&2100000`. See `UNITY_GIT.md` for the root cause.

### 5a. Detect committed conflict markers

```bash
grep -rl "<<<<<<" "Unity/My project/Assets/"
```

If this returns any files, the previous merge committed unresolved markers. They must be cleaned up before Unity can open the project.

### 5b. Resolution rules (which side to keep)

| File path pattern | Keep |
|---|---|
| `Assets/Materials/*.mat` and `.meta` | **ours** (HEAD) — these are Nic's grey/blue/skybox materials |
| `Assets/PortalMesh/.../source/materials/*.mat` | **theirs** — environment branch's material content |
| `Assets/PortalMesh/*.mat.meta` and `Assets/PortalMesh/.../Materials/*.mat.meta` | **theirs** — match the environment material GUIDs |
| `Assets/Samples/XR Hands/.../Materials/*.mat` | **ours** — Unity package, never accept external changes |
| `Assets/MRTemplateAssets/.../*.mat` | **ours** |
| `Assets/Editor.meta` | **theirs** |
| `Assets/Environment_Models.meta` | **ours** |
| `Assets/XR/.../*.asset.meta` | **ours** |
| `Assets/Scenes/SampleScene.unity` | **ours** (the scene-owner's stable state — never accept the marker-laden version) |

### 5c. Apply with the resolver script

Save this as `/tmp/resolve_conflicts.py`:

```python
#!/usr/bin/env python3
import re, os

def resolve(content, keep='ours'):
    out, state = [], 'normal'
    for line in content.split('\n'):
        if re.match(r'^<{7}', line):                         state = 'ours'
        elif re.match(r'^={7}', line) and state == 'ours':   state = 'theirs'
        elif re.match(r'^>{7}', line) and state == 'theirs': state = 'normal'
        elif state == 'normal':                              out.append(line)
        elif state == 'ours'   and keep == 'ours':           out.append(line)
        elif state == 'theirs' and keep == 'theirs':         out.append(line)
    return '\n'.join(out)

base = "Unity/My project"
files = [
    ("Assets/Materials/TrolleyMat.mat",                                                   "ours"),
    ("Assets/Materials/TrolleyMat.mat.meta",                                              "ours"),
    ("Assets/Materials/VRSkybox.mat",                                                     "ours"),
    ("Assets/Materials/VRSkybox.mat.meta",                                                "ours"),
    ("Assets/Materials/GroundMat.mat",                                                    "ours"),
    ("Assets/Materials/GroundMat.mat.meta",                                               "ours"),
    ("Assets/MRTemplateAssets/Materials/AR/ZMaterial.mat",                                "ours"),
    ("Assets/PortalMesh/test-chamber-00-01-v2/source/materials/DEV_REFLECTIVITY_40.mat",  "theirs"),
    ("Assets/PortalMesh/test-chamber-00-01-v2/source/materials/SIGNAGE_SIGNAGE_BORDER.mat","theirs"),
    ("Assets/PortalMesh/test-chamber-00-01-v2/source/materials/METAL_METALWALL060A.mat",  "theirs"),
    ("Assets/Samples/XR Hands/1.7.3/HandVisualizer/Materials/HighFidelity.mat",           "ours"),
    ("Assets/Samples/XR Hands/1.7.3/HandVisualizer/Materials/TransparentRed.mat",         "ours"),
    ("Assets/Environment_Models.meta",                                                    "ours"),
    ("Assets/Editor.meta",                                                                "theirs"),
    ("Assets/PortalMesh/poster.mat.meta",                                                 "theirs"),
    ("Assets/PortalMesh/postermat.mat.meta",                                              "theirs"),
    ("Assets/PortalMesh/test-chamber-00-01-v2/Materials/Bed01_normal.mat.meta",           "theirs"),
    ("Assets/PortalMesh/test-chamber-00-01-v2/Materials/Round_elevator_sheet_1.mat.meta", "theirs"),
    ("Assets/XR/UserSimulationSettings/Resources/XRSimulationPreferences.asset.meta",     "ours"),
    ("Assets/XR/Resources/XRSimulationRuntimeSettings.asset.meta",                        "ours"),
    ("Assets/Scenes/SampleScene.unity",                                                   "ours"),
]
for rel, keep in files:
    path = os.path.join(base, rel)
    content = open(path, encoding='utf-8').read()
    if '<<<<<<' not in content:
        print(f"skip  {rel}"); continue
    open(path, 'w', encoding='utf-8').write(resolve(content, keep))
    print(f"fixed [{keep}] {rel}")
```

```bash
python3 /tmp/resolve_conflicts.py
grep -rl "<<<<<<" "Unity/My project/Assets/"   # must return nothing
```

### 5d. Verify the resolution

After resolving, sanity-check that material names match their filenames:

```bash
cd "Unity/My project"
for f in \
  Assets/Materials/TrolleyMat.mat \
  Assets/Materials/VRSkybox.mat \
  Assets/Materials/GroundMat.mat \
  Assets/PortalMesh/test-chamber-00-01-v2/source/materials/DEV_REFLECTIVITY_40.mat \
  Assets/PortalMesh/test-chamber-00-01-v2/source/materials/SIGNAGE_SIGNAGE_BORDER.mat
do
  echo "$f: $(grep -m1 'm_Name:' "$f" | grep -v '^  m_Name: $')"
done
# Each line should show the material name matching the filename.
```

---

## 6. Apply the RobotDataPanel / JointStateSubscriber fix

After integration, the `RobotDataPanel` showed all-zero joint angles even when ROS was connected. Root cause: `RobotDataPanel.OnEnable()` subscribed to `/joint_states` before `ROSConnection.Start()` had initialised, so the subscription registered with a null message name and never received callbacks.

### 6a. `JointStateSubscriber.cs` — add a public cache

In `Unity/My project/Assets/Scripts/JointStateSubscriber.cs`:

1. Add public state at the top of the class:
   ```csharp
   public float[] LastJointAnglesRad { get; private set; } = new float[6];
   public bool HasReceivedJointState { get; private set; } = false;

   private static readonly Dictionary<string, int> rosNameToIndex = new Dictionary<string, int>
   {
       { "shoulder_pan_joint",  0 },
       { "shoulder_lift_joint", 1 },
       { "elbow_joint",         2 },
       { "wrist_1_joint",       3 },
       { "wrist_2_joint",       4 },
       { "wrist_3_joint",       5 },
   };
   ```

2. Inside `OnJointState`, after the existing `Mathf.Clamp` block and before applying the rotation, write the cache:
   ```csharp
   if (rosNameToIndex.TryGetValue(rosName, out int idx))
   {
       LastJointAnglesRad[idx] = angle;
       HasReceivedJointState = true;
   }
   ```

### 6b. `RobotDataPanel.cs` — read from the cache and self-heal

In `Unity/My project/Assets/Scripts/RobotDataPanel.cs`:

1. Add Inspector field:
   ```csharp
   [Tooltip("Auto-found at runtime if left empty")]
   public JointStateSubscriber jointStateSubscriber;
   ```

2. **Remove** the `SubscribeToJointStates()` call from `OnEnable()` (it runs before `ROSConnection.Start()` and silently fails).

3. Augment `Start()` so it auto-finds references and subscribes after Unity's lifecycle has settled:
   ```csharp
   void Start()
   {
       if (Application.isPlaying)
       {
           if (jointStateSubscriber == null)
               jointStateSubscriber = FindObjectOfType<JointStateSubscriber>();
           if (robotController == null)
               robotController = FindObjectOfType<RobotController>();
           if (endEffectorTransform == null)
           {
               var t = GameObject.Find("tool0");
               if (t != null) endEffectorTransform = t.transform;
           }
           if (robotBase == null)
           {
               var b = GameObject.Find("base_link");
               if (b != null) robotBase = b.transform;
           }
       }
       Rebuild();
       if (Application.isPlaying)
           SubscribeToJointStates();
   }
   ```

4. In `LateUpdate()`, add a self-healing subscribe (covers the "Disable Domain Reload" case where `Start` doesn't re-run on Play):
   ```csharp
   if (!rosSubscribed) SubscribeToJointStates();
   ```

5. In `UpdateData()`, replace the joint-angle source so the panel reads from `JointStateSubscriber` instead of its own callback:
   ```csharp
   if (jointStateSubscriber != null)
   {
       var rad = jointStateSubscriber.LastJointAnglesRad;
       for (int i = 0; i < jointAngles.Length && i < rad.Length; i++)
           jointAngles[i] = rad[i] * Mathf.Rad2Deg;
       connectionStatus = jointStateSubscriber.HasReceivedJointState
           ? "Connected" : "Waiting for ROS";
   }
   else if (rosSubscribed)
   {
       connectionStatus = "Connected";
   }
   else
   {
       connectionStatus = "Disconnected";
   }
   ```

### 6c. `RobotHUD.cs` — same self-heal pattern

In `Unity/My project/Assets/Scripts/RobotHUD.cs`, in `LateUpdate()`:

```csharp
if (!hudRosSubscribed) SubscribeJointStates();
```

This guards against the same lifecycle race.

---

## 7. Repository hygiene

### 7a. Stop tracking build artefacts

```bash
# APKs were tracked once and gitignore alone won't clear them
git rm --cached "Unity/My project/xrtest.apk" 2>/dev/null || true

# Anything else built locally that shouldn't be tracked
git rm --cached -r "Unity/My project/Library" 2>/dev/null || true
git rm --cached -r "Unity/My project/Temp"    2>/dev/null || true
```

### 7b. `.gitignore` additions

Confirm these patterns exist (the current `.gitignore` already has them, but if you're rebuilding from an older state, add the missing ones):

```
Unity/My project/*.apk
Unity/My project/Library/
Unity/My project/Temp/
Unity/My project/Logs/
Unity/My project/obj/
Unity/My project/Builds/
Unity/My project/UserSettings/
Unity/My project/.utmp/
Unity/My project/*.sln
Unity/My project/*.csproj
Unity/My project/Assets/_Recovery/
Unity/My project/.vscode/
/Library/
/Logs/
/Packages/
/ProjectSettings/
/UserSettings/
/HoloAssist_Main.sln
/Unity/Unity.sln
/Unity/My project/My project.slnx
```

### 7c. Document the ROS-TCP-Connector patch in SETUP.md

The patch in `ROS-TCP-Connector/com.unity.robotics.ros-tcp-connector/Runtime/TcpConnector/RosTopicState.cs` lives in a gitignored package, so it isn't tracked. Add a "Required patch" section to `SETUP.md` describing the change so a fresh contributor knows to re-apply it after cloning the package.

---

## 8. Commit and push

```bash
git add -A
git commit -m "Integrated nic and seb unity features. Generally stable with minimal issues. Added environment features."

# If the remote has diverged work that should be discarded (e.g. a prior buggy main):
git push origin main

# If rejected as non-fast-forward and team has agreed to overwrite:
git push --force origin main
```

Subsequent commits made during the integration:
- `Stop tracking xrtest.apk build artifact` — the `git rm --cached` step
- `Added fix for RobotDataPanel (Unity XR Interation)` — the Section 6 changes (this one introduced *new* `SampleScene.unity` conflict markers because the merge was committed without resolving them — re-run Section 5 if you see them)

---

## 9. Verification

Open Unity. The project should import without errors. Then:

| Check | Pass criteria |
|---|---|
| Unity Console clean | No "merge conflict" errors, no `NullReferenceException` from `MaterialPostprocessor` |
| Asset names | `Assets/Materials/TrolleyMat.mat` shows `m_Name: TrolleyMat`, etc. |
| `SampleScene.unity` opens | Scene hierarchy loads, no warnings about `Invalid Script reference` |
| Press Play (no ROS) | RobotDataPanel renders, status reads "Waiting for ROS", joint angles show 0.0° |
| Run `./launch.sh` (fake hardware) | RobotDataPanel status switches to "Connected", joint angles update |
| Robot moves in Unity | Right stick in RMRC mode drives the digital twin |
| Gripper | Right index trigger animates the RG2 gripper |
| HUD | Floating mode panel and joint name appear; cycle modes via Menu button |

If any check fails, the most likely cause is in Section 5 (unresolved YAML conflict) or Section 6 (lifecycle race) — re-read those sections.

---

## 10. Roll-back

If the integration breaks something irrecoverable:

```bash
# The pre-integration main is at commit 6263945 (before 9036c28 force push)
git reset --hard 6263945

# Or recover the integrated state from before the latest John-driven ROS work:
git reset --hard 9036c28
```

Always work in a fresh local branch before force-pushing back to `main`.

---

## Appendix — what each major commit did

| Commit | Effect |
|---|---|
| `035e41a` | First import of seb's UI scripts and environment models |
| `b70013f` | Wired seb's panels, radial menu, bins into main scene |
| `2c3485e` | "set environment" — introduced the cross-file YAML conflict markers (Section 5 cleans this up) |
| `ee721e2` | Restored portal materials lost during the bad merge |
| `1dc0496` / `ec5cc33` | Brought nic's teleop work into main |
| `9036c28` | Force-pushed integrated state — the canonical "integration done" commit |
| `26b5b7e` | Untracked `xrtest.apk` |
| `ab7f433` | RobotDataPanel fix (Section 6) — re-introduced `SampleScene.unity` markers that need re-resolving |
| `9426957` and after | John's ROS-side work (calibration, OnRobot driver, launch args) — independent of Unity changes |
