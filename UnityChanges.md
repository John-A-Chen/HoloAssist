# Unity Editor Changes Required for Integration

After pulling these code changes, the following must be done **inside the Unity Editor** to wire everything up.

---

## 1. Add CubePoseSubscriber

This script subscribes to perception cube poses (relayed to `base_link` frame) and spawns/moves virtual objects in the scene.

1. **Create an empty GameObject**: `GameObject > Create Empty`, name it `CubePoseManager`
2. **Attach script**: drag `Assets/Scripts/CubePoseSubscriber.cs` onto `CubePoseManager`
3. **Configure in Inspector**:
   - **Robot Base**: drag the robot's `base_link` GameObject (inside `ur3e_rg2` hierarchy) into the `Robot Base` field
   - **Cube Count**: `4` (default)
   - **Topic Prefix**: `/holoassist/unity` (default — matches the cube_pose_relay node)
   - **Cube Prefabs** (optional): drag up to 4 prefabs for the virtual objects (e.g. bomb, gem, etc. for the gamified mapping). Leave empty for auto-generated coloured cubes.
   - **Default Prefab** (optional): a single fallback prefab if you don't want individual ones
   - **Object Scale**: `0.04` (4 cm cubes, matching the physical AprilTag cubes)
   - **Pose Timeout**: `3.0` seconds
   - **Smoothing**: `0.5` (adjust for feel)

4. **For bin verification**: ensure the virtual cube prefabs (or the auto-generated cubes) have **Colliders** and **Rigidbody** (set to `Is Kinematic = true`). The `BinDetector.cs` on each bin uses trigger colliders to detect objects entering the bin.

---

## 2. Add OperatingModeController

This script lets the Quest 3 operator switch between TELEOP and MOVEIT modes, syncing with the dashboard.

1. **Create an empty GameObject**: `GameObject > Create Empty`, name it `OperatingModeManager`
   - Or attach to the same GameObject as `CubePoseManager` — either works
2. **Attach script**: drag `Assets/Scripts/OperatingModeController.cs` onto it
3. **No Inspector configuration needed** — topics are set to defaults:
   - Publishes to `/holoassist/mode_command`
   - Subscribes to `/holoassist/mode_status`

---

## 3. Wire OperatingModeController into RadialMenu

The RadialMenu has a new "Auto Mode" button on page 1 (Robot Controls page). It needs a reference to the OperatingModeController.

1. **Select the RadialMenu GameObject** in the hierarchy
2. In the Inspector, find the new **Mode Controller** field
3. **Drag** the `OperatingModeManager` GameObject (or whichever object has `OperatingModeController.cs`) into the `Mode Controller` field

The button will appear on page 1 of the radial menu as "Auto Mode". Toggling it switches between TELEOP and MOVEIT. The dashboard confirms the switch and the button state will update via the status subscription.

---

## 4. ROSAutoConnect (if not already in scene)

Per CLAUDE.md, `ROSAutoConnect.cs` needs to be attached to a GameObject in the hierarchy.

1. Check if any GameObject has `ROSAutoConnect.cs` attached
2. If not: create an empty GameObject named `ROSAutoConnect`, attach the script
3. It will auto-scan `192.168.0.101-109:10000` for `ros_tcp_endpoint`

---

## 5. Verify Bin Setup (for bin verification in Unity)

For the bin verification to work (detecting when a virtual cube enters a bin):

1. Each bin GameObject should have `BinDetector.cs` attached (Seb's script)
2. Each bin needs a **trigger collider** — `BinDetector` auto-creates one from mesh bounds, but verify it exists
3. The `BinStatusPanel` should be in the scene (created by Seb's `HoloAssist > Setup All UI Features` menu)
4. Virtual cubes from `CubePoseSubscriber` need **Colliders** to trigger the bin detection

---

## 6. Scene Checklist

After all changes, verify in Play mode (with fake hardware running):

- [ ] `CubePoseManager` exists with `CubePoseSubscriber.cs` attached
- [ ] `OperatingModeManager` exists with `OperatingModeController.cs` attached
- [ ] RadialMenu's `Mode Controller` field is assigned
- [ ] RadialMenu page 1 shows "Auto Mode" button
- [ ] Robot Base is assigned on `CubePoseSubscriber`
- [ ] `ROSAutoConnect` is in the scene
- [ ] Bins have `BinDetector.cs` with trigger colliders

---

## Summary of New Scripts

| Script | Purpose | Attach To |
|---|---|---|
| `CubePoseSubscriber.cs` | Subscribes to cube poses from perception, spawns virtual objects | New `CubePoseManager` GO |
| `OperatingModeController.cs` | TELEOP/MOVEIT mode switching via ROS topics | New `OperatingModeManager` GO |

## Modified Scripts

| Script | Change |
|---|---|
| `RadialMenu.cs` | Added `modeController` field + "Auto Mode" button on page 1 |
