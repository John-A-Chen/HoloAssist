# Perception-Spawned Object → Task Tracking Integration Plan

**Status:** planning, not implemented.
**Owner:** Sebastian (Subsystem 4 — XR Scene & Visualisation).
**Depends on:** Subsystem 1 perception output from John (cube pose topics + future class label).
**Prerequisite features (already in main):** `ObjectClass`, `BinDetector` correct/wrong split, `TaskTracker`, `EventBanner` progress ring + fail state, `TaskConfigPanel`.

---

## 1. Why this document

Today, dropping a Unity-side object (e.g. the moveable sphere) tagged with `ObjectClass`
into a bin updates the task tracker as expected — green/red flash, ✓/✗ counters
advance, banner progress ring fills. **Perception-spawned cubes are deliberately
untagged** so the existing hardware-tested perception pipeline isn't disturbed.

This document plans the next step: cubes that perception spawns in Unity must
participate in the task system **exactly like the manually-placed sphere does**,
with zero changes to the perception ROS nodes themselves on the first pass, and a
clean upgrade path to perception-driven class labels later.

The goal is a streamlined integration — so when we sit down to make the change,
every file, every line, and every inspector field is already mapped out.

---

## 2. Current data flow (factual)

```
┌────────────────────────────────────────────────────────────┐
│ ROS 2 side (Ubuntu workstation, ros2_ws)                   │
│                                                             │
│  AprilTag detector ──► holoassist_cube_pose_node ──►        │
│    /holoassist/perception/april_cube_{1-4}_pose            │
│    (PoseStamped, frame_id = workspace_frame OR base_link)  │
│                                                             │
│  cube_pose_relay.py ──►                                    │
│    /holoassist/unity/cube_{1-4}_pose                       │
│    (PoseStamped, frame_id = base_link)                     │
└──────────────────────────────┬─────────────────────────────┘
                               │  ros_tcp_endpoint (port 10000)
                               ▼
┌────────────────────────────────────────────────────────────┐
│ Unity side (Quest 3 / Editor)                              │
│                                                             │
│  CubePoseSubscriber.cs (Start)                              │
│    For i in 1..cubeCount:                                  │
│      instance = CreateVirtualObject(i)                     │
│        ├─ Instantiate(cubePrefabs[i-1])  OR                │
│        └─ GameObject.CreatePrimitive(Cube) + tint color    │
│      instance.SetActive(false)                              │
│      ros.Subscribe(topic, OnCubePose)                       │
│                                                             │
│  OnCubePose: store target pose                              │
│  Update: smoothly lerp instance to target pose;             │
│          SetActive(true) on first pose, false on timeout    │
└────────────────────────────────────────────────────────────┘
```

**Where the task system listens (already wired):**

```
sphere / cube (with ObjectClass) ─drop─►
   BinDetector.HandleEntry()
     ├─ ObjectClass.FindOn(collider) — walks up from the collider
     ├─ if null → ignore
     ├─ if isGood → correctCount++, ConfettiBlaster.FireIfEnabled(),
     │             EventBanner.FlashCorrect(),
     │             TaskTracker.OnSort(this, oc, true)
     └─ else     → wrongCount++,
                   EventBanner.FlashWrong(),
                   TaskTracker.OnSort(this, oc, false)
```

So the integration question reduces to: **how does the cube spawned in
`CubePoseSubscriber.CreateVirtualObject` end up with an `ObjectClass` component on
its hierarchy by the time it enters a bin?**

---

## 3. What's missing today

1. **`CubePoseSubscriber.CreateVirtualObject` doesn't attach `ObjectClass`.** Spawned
   instances have a Renderer + Collider (if primitive) but no class metadata.
2. **No mapping** from "cube_1 / cube_2 / cube_3 / cube_4" to a class name and
   good/bad flag. The perception side identifies cubes by AprilTag ID — there's
   currently no semantic class info travelling over ROS.
3. **No Rigidbody on the cube.** It's a kinematic transform driven by the
   subscriber. Trigger detection still works (bin's child trigger has a kinematic
   Rigidbody, so the trigger pair fires), but if any future code path expects a
   Rigidbody-driven cube (e.g. dropping by physics after detection loss), it will
   need one.
4. **No fallback Collider on prefab spawns.** Primitive Cubes get a BoxCollider for
   free; user-supplied prefabs may or may not include one. Without a Collider, the
   cube cannot trigger `BinDetector`.

---

## 4. Proposed design (high level)

### 4.1 Tagging strategy — one master toggle, two parallel arrays

Three inspector fields on `CubePoseSubscriber` decide everything:

1. **`tagSpawnedCubes` (bool, default `false`)** — master gate. Off = do nothing,
   identical to today's behaviour.
2. **`cubeClassNames[]`** — string per cube index. Empty = skip that one cube.
3. **`cubeIsGood[]`** — bool per cube index. Drives correct vs wrong on sort.

If a spawned prefab already has an `ObjectClass` component baked in (e.g. a
high-fidelity `Bomb.prefab`), the spawner respects it and the inspector arrays
are ignored for that cube. So prefabs always win over inspector defaults, but
there's no explicit "priority list" to memorise — just one rule: prefab-baked
component takes precedence if present.

The two-array form is intentionally simpler than a custom serializable struct:
- No nested foldouts in the inspector — fields appear flat.
- Trivial to read and edit.
- Length mismatches gracefully fall back to `className = "cube_N"` and
  `isGood = true`.

### 4.2 Default behaviour change

When the user runs without enabling `tagSpawnedCubes`, perception-spawned
cubes continue to **NOT** count toward the task — exactly matching today's
behaviour. Zero-config = zero regression. The master toggle is the kill switch.

### 4.3 Visual identity

Class names should also drive cube colour when no prefab is supplied, so
spectators in the headset can visually distinguish "good" from "bad" cubes during
early dev before high-fidelity meshes arrive. Today the primitive Cube path picks
from a hardcoded `[Color.red, Color.green, Color.blue, orange]` array indexed by
cube ID. Long-term we drive the colour off `ObjectClass.isGood` so green = good,
red = bad, regardless of which AprilTag is on it.

---

## 5. Files & locations

### 5.1 Files that **must change**

| Path | Lines (approx) | What changes |
|---|---|---|
| `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs` | ~149–183 (`CreateVirtualObject`) + 1–30 (inspector fields) | Add three inspector fields (`tagSpawnedCubes`, `cubeClassNames[]`, `cubeIsGood[]`); ~12 added lines inside `CreateVirtualObject` to attach `ObjectClass` and ensure a Collider exists. |

That is the **only** file that strictly needs to change for the MVP integration.
All other changes below are optional / additive.

### 5.2 Files that **may need changes** (optional, scope-dependent)

| Path | When | What |
|---|---|---|
| `Unity/My project/Assets/Scripts/ObjectClass.cs` | If we want default-colour-on-isGood (4.3) | Add a static helper `ApplyVisualHint(GameObject, ObjectClass)` that tints renderers. Keep ObjectClass itself as a pure marker. |
| `Unity/My project/Assets/Scripts/TaskTracker.cs` | If we want perception cubes counted under `SortAllGood` | None — already correct. `FindObjectsByType<ObjectClass>` includes inactive, and cubes start inactive until a pose arrives, then activate. The cache is 1s, so the denominator catches up. |
| `Unity/My project/Assets/Scripts/BinDetector.cs` | If we want to dedupe re-entries (cube briefly leaves + re-enters trigger due to perception jitter) | `objectsInBin` is a HashSet keyed on the Collider; same cube re-entering after exit DOES re-count. Could add a `recentlyCountedCooldown` per object. Not blocking for MVP. |

### 5.3 Files that **must be added** — none

The integration is small enough that no new C# scripts are needed. All required
plumbing lives inside `CubePoseSubscriber`.

### 5.4 ROS-side files (NOT touched in this MVP)

The intent of Q6 from earlier planning was explicit: keep the perception
pipeline hardware-tested and unchanged. So **none of these need edits for the
MVP**:

| Path | Role | Touch? |
|---|---|---|
| `ros2_ws/cube_pose_relay.py` | Workspace-frame → base_link relay | **No** |
| `ros2_ws/src/holo_assist_depth_tracker/.../holoassist_cube_pose_node` | Publishes `/holoassist/perception/april_cube_{1-4}_pose` | **No** |
| `apriltag_ros` node config | Tag size, families, publish_tf | **No** |
| `launch.py` (perception block) | Phase 4 launch sequence | **No** |

A future "perception drives the class" enhancement would touch the cube_pose_node
to add a `class_name` field — covered in Section 11.

---

## 6. The actual change in `CubePoseSubscriber.cs`

> **TL;DR — minimum viable integration**
>
> Add four inspector fields to `CubePoseSubscriber`:
> 1. `tagSpawnedCubes` (bool, default false) — master toggle
> 2. `cubeClassNames` (string[]) — class label per cube index
> 3. `cubeIsGood` (bool[]) — correct/wrong flag per cube index
> 4. `autoAddCollider` (bool, default true) — fallback BoxCollider on spawn
>
> Add ~12 lines inside `CreateVirtualObject` to attach an `ObjectClass` to
> each cube based on the arrays. Prefab-baked `ObjectClass` (if any) wins.
>
> That's the whole MVP. Subsections §6.1–§6.3 cover the code shape and
> wiring; §6.4 explains how the result interacts with TaskConfigPanel; §6.5
> is a forward-looking roadmap of optional inspector fields to add later.

**Design principle:** the simplest workable form. One master toggle plus two
parallel inspector arrays — no custom structs, no nested foldouts, no priority
layers to memorise. If a prefab happens to ship with its own `ObjectClass`
component already attached we respect that; otherwise the inspector arrays
decide.

### 6.1 New inspector section

Three flat fields below the existing `[Header("Virtual Objects")]` block:

```csharp
[Header("Task System Integration")]

[Tooltip("Master toggle. When off, spawned cubes are untagged (preserves today's behaviour).")]
public bool tagSpawnedCubes = false;

[Tooltip("Class name per cube (index 0 = cube_1). Empty string = skip that one cube.")]
public string[] cubeClassNames = { "red", "green", "blue", "yellow" };

[Tooltip("isGood per cube (index 0 = cube_1). True = correct sort, false = wrong sort.")]
public bool[] cubeIsGood = { true, true, true, true };

[Tooltip("If a Collider is missing on the spawned instance, add a BoxCollider so BinDetector can pick it up.")]
public bool autoAddCollider = true;
```

That's it. No `ClassMapEntry` struct, no skip field, no Rigidbody flag (the
bin's own trigger has a Rigidbody so detection works without one on the cube).

### 6.2 `CreateVirtualObject` modifications

**Exact insertion anchor:** in the current file, locate this pair of lines at
the end of `CreateVirtualObject` (around line 180–181):

```csharp
        obj.name = $"VirtualCube_{cubeIndex}";
        obj.SetActive(false);                  // ← insert NEW block BEFORE this line
        return obj;
```

Insert the following block **between** `obj.name = …` and `obj.SetActive(false)`.
Do not modify any line above that anchor. The block is self-contained — no
helper methods, no new `using` directives required (both `ObjectClass` and
`BoxCollider` are in the same namespace as `CubePoseSubscriber`).

```csharp
        // --- NEW: ensure Collider so BinDetector trigger fires on entry ---
        if (autoAddCollider && obj.GetComponentInChildren<Collider>() == null)
        {
            var box = obj.AddComponent<BoxCollider>();
            box.size = Vector3.one * objectScale;
        }

        // --- NEW: tag with ObjectClass for the task system ---
        // Prefab-baked ObjectClass wins (early exit if any exists in hierarchy).
        if (tagSpawnedCubes && obj.GetComponentInChildren<ObjectClass>() == null)
        {
            int idx = cubeIndex - 1;
            string className = (cubeClassNames != null && idx >= 0 && idx < cubeClassNames.Length)
                ? cubeClassNames[idx]
                : $"cube_{cubeIndex}";
            bool isGood = (cubeIsGood != null && idx >= 0 && idx < cubeIsGood.Length)
                ? cubeIsGood[idx]
                : true;
            if (!string.IsNullOrEmpty(className))
            {
                var oc = obj.AddComponent<ObjectClass>();
                oc.className = className;
                oc.isGood = isGood;
            }
        }
```

**Behaviour rules in three lines:**
1. `tagSpawnedCubes = false` → no tagging happens. Identical to today.
2. `tagSpawnedCubes = true` + prefab already has `ObjectClass` → prefab wins.
3. `tagSpawnedCubes = true` + no prefab `ObjectClass` → use index-N entry of
   the arrays; null arrays / out-of-bounds / empty `className` all fall back
   gracefully (`"cube_N"`, `isGood = true`, or skip).

### 6.3 Inspector wiring (per scene)

1. Select the **CubePoseSubscriber** GameObject in the Hierarchy.
2. Tick **Tag Spawned Cubes**.
3. **Cube Class Names** array shows four entries by default. Edit in place.
4. **Cube Is Good** array shows four toggles. Edit in place.

   | i | cubeClassNames[i] | cubeIsGood[i] |
   |---|---|---|
   | 0 (cube_1) | `red`   | ✅ |
   | 1 (cube_2) | `green` | ✅ |
   | 2 (cube_3) | `blue`  | ✅ |
   | 3 (cube_4) | `bomb`  | ☐ |

   For first-test convenience leave all four `isGood = true` — confirms the
   green-flash path. Then flip cube_4 to false to test the red-flash + fail path.

5. Hit Play. Console should log e.g. `[BinDetector] CORRECT: red entered Bin_X`.
6. Quickest revert to "perception cubes are inert": untick **Tag Spawned Cubes**.
   No need to clear individual entries.

### 6.4 TaskConfigPanel (TCP) compatibility & class-naming guide

The whole point of tagging perception cubes is so they participate in the task
system. The TCP controls drive `TaskTracker`; perception cubes feed `TaskTracker`
through `BinDetector → OnSort()` the same way the manually-placed sphere does.
Source of the `ObjectClass` (prefab-baked vs spawner-attached) is invisible to
TaskTracker — so by design, the plan is fully compatible with every TCP setting.

**Per-control compatibility:**

| TCP control | What it changes | Perception cubes work? |
|---|---|---|
| **Fixed Target** | `mode = FixedTarget`, progress = `totalCorrect / targetCount` | ✅ Cube drops with `isGood = true` advance `totalCorrect` identically to the sphere. |
| **Sort All** | `mode = SortAllGood`, progress = `totalCorrect / good ObjectClass count in scene` | ✅ The denominator uses `FindObjectsByType<ObjectClass>(FindObjectsInactive.Include, ...)` so inactive cubes (waiting for pose / in timeout) are still counted. Stable progress under perception jitter. |
| **One Each** | `mode = OneOfEachClass`, progress = `distinct classes seen / distinct classes in scene` | ✅ Each cube's `className` becomes one distinct entry. Set `cubeClassNames = {"red","green","blue","yellow"}` → 4 distinct classes required to win. |
| **TARGET − / +** | `targetCount` | ✅ Only consulted by FixedTarget. Doesn't care where correct sorts originate. |
| **Reset** | Zeros all counters, calls `bd.ResetScore()` on every bin | ✅ Cube GameObjects are not destroyed — perception keeps streaming poses so cubes stay visible and you can resume immediately. |
| **Timer / Reset Timer** | Independent of task state | ✅ No interaction with classification. |
| **`wrongFailThreshold`** | Triggers `Failed` state | ✅ Dropping a cube with `isGood = false` increments `totalWrong`. Banner stays red until **Reset**. |

**Behaviour quirk — `SortAllGood` re-drop:**

`TaskTracker` counts every successful sort, not distinct sorted instances. In
`SortAllGood` you could drop the same cube four times to "win" without ever
sorting cubes 2, 3, 4. This is a TaskTracker design choice that pre-dates the
perception integration — the sphere has the same loophole today. `OneOfEachClass`
is naturally protected because it uses a `HashSet<string>` of seen classes.

If `SortAllGood` becomes a demo mode and re-drop cheesing is a problem, fix is
~10 lines in TaskTracker: track a `HashSet<int>` of sorted GameObject instance
IDs, and for that mode use `distinctSorted / denominator` instead of
`totalCorrect / denominator`. Independent of the perception work.

**Class-naming patterns for common demo intents:**

| Demo intent | `cubeClassNames[]` | `cubeIsGood[]` | TCP mode + extras |
|---|---|---|---|
| "Sort 5 anything-cubes" | `"cube", "cube", "cube", "cube"` | all `true` | FixedTarget, target = 5 |
| "Sort one of each kind" | `"red", "green", "blue", "yellow"` (distinct) | all `true` | OneOfEachClass |
| "Sort everything in workspace" | any names | all `true` | SortAllGood (be aware of re-drop quirk) |
| "Defuse bombs, sort tomatoes" | `"tomato", "tomato", "tomato", "bomb"` | `true, true, true, false` | FixedTarget, target = 3, `wrongFailThreshold = 1` → one bomb = lose |
| "Mixed game — colour-coded bins, one bomb" | `"red", "green", "blue", "bomb"` | `true, true, true, false` | OneOfEachClass + `wrongFailThreshold = 2` |

The key insight: TCP doesn't need to change, the **class-name strings you choose
in the inspector decide the gameplay**. Inspector arrays are the single point of
configuration for "what does this scene's task feel like?".

### 6.5 Future spawn-customisation options (inspector roadmap)

Phase-1 implementation (§6.1) adds only the four fields needed for task
integration. As we develop more elaborate demos, the inspector will probably
grow with additional per-cube or global customisation hooks. This section
inventories the **most likely additions** so future contributors know which
options have already been discussed and where they'd live.

**Design principle:** every new field should be optional and default to today's
behaviour. If you don't touch the field, the spawn looks like it does now. This
prevents the inspector from becoming a wall of mandatory checkboxes for the
common case.

**Visual customisation**

| Field | Type | Purpose |
|---|---|---|
| `cubeMaterials` | `Material[]` | Per-cube material override. Lets you tint cube_4 differently without making a unique prefab. Index matches cube number. |
| `tintByIsGood` | `bool` | When the primitive-Cube fallback path runs, override its colour from the existing red/green/blue/orange array to green-if-good / red-if-bad. Useful for spectator visibility before high-fidelity meshes arrive. |
| `goodTint` / `badTint` | `Color` | Companion colours for `tintByIsGood`. Default green/red. |
| `outlineWhenSelected` | `bool` | Adds a thin emissive shader pass when cube is the "next to sort" hint object. Useful in coaching mode. |

**Physics & collider customisation**

| Field | Type | Purpose |
|---|---|---|
| `autoAddCollider` | `bool` (already proposed in §6.1) | Already in plan. Drop a BoxCollider if the spawn has none. |
| `autoAddKinematicRigidbody` | `bool` | Adds a kinematic Rigidbody to the spawn. Off by default — bin trigger already has one so detection works without it. Useful if a future code path expects a Rigidbody-driven cube. |
| `cubeLayer` | `string` or `LayerMask` | Lets the user assign cubes to a Unity layer (e.g. "Interactable") so per-layer physics queries (RaycastMask, collision matrix) work cleanly. |
| `colliderSizeMultiplier` | `float` | Scales the auto-added BoxCollider relative to `objectScale`. Useful when the visual mesh is smaller/larger than the physics shape we want. |

**Pose / coordinate adjustments**

| Field | Type | Purpose |
|---|---|---|
| `cubePositionOffsets` | `Vector3[]` | Per-cube positional nudge applied **after** the ROS pose. Compensates for prefab pivot mismatches (e.g. a tomato mesh whose origin is at the bottom instead of the centre). |
| `cubeRotationOffsets` | `Vector3[]` | Per-cube euler-angle rotation offset. Same purpose, for orientation. |
| `globalScaleMultiplier` | `float` | Scales every cube uniformly. Quick "make all cubes 50% bigger for the next test" knob. |
| `cubeScales` | `float[]` | Per-cube scale override — different physical objects might be different sizes. |

**Lifecycle behaviour**

| Field | Type | Purpose |
|---|---|---|
| `poseTimeout` | `float` (already exists) | Already in current script — kept for completeness. |
| `fadeOnTimeout` | `bool` | Smoothly fade the cube out over `fadeDuration` instead of hard `SetActive(false)` when pose stops. Less jarring for the operator. |
| `fadeDuration` | `float` | Companion to `fadeOnTimeout`. |
| `hideUntilFirstPose` | `bool` | Currently always true via `SetActive(false)` in the spawner. Make explicit so it can be disabled (debug / showing all 4 cubes in their last-known pose immediately). |

**Interaction**

| Field | Type | Purpose |
|---|---|---|
| `cubesGrabbable` | `bool` | Adds `XRGrabInteractable` to every spawn so the operator can manually move cubes via the right-hand ray. Useful for testing bin detection without perception running. |
| `manualOverridesPerception` | `bool` | When a cube is being grabbed, ignore incoming pose updates from ROS so the operator's manipulation isn't fought by the perception stream. |
| `cubeTags` | `string[]` | Unity tag assigned to each cube (e.g. "TaskObject"). Lets other systems (audio, particle, accessibility) target task objects with a tag-based query. |

**Inspector ergonomics**

If the field count keeps growing, consider migrating to one of these patterns —
**not needed for MVP, but worth knowing about**:

- **`[Foldout]` headers** (Unity 2022+): visually collapse "Visual customisation",
  "Physics", "Interaction" into separate inspector sections that the user
  can expand on demand. Zero code impact, just `[Header]` calls.
- **Per-cube `ScriptableObject` profile**: replace the parallel arrays with
  `CubeProfile[] cubeProfiles` where each `CubeProfile` is a ScriptableObject
  asset bundling className, isGood, prefab, scale, color, etc. Reusable across
  scenes; survives rename refactors. ~30 lines + one new asset file. Recommended
  if we end up with more than ~6 per-cube fields.
- **Runtime config panel** (analogous to TaskConfigPanel): expose a subset of
  the fields via a world-space panel the user can toggle from the radial menu.
  Useful for live-tweaking during a demo without leaving the headset. Cost:
  significant — adopt only if a specific demo needs it.

**Suggested implementation order**

Don't do all of this at once. The smart sequence:

1. **MVP** — §6.1 fields only (`tagSpawnedCubes`, `cubeClassNames`, `cubeIsGood`, `autoAddCollider`).
2. **+ Visual** — `tintByIsGood` + `goodTint` / `badTint` so cubes look right before high-fidelity prefabs arrive. ~10 lines.
3. **+ Pose offsets** — `cubePositionOffsets` / `cubeRotationOffsets` once high-fidelity prefabs ship and their pivots need calibration.
4. **+ Interaction** — `cubesGrabbable` + `manualOverridesPerception` when we need offline-from-perception testing.
5. **+ ScriptableObject migration** — only if the inspector becomes unwieldy.

Each phase is independent; users at any phase get a working integration without
needing to follow the rest of the roadmap.

---

## 7. ROS-side schema (current + future)

### 7.1 Topics consumed today (unchanged)

| Topic | Type | Frame | Producer | Consumer |
|---|---|---|---|---|
| `/holoassist/perception/april_cube_{1-4}_pose` | `geometry_msgs/PoseStamped` | `workspace_frame` (sim) or AprilTag frame (real) | `holoassist_cube_pose_node` | `cube_pose_relay.py` |
| `/holoassist/unity/cube_{1-4}_pose` | `geometry_msgs/PoseStamped` | `base_link` | `cube_pose_relay.py` | `CubePoseSubscriber.cs` |

Cube identity travels only as the numeric topic suffix. The MVP integration
maps that numeric ID to class/goodness on the Unity side via the inspector,
without touching ROS.

### 7.2 Future "perception drives the class" contract

When perception is ready to publish class labels (e.g. colour from CV +
AprilTag ID lookup), we propose a small extension:

**Option A (simplest — additive):** Publish a parallel topic per cube:

```
/holoassist/perception/april_cube_{1-4}_class    std_msgs/String
```

Unity subscribes to both pose + class topics, applies the perception-driven
class on receipt (overriding the inspector mapping).

**Option B (cleaner — typed message):** Introduce a custom message type:

```
HoloAssistObjectPose.msg:
    geometry_msgs/PoseStamped pose
    string class_name      # "red", "tomato", "bomb"
    bool is_good           # true if sorting should count as correct
    float32 confidence     # detection confidence 0..1
```

Requires `holo_assist_msgs` package + ROS-TCP-Connector message regeneration.
Heavier lift but is the right long-term design.

**Recommendation:** start with Option A when perception is ready, migrate to B
when the team is comfortable with custom messages.

---

## 8. Migration plan (sim → hardware)

### Phase 0 — Setup (no ROS / hardware needed)
1. Implement the `CubePoseSubscriber.cs` change.
2. Add a test scene object with `ObjectClass(isGood=true)` and verify the task
   pipeline still works.
3. Smoke-test that the inspector **Tag Spawned Cubes** toggle + the two arrays
   appear and accept edits.

### Phase 1 — Simulated perception (no robot, no camera)
1. `./launch.sh --perception` (fake hardware + sim perception). Spawns
   `april_cube_1..4` in the perception simulator.
2. Press Play in Unity. Watch console for `[CubePoseSubscriber] Subscribed to
   /holoassist/unity/cube_1_pose` (×4).
3. Confirm cubes appear in scene at the expected poses.
4. Grab a cube with the right-hand controller, drop it into a bin. Verify:
   - Green flash on banner (if `isGood`)
   - ✓ counter on BinStatusPanel increments
   - Progress ring on banner advances
5. Repeat with a cube where `isGood = false` → red flash, ✗ counter, no
   progress ring advance.
6. Iterate on `cubeClassNames[]` / `cubeIsGood[]` until classifications match design intent.

### Phase 2 — Real perception, no robot
1. RealSense + AprilTag printouts in the workspace.
2. `./launch.sh --robot-ip 192.168.0.194 --perception` (full stack, but the user
   does NOT touch the robot — just observes the spawned cubes).
3. Manually move physical AprilTag cubes around; confirm virtual cubes track
   them.
4. Drop the virtual cubes into bins via XR — same checks as Phase 1.
5. Validate that classification still works under jitter from real perception
   (cubes briefly disappearing, pose noise, etc.).

### Phase 3 — Robot integrated
1. Same launch, but now Oliver's autonomous sorting drives the robot.
2. Perception cubes are picked + placed by the robot — `BinDetector` should
   fire on entry the same way.
3. Tune `BinDetector.triggerSizeMultiplier` if needed so the robot's gripper
   placement reliably crosses the trigger zone.

### Rollback
At any phase, unticking **Tag Spawned Cubes** on `CubePoseSubscriber` returns
to today's "perception cubes are inert" behaviour with one click — no code
revert needed. Useful for debugging or falling back during a demo if the task
system misbehaves.

---

## 9. Test plan

### 9.1 Manual verification matrix

| # | Scenario | Expected |
|---|---|---|
| 1 | Drop manually-tagged sphere (`isGood=true`) into bin | Banner flashes green, confetti, ✓ count +1 |
| 2 | Drop manually-tagged sphere (`isGood=false`) into bin | Banner flashes red, no confetti, ✗ count +1 |
| 3 | Drop perception-spawned cube_1 (`cubeClassNames[0]="red"`, `cubeIsGood[0]=true`) into bin | Same as #1 — proves the spawner attached ObjectClass |
| 4 | Drop perception-spawned cube_4 (`cubeClassNames[3]="bomb"`, `cubeIsGood[3]=false`) into bin | Same as #2 — proves the bad path |
| 5 | Drop perception cube while **Tag Spawned Cubes** is unticked | No banner flash, no counters move (current behaviour preserved) |
| 6 | Cycle TaskTracker to `SortAllGood`; drop one of each cube → progress hits 100% | Banner shows COMPLETE, ring turns full-green |
| 7 | Cycle to `OneOfEachClass` with two cubes sharing className `"red"` | Progress = 0.5 (1 of 2 distinct), not 1.0 — confirms dedupe |
| 8 | Reset via TCP → drop again | Counters back to 0 then resume incrementing |
| 9 | Set `wrongFailThreshold = 2`; drop 2 bad cubes | Banner turns persistent red, text = "FAILED ✗2", stays red after flash fades |

### 9.2 Logs to watch

- `[CubePoseSubscriber] Subscribed to /holoassist/unity/cube_N_pose` (×N) — startup.
- `[BinDetector] CORRECT: <className> entered <binName> (✓N, ✗M)` — on each good sort.
- `[BinDetector] WRONG: <className> entered <binName>` — on each bad sort.
- `[BinDetector] <name> entered <bin> (untagged — ignored)` — if **Tag Spawned
  Cubes** is off or `cubeClassNames[i]` is empty. Useful negative-case verification.
- `[TaskTracker] → Complete / Failed / Reset` — state transitions.

### 9.3 Things that would indicate a bug

- Cubes spawn but never trigger bins → no Collider on spawn (check `autoAddCollider`).
- Cubes trigger bins but no counter moves → `ObjectClass` not attached (check
  inspector mapping; check `Skip` field).
- Counter moves twice per drop → cube briefly exited+re-entered trigger; dedupe
  via `BinDetector.objectsInBin` HashSet *should* prevent this but worth
  watching.

---

## 10. Edge cases & risks

| Risk | Likelihood | Mitigation |
|---|---|---|
| Perception cube briefly drops out (no pose for >timeout=3s) while inside bin → `SetActive(false)` removes it from physics; on return, fires another OnTriggerEnter → double-counts | Medium during real-perception jitter | Add a recently-seen-in-bin grace window in `BinDetector`, OR bump `poseTimeout` to 5s+ on the subscriber. |
| User puts an ObjectClass on the cube prefab AND configures the inspector arrays → which wins? | Low (documented) | Prefab wins. The tagging block in `CreateVirtualObject` early-returns if `GetComponentInChildren<ObjectClass>() != null`. |
| Auto-added BoxCollider is sized for primitive `objectScale` but the prefab mesh is bigger | Medium when high-fidelity meshes arrive | Recommend baking the Collider into the prefab. Document this so prefab authors know. |
| Multiple bins overlap the same cube position → cube counts as sorted in two bins | Low (bins shouldn't overlap) | `BinDetector.ShouldDetect` already rejects entries whose parent has a `BinDetector` (adjacent-bin guard). Standard cube → bin is fine. |
| Reset Task happens but the cubes are still inside bins → next perception update doesn't re-fire `OnTriggerEnter` because the Collider was never re-added | Low — Reset doesn't touch cubes | Workaround: move the cube briefly out and back in, OR call `bd.ResetScore()` and have the test plan re-drop. Document the expectation. |
| Inspector arrays shorter than `cubeCount` → out-of-bounds | Easy mistake | The tagging block bounds-checks `idx < cubeClassNames.Length` and falls back to `className = "cube_N"`, `isGood = true`. |
| Reordering of cube IDs by perception (e.g. tag 1 ↔ tag 2 swap) | Possible if AprilTag printouts get swapped on the workbench | This is a perception-side semantic issue, not a Unity issue. The inspector arrays assume stable tag↔cube_N assignment. If perception starts publishing class labels per cube (see §11.1), this risk disappears. |

---

## 11. Future enhancements (out of scope for MVP)

### 11.1 Perception-driven class labels

Once perception is confident enough to classify objects by colour / shape /
AprilTag-payload lookup, switch class assignment from the Unity inspector to
the ROS message itself. Sketch:

1. **ROS side:** `holoassist_cube_pose_node` publishes
   `/holoassist/perception/april_cube_{N}_class` (`std_msgs/String`) alongside
   pose.
2. **Unity side:** `CubePoseSubscriber` subscribes to the class topic, stores
   latest class per cube, applies it to the ObjectClass component on the
   spawned instance (creating one if needed). Inspector mapping becomes the
   fallback for cubes the perception node hasn't classified yet.

### 11.2 Real → virtual prefab map (the contract HD criterion)

When high-fidelity meshes arrive (tomato, bomb, etc.), the inspector's
`cubePrefabs[]` already supports per-index prefab swap. Combined with the
the inspector arrays here, the workflow becomes:

| cubeIndex | Prefab | ClassMap |
|---|---|---|
| 1 | `Tomato.prefab` (has ObjectClass baked in) | optional override |
| 2 | `Bomb.prefab` (has ObjectClass baked in) | optional override |
| 3 | `Gem.prefab` (no ObjectClass) | `("gem", true)` |
| 4 | empty | `("crate", false)` — falls through to primitive Cube |

### 11.3 Per-bin acceptance rules

Today an object is "good" or "bad" globally regardless of which bin it enters.
The original Q1 discussion considered a per-bin rules list (`BinDetector.acceptedClasses`).
If the contract demonstration requires "red goes in red bin, blue goes in blue bin",
we'd need to:

1. Add `acceptedClasses: List<string>` to `BinDetector`.
2. In `HandleEntry`, classify based on `oc.className ∈ acceptedClasses` instead
   of `oc.isGood` alone (or AND-combine: must be `isGood` AND in `acceptedClasses`).
3. Wrong-bin sort = red flash + ✗ count.
4. ObjectClass keeps `isGood` for the unsorted-vs-sorted-anywhere distinction.

Estimated effort: 30 lines of code + inspector update. **Defer until needed.**

### 11.4 Hardware-specific session logging

Once perception cubes contribute to the task, the session log (Nic's
`SessionLogger.cs`) should record per-cube events:

- Cube X first detected at T+12.3s.
- Cube X sorted into Bin Y at T+24.7s (correct).
- Cube X re-entered Bin Y at T+25.1s (cooldown ignored).

This is a Subsystem 3 cross-cut, not a Subsystem 4 change. Flag it to Nic when
this integration ships.

---

## 12. Step-by-step rollout checklist

Use this as a literal checklist when doing the work.

- [ ] Pull latest `main` to make sure ObjectClass / TaskTracker / TaskConfigPanel are present.
- [ ] Open `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs`.
- [ ] Add three inspector fields (`tagSpawnedCubes`, `cubeClassNames[]`, `cubeIsGood[]`) + `autoAddCollider` per §6.1.
- [ ] Insert ~12 lines in `CreateVirtualObject` per §6.2 (Collider check + ObjectClass attach block).
- [ ] Compile in Unity — verify no errors, the new fields show up in the inspector.
- [ ] Smoke-test with **Tag Spawned Cubes** off → perception cubes still inert (Phase 0 / rollback).
- [ ] Tick **Tag Spawned Cubes** and verify defaults per §6.3 → drop test → ✓ flash + count.
- [ ] Flip one cube's `cubeIsGood[i]` to false → drop that cube → red flash + ✗ count.
- [ ] Run Phase-1 sim test (§8).
- [ ] Update `Subsystem-4-Documentation/` with whatever changed from this plan.
- [ ] Commit on a branch, push, get a teammate to review on hardware before merging.

---

## 13. Appendix

### 13.1 Files referenced by this plan

| Path | Role |
|---|---|
| `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs` | The one C# file that changes |
| `Unity/My project/Assets/Scripts/ObjectClass.cs` | Component the spawner attaches |
| `Unity/My project/Assets/Scripts/BinDetector.cs` | Trigger handler, already wired to ObjectClass |
| `Unity/My project/Assets/Scripts/TaskTracker.cs` | Aggregator, already wired |
| `Unity/My project/Assets/Scripts/EventBanner.cs` | Visual feedback target |
| `Unity/My project/Assets/Scripts/TaskConfigPanel.cs` | Operator-facing mode picker |
| `ros2_ws/cube_pose_relay.py` | NOT touched — workspace→base_link relay |
| `ros2_ws/src/holo_assist_depth_tracker/.../holoassist_cube_pose_node` | NOT touched in MVP — would extend with a class topic for §11.1 |
| `launch.py` | NOT touched |

### 13.2 Quick commands for testing

Sim mode (no robot, no camera, perception faked):
```bash
./launch.sh --perception
```

Real perception with fake robot:
```bash
./launch.sh --perception
# perception block runs the real RealSense + AprilTag detector
# while robot side stays in fake-hardware mode
```

Full hardware:
```bash
./launch.sh --robot-ip 192.168.0.194 --perception
```

Verify topics are flowing (separate terminal):
```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic hz /holoassist/unity/cube_1_pose
ros2 topic echo /holoassist/unity/cube_1_pose --once
```

### 13.3 Glossary

- **MVP** — minimum viable product. The smallest change that closes the integration.
- **Tagging precedence** — prefab-baked `ObjectClass` wins over inspector arrays. There is no third source today; perception-driven class is a §11.1 future option.
- **`isGood`** — the `ObjectClass` boolean flag determining correct/wrong on sort.
- **Tag Spawned Cubes** — master toggle on `CubePoseSubscriber`. Off = no tagging (today's behaviour).
- **Untagged** — an object with no `ObjectClass` on any ancestor; ignored by `BinDetector`.

---

## 14. Implementation brief — for cold LLM session

This section is a **self-contained checklist** for an AI agent (or a developer
with no session memory) implementing the MVP from scratch. It assumes you have
read **only this file**. If you've read the rest of the document, this is the
TL;DR action plan.

### 14.1 Prerequisite check (do this first)

Verify the following files exist with these signatures. If any are missing,
**STOP** and surface the problem — do not attempt to implement.

| File | Must contain |
|---|---|
| `Unity/My project/Assets/Scripts/ObjectClass.cs` | `public class ObjectClass : MonoBehaviour` with `public string className`, `public bool isGood`, and `public static ObjectClass FindOn(Collider c)` |
| `Unity/My project/Assets/Scripts/BinDetector.cs` | `HandleEntry(Collider)` method that calls `ObjectClass.FindOn(other)` and routes correct/wrong to `EventBanner` + `TaskTracker`. Untagged objects already ignored. |
| `Unity/My project/Assets/Scripts/TaskTracker.cs` | `public static TaskTracker Instance` + `public void OnSort(BinDetector, ObjectClass, bool)` + completion-mode enum |
| `Unity/My project/Assets/Scripts/CubePoseSubscriber.cs` | `public class CubePoseSubscriber` with `CreateVirtualObject(int cubeIndex)` method. Lines 180–181 should read `obj.name = $"VirtualCube_{cubeIndex}"; obj.SetActive(false);` |

Verify with:
```bash
grep -l "public static ObjectClass FindOn" "Unity/My project/Assets/Scripts/ObjectClass.cs"
grep -l "HandleEntry" "Unity/My project/Assets/Scripts/BinDetector.cs"
grep -l "public static TaskTracker Instance" "Unity/My project/Assets/Scripts/TaskTracker.cs"
grep -n "obj.name = \\$\"VirtualCube_" "Unity/My project/Assets/Scripts/CubePoseSubscriber.cs"
```

### 14.2 Do-NOT-modify list

| Path | Why |
|---|---|
| `Unity/My project/Assets/Scripts/ObjectClass.cs` | Component contract — touching breaks all consumers. |
| `Unity/My project/Assets/Scripts/BinDetector.cs` | Already correctly handles tagged + untagged objects. |
| `Unity/My project/Assets/Scripts/TaskTracker.cs` | Already correctly aggregates from `OnSort`. |
| `Unity/My project/Assets/Scripts/EventBanner.cs` | Visual feedback wired correctly. |
| `Unity/My project/Assets/Scripts/TaskConfigPanel.cs` | TCP unrelated to this change. |
| `ros2_ws/cube_pose_relay.py` | ROS-side. Hardware-tested. **Do not touch.** |
| `ros2_ws/src/holo_assist_depth_tracker/**/*` | Perception team's territory. **Do not touch.** |
| `launch.py`, `launch.sh`, `dashboard.sh` | Launch scripts. **Do not touch.** |
| Anything in `ros2_ws/src/` | **Do not touch.** |
| The Unity scene file (`SampleScene.unity`) | No code-side changes should touch it. Inspector wiring is a separate manual step by the user. |

### 14.3 The one file you DO modify

`Unity/My project/Assets/Scripts/CubePoseSubscriber.cs`. Two edits:

**Edit 1 — Add inspector fields.** Find the existing `[Header("Virtual Objects")]`
block (around line 19). Below it, after the existing `defaultPrefab` /
`poseTimeout` / `smoothing` / `objectScale` fields, insert this new block:

```csharp
    [Header("Task System Integration")]

    [Tooltip("Master toggle. When off, spawned cubes are untagged (preserves today's behaviour).")]
    public bool tagSpawnedCubes = false;

    [Tooltip("Class name per cube (index 0 = cube_1). Empty string = skip that one cube.")]
    public string[] cubeClassNames = { "red", "green", "blue", "yellow" };

    [Tooltip("isGood per cube (index 0 = cube_1). True = correct sort, false = wrong sort.")]
    public bool[] cubeIsGood = { true, true, true, true };

    [Tooltip("If a Collider is missing on the spawned instance, add a BoxCollider so BinDetector can pick it up.")]
    public bool autoAddCollider = true;
```

**Edit 2 — Modify `CreateVirtualObject`.** See §6.2 for the exact insertion
anchor (between `obj.name = …` and `obj.SetActive(false)`) and the block to
insert. Copy that block verbatim.

### 14.4 Verification

After both edits compile in Unity:

1. **Inspector check.** Select the GameObject hosting `CubePoseSubscriber`. The
   four new fields appear under a "Task System Integration" header. Default
   values: toggle off, arrays have 4 entries each.

2. **Regression check (toggle off — should match pre-change behaviour).** Hit
   Play with `Tag Spawned Cubes` unticked. Drop a cube into a bin. Console
   should say:
   ```
   [BinDetector] <cubename> entered <binname> (untagged — ignored)
   ```
   No banner flash, no counter movement. Same as before the change.

3. **Happy path (toggle on).** Stop, tick `Tag Spawned Cubes`, hit Play. Drop
   cube_1 into a bin. Console should say:
   ```
   [BinDetector] CORRECT: red entered <binname> (✓1, ✗0)
   ```
   Banner flashes green, confetti fires, ✓ counter on BinStatusPanel = 1.

4. **Bad-path check.** Stop, change `cubeIsGood[0]` to false in inspector. Play,
   drop cube_1. Console:
   ```
   [BinDetector] WRONG: red entered <binname> (✓0, ✗1)
   ```
   Banner flashes red, no confetti, ✗ counter = 1.

5. **Prefab-baked override.** If `cubePrefabs[0]` is set to a prefab that already
   has an `ObjectClass` component (className "tomato", isGood true), playing
   and dropping cube_1 logs `CORRECT: tomato entered …` — prefab wins over the
   inspector array entry.

### 14.5 If verification fails

| Symptom | Likely cause | Fix |
|---|---|---|
| Cubes spawn but don't trigger bins | No Collider on spawn | Confirm `autoAddCollider = true` AND that `GetComponentInChildren<Collider>()` returns null on the prefab. If using a prefab, add a Collider to the prefab. |
| Cubes trigger bins but no counter moves | `ObjectClass` not attached | Confirm `Tag Spawned Cubes` is ticked AND `cubeClassNames[i]` is non-empty for that cube. Inspect the spawned cube in scene hierarchy — should have `ObjectClass` component. |
| Counter moves twice per drop | Cube exit/re-enter triggered twice | Probably perception jitter — bump `poseTimeout` to 5+ seconds, OR don't worry about it for now (sphere has same behaviour). |
| Compile error | Missing reference to `ObjectClass` | Confirm ObjectClass.cs exists in the same Assets/Scripts folder. Both classes are in the global namespace — no `using` directives needed. |

### 14.6 Commit guidance

After verification passes, single commit titled approximately:

```
Tag perception-spawned cubes so they participate in TaskTracker

CubePoseSubscriber gains 4 inspector fields (tagSpawnedCubes, cubeClassNames,
cubeIsGood, autoAddCollider) and attaches an ObjectClass to each spawned cube
when the master toggle is on. Prefab-baked ObjectClass takes precedence. Off
by default — preserves today's behaviour when feature is not enabled.
```

Do NOT include any scene-file changes in this commit. Inspector wiring is a
separate manual step by the user.

### 14.7 What is explicitly out of scope for THIS commit

- Per-bin acceptance rules (would touch BinDetector — defer per §11.3).
- Perception-side `class_name` topic (ROS — defer per §11.1).
- Real-mesh prefab swap (just inspector wiring — user does it separately).
- Any of §6.5 future options (visual tinting, pose offsets, grabbable, etc.).
- Updating SortAllGood to be dedupe-safe (would touch TaskTracker — defer).
- Updating the BinStatusPanel / EventBanner / TaskConfigPanel — none needed.

If asked to do any of the above as part of this commit, push back and confirm
with the user.

---

*End of plan.*
