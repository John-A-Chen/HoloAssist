# Cube Perception Spawning — Dev Log

Working document tracking changes to Unity-side cube perception spawning during the
"blink + slow update" investigation that started 2026-05-26.

**Owner:** Seb (Subsystem 4 — XR/UI)
**Primary script:** [CubePoseSubscriber.cs](../Unity/My%20project/Assets/Scripts/CubePoseSubscriber.cs)
**Related:** ROS-side perception pipeline (`ros2_ws/src/holo_assist_depth_tracker/`), topic relay `/holoassist/perception/april_cube_N_pose` → `/holoassist/unity/cube_N_pose`

> **Teammates testing tomorrow:** the entries dated 2026-05-26 are *deliberate setup changes* on Seb's machine, not bugs. If detector behaviour or cube-spawn behaviour looks different from the original Demo branch state, check this log before reporting a regression.

---

## Symptoms observed (2026-05-26)

Smoke test ran end-to-end on Brio + Quest 3 + 3 AprilTag cubes, no robot. Two issues:

1. **Blink** — cubes despawn momentarily then respawn. Triggered by the `poseTimeout = 3 s` hide-on-stale logic when detection drops out (occlusion, frame loss).
2. **Lag** — visual cube position updates feel slow when the physical cube moves. Dashboard debug viewer shows ~3 FPS on `/holo_assist_depth_tracker/debug_image`.

Hypothesis at the time of the symptom: 3 FPS = the AprilTag detection rate, not the camera publish rate. Overlay node redraws on each detection so the viewer FPS reflects detector throughput. Not yet confirmed with `ros2 topic hz`.

---

## Changes

### 2026-05-26 — AprilTag detector tuned for higher FPS

Edited [`apriltag_all.yaml`](../ros2_ws/src/holo_assist_depth_tracker/config/apriltag_all.yaml) to speed up detection on Seb's laptop. **Deliberate setup change, not a bug** — if a teammate sees different detector behaviour tomorrow, this is why.

| Param | Old | New | Effect |
|---|---|---|---|
| `threads` | 2 | 4 | Uses more CPU cores in parallel |
| `decimate` | 1.0 | 2.0 | Detects on a 2× downsampled image — biggest single speedup |
| `refine` | true | false | Skips subpixel pose refinement — small accuracy cost (~mm), large speed gain |

`blur`, `sharpening`, `debug`, `family`, `size`, `max_hamming`, `pose_estimation_method` unchanged. Tag ID list unchanged.

**Trade-offs:**
- Detection range *might* drop slightly — decimated images need larger tags in pixels. With 32 mm tags at typical Brio working distance this should still be fine; verify by checking detection still triggers when a cube is at the far end of the workspace.
- Pose accuracy drops marginally without `refine`. For the cube-spawn smoke test this is irrelevant; if we later need sub-cm placement against a real robot, flip `refine: true` back on.

**To revert:** set `threads: 2`, `decimate: 1.0`, `refine: true` in the same file. No rebuild needed — runtime param file, takes effect on next `./launch.sh`.

---

### 2026-05-26 — `persistOnTagLoss` Inspector toggle

Added a `bool persistOnTagLoss` field (default `false` — preserves legacy behaviour) on `CubePoseSubscriber`.

- **Off (legacy):** cube hides after `poseTimeout` seconds without a fresh pose, respawns when a new pose arrives. This is the source of the blink.
- **On (new):** cube stays visible at its last received pose indefinitely after `poseTimeout`. Moving-average buffers are cleared exactly once on the visible→stale transition, so a future detection starts a fresh average instead of blending with poses from before the gap.

Implementation: new `bool stale` field on the inner `CubeState`; `Update()` branches on `persistOnTagLoss`; `state.stale = false` resets the transition flag whenever a fresh pose is received.

Also clarified the `smoothing` tooltip: 0 = snap-to-perception-update (event-driven visual moves, no per-frame lerp). Combine `persistOnTagLoss=true` + `smoothing=0` for the "only update visually when perception sees a change, never blink" mode.

No behavioural change when the toggle stays at its default.

---

## How to test the toggle

In the active scene, select the GameObject carrying `CubePoseSubscriber`.

| Toggle config | Expected behaviour |
|---|---|
| `persistOnTagLoss = false`, `smoothing = 0.5` | Legacy. Cube blinks on occlusion, lerps between poses. |
| `persistOnTagLoss = true`, `smoothing = 0.5` | No blink. Cube stays at last pose during occlusion, lerps toward new pose when it returns. |
| `persistOnTagLoss = true`, `smoothing = 0` | No blink, no lerp. Cube snaps to each perception update; stays put between. (Seb's target behaviour.) |
| `persistOnTagLoss = false`, `smoothing = 0` | Cube snaps to updates while detected; still blinks on occlusion. |

Verify by:
1. Pointing the Brio at one cube, confirming it appears in the Quest.
2. Briefly covering the AprilTag with a hand for > `poseTimeout` (default 3 s).
3. Persist on: cube stays where it was. Persist off: cube disappears, reappears when uncovered.

---

## Open items (not yet implemented)

- **Confirm the new FPS after the detector tuning.** Run during a live test:
  ```bash
  ros2 topic hz /camera/camera/color/image_raw   # camera publisher rate
  ros2 topic hz /detections_all                  # apriltag output rate
  ```
  After today's `decimate: 2.0`, `refine: false`, `threads: 4` change, detections should be much closer to camera rate. If camera itself is still ~5–8 Hz, the next fix is MJPG codec on the webcam publisher (see below).

- **MJPG codec on webcam publisher** (not yet applied). [`webcam_image_publisher_node.py`](../ros2_ws/src/holo_assist_depth_tracker/holo_assist_depth_tracker/webcam_image_publisher_node.py) sets width/height/fps but not `CAP_PROP_FOURCC`. OpenCV defaults to raw YUYV, which caps the Brio at ~5–7 Hz over USB 2.0. Adding `self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))` before the size sets would lift it to 30 Hz. Requires workspace rebuild.

- **USB bandwidth check.** If camera publisher is still slow with MJPG, the Brio is competing with the Quest ADB cable on the USB bus. Move the Quest cable to a different USB controller or work over WiFi-only during test.

- **`poseTimeout` exposure.** Currently the user must lower `poseTimeout` to mask short blinks in legacy mode. With `persistOnTagLoss=true` the field is moot. Consider hiding it from the Inspector when persist is on (low priority).

- **Position-change threshold** and other smoothness knobs — moved to the [Proposed future knobs](#proposed-future-knobs--early-concept) section below. `minMoveDistance` is candidate #1 there.

---

## Proposed future knobs — early concept

> **Status: very early concept, nothing committed.** This section is a working plan for additional `CubePoseSubscriber` tuning knobs we *might* add to dial in smoothness further. Not a roadmap — each entry needs sign-off before implementing, and any subset could be skipped if the current toggle + the apriltag tuning prove sufficient.

The existing knobs (`persistOnTagLoss`, `smoothing`, `useAveraging`, `positionAverageWindow`, `rotationAverageWindow`, `outlierRejectDistance`) cover blink-on-loss, per-frame lerp, jitter from raw noise, and bad-sample rejection. They don't yet cover: jitter when a cube sits still, slow lerp tail when a cube is re-placed far away, "creeping-to-stop" feel of Lerp, hallucination spikes, or visual indication that a persisted cube is no longer live.

### Candidate knobs

| # | Field | Default proposal | What it solves | Code cost |
|---|---|---|---|---|
| 1 | `minMoveDistance` (float, m) | `0.002` | Sub-mm wobble when cube is at rest — moving-average alone doesn't kill it if samples drift coherently. Ignores pose updates that move the cube less than threshold. | ~5 lines, no new state |
| 2 | `snapDistanceThreshold` (float, m) | `0.15` (`0` = always lerp) | Slow lerp tail when a cube is re-placed across the table — currently takes ~½ s to settle. Bypasses lerp when the new pose is far from current. | ~5 lines, no new state |
| 3 | `useSmoothDamp` (bool) + `smoothTime` (float, s) | `false`, `0.15` | "Creeping to a stop" feel of Lerp — `Vector3.SmoothDamp` is a critical-damped spring that accelerates and decelerates naturally. | ~15 lines, 1 new `CubeState` field (per-cube velocity) |
| 4 | `maxSpeed` (float, m/s) | `0` (off) | Perception hallucination spikes — cube can't teleport to a bad sample; the next good sample overrides before it gets there. | ~5 lines, no new state |
| 5 | `staleFade` (float, s) | `5` | In persist mode, teammates can't tell if perception is still tracking or just persisting old data. Drops alpha to 50% after threshold. | ~10 lines, 1 new `CubeState` field (cached renderer), needs transparent material on the prefab |

### Recommended implementation order

1. **`minMoveDistance`** — small, isolated, biggest perceived smoothness win. Strong recommendation regardless.
2. **`snapDistanceThreshold`** — small, isolated, fixes the *opposite* corner case from #1. Worth pairing with #1.
3. **Pause + test** — combine (1) + (2) + existing `persistOnTagLoss`/`smoothing` against real cube movement. Decide whether #3 is still needed; the existing Lerp may now feel fine.
4. **`useSmoothDamp`** — only if step 3 isn't satisfying. Biggest change; benefits from going last after the rest is dialled in.
5. Skip **`maxSpeed`** and **`staleFade`** unless an actual symptom appears (hallucination spikes for #4, teammate confusion about freshness for #5).

### Interactions with existing knobs

| New knob | Touches | Resolution |
|---|---|---|
| `minMoveDistance` | `useAveraging`, `positionAverageWindow` | Apply deadband **after** averaging — the incoming raw sample still enters the moving-average buffer, but the *visual* cube doesn't move unless the averaged target shifted beyond threshold. |
| `snapDistanceThreshold` | `smoothing`, `useSmoothDamp` | In `Update()`, check distance to target before lerping. If beyond threshold, set position directly and bypass the lerp branch entirely. |
| `useSmoothDamp` | `smoothing` | Add a separate `smoothTime` field rather than overloading `smoothing` (cleaner Inspector). Ignore `smoothing` when `useSmoothDamp = true`; document in both tooltips. |
| `maxSpeed` | `smoothing`, `useSmoothDamp` | Clamp the per-frame visual displacement after the lerp/SmoothDamp step. Doesn't interfere with averaging. |
| `staleFade` | `persistOnTagLoss` | Pure additive — only mutates `MeshRenderer.material.color.a` during stale frames. No-op if persist is off. |

### Named preset recipes (post-implementation)

Once the chosen knobs land, document two Inspector configs as named recipes so a demoer can pick one without thinking:

**Demo (Snap):** for "show me where the cube is right now"
```
persistOnTagLoss     = true
smoothing            = 0
minMoveDistance      = 0.002      # 2 mm
snapDistanceThreshold = 0.15      # 15 cm
useAveraging         = true
positionAverageWindow = 3         # small, faster response
```

**Smooth Tracking:** for "show me the cube moving smoothly through space"
```
persistOnTagLoss     = true
smoothing            = 0.4        # or useSmoothDamp=true, smoothTime=0.15
minMoveDistance      = 0.001      # smaller, lerp absorbs more
snapDistanceThreshold = 0         # let everything lerp, even fast moves
useAveraging         = true
positionAverageWindow = 6         # default
```

### Test plan template (per knob)

Mirror the existing `persistOnTagLoss` test table structure:

- **`minMoveDistance`:** put a cube still on the table. Without the field, expect visible mm-scale wobble. With `0.002`, jitter freezes. Nudge ~1 cm → virtual should move. Nudge ~1 mm → virtual should not move.
- **`snapDistanceThreshold`:** with `smoothing = 0.5` and threshold = 0.15, move a cube slowly across 10 cm → smooth lerp. Move fast across 30 cm → snap, no visible lerp tail. Lift and place 50 cm away → snap.
- **`useSmoothDamp`:** A/B against Lerp at equivalent settings. Same end position, different feel — no "creeping to a stop" tail.
- **`maxSpeed`:** harder to test deliberately. Watch during normal use; if cube teleports were observed before, they shouldn't be now. Note in dev log when triggered.
- **`staleFade`:** cover a tag for 6 s with persist on — alpha should drop after 5 s. Uncover — alpha should restore on next pose.

### Documentation plan when each lands

For every knob implemented:
- New `### YYYY-MM-DD — <knob>` entry under `## Changes`
- Existing `How to test the toggle` matrix grows new columns for the new fields (or split into per-knob test sections if the matrix gets unwieldy)
- Remove the corresponding bullet from `Open items` or this `Proposed future knobs` section
- If both preset recipes are now expressible, append them as a final `## Preset recipes` section so demoers don't have to read the dev log to pick a config

### Implementation cost summary

| Scope | Total lines | New CubeState fields | Time estimate (incl. testing + dev log) |
|---|---|---|---|
| #1 + #2 only | ~10 | 0 | ~30 min |
| #1 + #2 + #3 | ~25 | 1 (velocity) | ~1 hour |
| All five | ~40 | 2 (velocity, renderer cache) | ~2 hours, plus prefab material work for #5 |

### Open design questions to resolve before building

- Should `minMoveDistance` apply to *received pose* or *averaged target*? The interaction table says averaged target, but if averaging is off (`useAveraging = false`), it should apply to the received pose. The field should "just work" either way — confirm in the implementation that both code paths route through the same deadband check.
- Default `snapDistanceThreshold` of 0.15 m is a guess based on typical workspace dimensions. May need tuning per setup. Consider exposing the field with `0` (off) as default to avoid surprise behaviour on existing scenes.
- For `staleFade`, what happens if the prefab uses a non-transparent material? Silent no-op (and log a warning once)? Or refuse to enable the toggle? Pick a policy before implementing.
- Decide: do we want a single `[Header("Smoothness Tuning (Advanced)")]` group in the Inspector for all the new fields, to keep the default Inspector view tidy? Most demoers won't touch these.

---

## Related: bin & sorting future considerations — early concept

> **Status: very early concept, nothing committed.** Seb asked whether adding a second/third bin is supported; current answer is "yes for drop-target purposes, but not for true per-bin sorting." This section captures what would change if we want differentiated bins later.

### Current state (as of 2026-05-26)

Multi-bin works out of the box for the *drop target / counter / panel display* side:

- [`BinDetector`](../Unity/My%20project/Assets/Scripts/BinDetector.cs) is per-GameObject. Duplicating a bin in the scene gives you a fully independent instance with its own correct/wrong counters.
- [`BinStatusPanel`](../Unity/My%20project/Assets/Scripts/BinStatusPanel.cs) auto-discovers all `BinDetector` instances via `FindObjectsByType` and renders one row per bin.
- [`TaskTracker.OnSort(bin, obj, isGood)`](../Unity/My%20project/Assets/Scripts/TaskTracker.cs) accepts the bin reference and resets all bins via `FindObjectsByType<BinDetector>`.
- `TaskConfigPanel` is bin-agnostic — it only configures completion mode + target count.

### What's missing

The "correct vs wrong" decision lives on the **cube** (`ObjectClass.isGood`), not on the bin. Today, dropping a `isGood = true` cube into *any* bin counts as correct; dropping a `isGood = false` cube into *any* bin counts as wrong. There's no concept of "Bin A accepts red, Bin B accepts blue, mixing counts as wrong."

So adding more bins today gets you more drop locations and more rows on the status panel, but no actual sorting semantics.

### Two ways to add real sorting (when we want it)

**Option A — `acceptedClasses[]` on BinDetector** (cheap, local)

Add an Inspector field on `BinDetector`:
```csharp
[Tooltip("Class names this bin accepts. Empty = accept all (legacy behaviour).")]
public string[] acceptedClasses;
```

Modify `HandleEntry`:
```csharp
bool classAccepted = acceptedClasses == null
                  || acceptedClasses.Length == 0
                  || System.Array.IndexOf(acceptedClasses, oc.className) >= 0;
bool isCorrect = oc.isGood && classAccepted;
```

~10 lines, backwards compatible (empty list preserves current behaviour). Each bin owns its own acceptance rules in the Inspector.

**Option B — Bin-class mapping centralised on TaskTracker** (more flexible, more code)

`TaskTracker` holds a `Dictionary<string binName, string[] acceptedClasses>`. Bins remain dumb; the mapping is queried per sort event. Better if the mapping needs to change at runtime (via `TaskConfigPanel`). Larger change.

### Open questions to resolve before building

- Does the planned demo actually need per-bin sorting, or are multiple bins purely visual (e.g., "drop any cube anywhere, counter goes up")? If the latter, no code changes needed — just duplicate bin GameObjects.
- If we want per-bin sorting, do the acceptance rules need to be runtime-editable (e.g., a configuration screen in the headset), or is Inspector-only fine? Inspector-only → Option A. Runtime-editable → Option B.
- How should "wrong bin but cube was isGood=true" be scored? Counted as wrong (penalty for misplacement), or just ignored? Mixing semantics may confuse demoers — write an explicit rule before building.

### When this lands

Same pattern as the smoothness knobs:
- New dated entry under `## Changes` summarising the change
- Update `BinStatusPanel` rendering to show acceptance rules per bin (optional UX)
- Cross-reference from `Open items` if any of the above questions are unresolved

---

## Topic / pipeline reference

- Perception publishes: `/holoassist/perception/april_cube_{1..4}_pose` (`geometry_msgs/PoseStamped`)
- Unity subscribes: `{topicPrefix}/cube_{N}_pose` — Inspector default `/holoassist/unity`
- Bridge: `ros2 run topic_tools relay /holoassist/perception/april_cube_N_pose /holoassist/unity/cube_N_pose` (one process per cube, started by hand for now)
- Frame: `camera_color_optical_frame` (no calibration loaded) or `base_link` (calibration loaded). Unity transforms via the `robotBase` Transform Inspector field regardless of the message frame_id.
