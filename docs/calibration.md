# Hand-Eye Calibration

The camera must be calibrated relative to the robot base so cube poses are expressed in robot frame.

Calibration is performed once and the result is saved to:

```
~/.ros2/easy_handeye2/calibrations/holoassist_calibration.calib
```

The launcher reads this file at startup and publishes the static TF automatically. **Calibration survives reboots** — no re-calibration needed each session unless the camera is physically moved.

---

## Running Calibration

```bash
cd ~/git/RS2-HoloAssist/main
./calibrate.sh
```

The calibration script uses [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2) to collect robot pose / camera pose pairs while the arm moves to pre-defined sample positions.

### Process

1. Attach a calibration target (AprilTag board) to the robot end-effector
2. Run `./calibrate.sh` — RViz opens with the calibration GUI
3. Move the arm to each sample pose (the GUI shows where to move)
4. Press **Take Sample** at each pose
5. After 12–15 samples: press **Compute** → check reprojection error (< 2 mm is good)
6. Press **Save** — writes `.calib` file

### Verification

After saving, restart the launcher and check the TF tree:

```bash
ros2 run tf2_tools view_frames
# Should show: world → base_link → camera_color_optical_frame
```

Or check the published transform:

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

---

## Calibration File Format

```yaml
# ~/.ros2/easy_handeye2/calibrations/holoassist_calibration.calib
transform:
  translation: {x: 0.xxx, y: 0.xxx, z: 0.xxx}
  rotation: {x: 0.xxx, y: 0.xxx, z: 0.xxx, w: 0.xxx}
parameters:
  tracking_base_frame: base_link
  tracking_marker_frame: camera_color_optical_frame
```

The launcher reads this at startup and publishes a static TF.

---

## Repeatable Without Re-calibration

Once calibrated, the system is reliable session-to-session as long as:

- The camera mount position relative to the robot has not changed
- The camera USB connection is to the same port (device index stays consistent)

If the camera is physically moved or remounted, re-run `./calibrate.sh`.

---

## Impact on Bin Verification

Without a valid calibration file:

- Cube poses are published in `camera_color_optical_frame` (not robot frame)
- Bin verification silently skips (logs a throttled warning, no crash)
- The bin check TF lookup `camera_color_optical_frame → base_link` fails

With calibration loaded, bin verification becomes active automatically.
