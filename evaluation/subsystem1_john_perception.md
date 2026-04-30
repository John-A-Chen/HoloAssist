# Subsystem 1 — Perception

**Lead:** John Chen (24837332)

## Scope

Object detection, identification, and spatial tracking using the Intel RealSense depth camera. Provides the perception pipeline that feeds both the teleoperation and autonomous sorting modes.

---

## Pass (P) — Camera publishing in ROS

- RealSense ROS 2 wrapper installed and launching reliably
- Colour and depth image topics publishing at expected rates
- Point cloud topic available
- Camera feed verified in RViz

---

## Credit (C) — Object detection and segmentation

- Background subtraction or depth thresholding isolates objects on the workspace surface
- Detected object regions are identifiable (bounding box or centroid)
- Detection works for predefined objects placed on the workspace

---

## Distinction (D) — Classification and pose in robot frame

- Object classification distinguishes between the predefined object set (e.g., by colour or shape)
- QR code / AprilTag calibration transforms camera-frame detections into robot-frame poses
- Object type and pose published as ROS messages usable by Unity and the autonomous sorting node
- Calibration is repeatable and does not require manual tuning each session

---

## High Distinction (HD) — Real-time tracking and bin verification

- Continuous tracking of detected objects — objects tracked from entry until manipulation
- Object identity persists across frames (same object is not re-detected as new)
- After a sort, camera verifies the object is in the expected bin and publishes success/failure confirmation
- System handles multiple objects in the workspace simultaneously
