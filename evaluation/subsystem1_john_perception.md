# Subsystem 1 — Perception

**Lead:** John

## Scope

Object detection, identification, and spatial tracking using the Intel RealSense depth camera. Provides the perception pipeline that feeds both the teleoperation and autonomous sorting modes.

---

## Pass (P)

The Intel RealSense depth camera is configured and publishing colour and depth data to ROS 2. The camera feed is stable and viewable in RViz.

**Criteria:**
- RealSense ROS 2 wrapper installed and launching reliably
- Colour and depth image topics publishing at expected rates
- Point cloud topic available
- Camera feed verified in RViz

---

## Credit (C)

Objects entering the workspace are detected using the depth camera. Basic object segmentation separates objects from the workspace background.

**Criteria:**
- Background subtraction or depth thresholding isolates objects on the workspace surface
- Detected object regions are identifiable (bounding box or centroid)
- Detection works for predefined objects placed on the workspace

---

## Distinction (D)

Detected objects are classified by type (colour, shape, or computer vision) and their pose is published to ROS in the robot's coordinate frame. Camera-to-robot calibration is functional using QR codes.

**Criteria:**
- Object classification distinguishes between the predefined object set (e.g., by colour or shape)
- QR code (or April tag) based calibration transforms camera-frame detections into robot-frame poses
- Object type and pose published as ROS messages usable by both Unity (via ros_tcp_endpoint) and the autonomous sorting node
- Calibration is repeatable and does not require manual tuning each session

---

## High Distinction (HD)

Objects are tracked in real-time as they enter and move through the workspace. The depth camera confirms whether a sorted object has been placed in the correct bin.

**Criteria:**
- Continuous tracking of detected objects (not just single-frame detection) — objects are tracked from entry until manipulation
- Object identity persists across frames (same object is not re-detected as a new one)
- After a sort operation, the camera verifies the object is in the expected bin and publishes a success/failure confirmation
- System handles multiple objects in the workspace simultaneously
