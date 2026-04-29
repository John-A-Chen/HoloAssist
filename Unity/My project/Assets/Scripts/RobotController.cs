using System;
using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class RobotController : MonoBehaviour
{
    public enum ControlMode { RMRC, DirectJoint, HandGuide }
    public enum RMRCSubMode { Translate, Rotate }

    [Header("Control Settings")]
    public ControlMode mode = ControlMode.RMRC;
    public float linearSpeed = 0.25f;    // m/s max Cartesian speed for RMRC
    public float angularSpeed = 0.3f;    // rad/s max Cartesian rotation for RMRC
    public RMRCSubMode rmrcSubMode = RMRCSubMode.Translate;
    public float jointJogSpeed = 0.5f;   // rad/s max for Direct Joint mode
    public float publishRate = 50f;      // Hz
    public float maxJointVelocity = 2.0f; // rad/s safety limit — all joints scaled proportionally

    [Header("Hand Guide")]
    [Tooltip("Robot root GameObject (e.g. 'ur'). Needed for Hand Guide mode coordinate transform.")]
    public Transform robotBase;
    [Tooltip("Proportional gain — higher = snappier tracking, lower = smoother.")]
    public float positionGain = 1f;
    [Tooltip("Maximum Cartesian speed in Hand Guide mode (m/s).")]
    public float maxTrackingSpeed = 0.09f;

    [Header("Gripper")]
    [Tooltip("Dead zone at trigger rest position. Below this value = fully open.")]
    public float gripperDeadZone = 0.05f;
    [Tooltip("Smoothing time for gripper input (seconds).")]
    public float gripperSmoothTime = 0.08f;

    [Header("Collision Protection")]
    [Tooltip("Table surface object. Uses its Y position as the collision boundary. If not set, uses tableWorldY.")]
    public Transform tableTransform;
    [Tooltip("World-space Y height of table surface. Used when tableTransform is not set.")]
    public float tableWorldY = 0f;
    [Tooltip("Minimum clearance (m) above the table surface.")]
    public float collisionMargin = 0.02f;
    [Tooltip("Distance (m) above the hard boundary where velocity is gradually reduced.")]
    public float collisionSoftZone = 0.08f;
    [Tooltip("Gripper length from tool0 to fingertip (m).")]
    public float gripperLength = 0.17f;
    [Tooltip("Minimum distance (m) between distal and proximal joints for self-collision.")]
    public float selfCollisionDistance = 0.08f;
    [Tooltip("Layer mask for obstacle colliders. Joints slow down near objects on these layers.")]
    public LayerMask obstacleLayer;
    [Tooltip("Radius (m) around each joint to scan for obstacle colliders.")]
    public float obstacleCheckRadius = 0.15f;

    [Header("EE Lock-Down")]
    [Tooltip("Gain for orientation correction when EE lock is active. Higher = snappier.")]
    public float lockOrientationGain = 0.5f;
    [Tooltip("Maximum angular speed (rad/s) for orientation correction.")]
    public float maxOrientationSpeed = 0.3f;
    [Tooltip("Orientation error below this angle (deg) is ignored to prevent jitter.")]
    public float lockDeadZoneDeg = 1f;

    [Header("Smoothing")]
    [Tooltip("Input smoothing time constant (seconds). Lower = more responsive, higher = smoother.")]
    public float inputSmoothTime = 0.12f;
    [Tooltip("Output velocity smoothing time constant (seconds). Smooths final joint velocities before publishing.")]
    public float outputSmoothTime = 0.06f;

    [Header("RMRC")]
    [Tooltip("Damping factor for singularity robustness. Higher = safer near singularities but less precise.")]
    public float damping = 0.01f;
    [Tooltip("Manipulability threshold — increases damping when near singularities.")]
    public float singularityThreshold = 0.01f;
    [Tooltip("Maximum damping applied near singularities.")]
    public float maxDamping = 0.5f;

    [Header("Input")]
    public InputActionAsset inputActions;

    // Public accessors for HUD
    public ControlMode CurrentMode => mode;
    public int SelectedJoint => selectedJoint;
    public string SelectedJointName => jointNames[selectedJoint];
    public bool HasJointState => hasReceivedJointState;
    public double[] CurrentPositions => currentPositions;
    public double[] VDesired => vDesired;
    public RMRCSubMode CurrentRMRCSubMode => rmrcSubMode;
    public bool IsHandGuideActive => isGripping;
    public float GripperValue => smoothedGripper;
    public bool IsEELockedDown => eeLockedDown;
    public Transform[] CollisionJoints => jointTransforms;
    public Transform Tool0 => tool0Transform;
    public float TableY => (tableTransform != null) ? tableTransform.position.y : tableWorldY;

    private ROSConnection ros;
    private int selectedJoint = 0;
    private double[] currentPositions = new double[6];
    private float publishTimer = 0f;
    private bool hasReceivedJointState = false;
    private float smoothedJointInput = 0f;
    private Vector2 smoothedLeft = Vector2.zero;
    private Vector2 smoothedRight = Vector2.zero;

    // Gripper state
    private float smoothedGripper = 0f;

    // EE Lock-Down state
    private bool eeLockedDown = false;

    // Collision protection — Unity transforms (auto-found at Start)
    private Transform[] jointTransforms;
    private Transform tool0Transform;
    private Vector3[] collisionCheckPoints = new Vector3[11];
    private Collider[] overlapBuffer = new Collider[8];
    private double[] toolZ = new double[3];
    private double[] wDesiredLock = new double[3];

    // Output smoothing
    private double[] smoothedQDot = new double[6];

    // Hand Guide state
    private bool isGripping = false;
    private Vector3 gripStartControllerLocal; // controller pos in robot base local frame at grip start
    private double[] gripStartEEPos = new double[3]; // EE pos in DH frame at grip start

    // Joint bias weights for Hand Guide — lower = less movement for that joint
    // Biases toward wrist joints so base/shoulder stay relatively still
    private static readonly double[] jointBias = { 0.5, 0.5, 0.7, 1.0, 1.0, 1.0 };

    // Input actions
    private InputAction leftStickAction;
    private InputAction rightStickAction;
    private InputAction nextJointAction;
    private InputAction prevJointAction;
    private InputAction toggleModeAction;
    private InputAction toggleSubModeAction;
    private InputAction gripAction;
    private InputAction controllerPosAction;
    private InputAction gripperTriggerAction;
    private InputAction lockDownAction;

    // Reusable arrays to avoid GC
    private double[] vDesired = new double[3];
    private double[] wDesired = new double[3];
    private double[] qDot = new double[6];

    private static readonly string[] jointNames =
    {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    // Joint limits from URDF (radians)
    private static readonly double[] jointLimitsLower = { -6.2832, -6.2832, -3.1416, -6.2832, -6.2832, -6.2832 };
    private static readonly double[] jointLimitsUpper = {  6.2832,  6.2832,  3.1416,  6.2832,  6.2832,  6.2832 };

    private const string VELOCITY_TOPIC = "/forward_velocity_controller/commands";
    private const string GRIPPER_TOPIC = "/finger_width_controller/commands";
    private const float RG2_MAX_WIDTH = 0.11f; // metres

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float64MultiArrayMsg>(VELOCITY_TOPIC);
        ros.RegisterPublisher<Float64MultiArrayMsg>(GRIPPER_TOPIC);
        ros.Subscribe<JointStateMsg>("/joint_states", OnJointState);

        SetupInputActions();
        DisableConflictingXRIActions();
        FindCollisionTransforms();
    }

    void SetupInputActions()
    {
        if (inputActions == null)
        {
            Debug.LogError("[RobotController] InputActionAsset not assigned! Drag RobotControlActions into the Inspector.");
            return;
        }

        var robotMap = inputActions.FindActionMap("Robot", true);

        leftStickAction = robotMap.FindAction("LeftStick", true);
        rightStickAction = robotMap.FindAction("RightStick", true);
        nextJointAction = robotMap.FindAction("NextJoint", true);
        prevJointAction = robotMap.FindAction("PrevJoint", true);
        toggleModeAction = robotMap.FindAction("ToggleMode", true);

        robotMap.Enable();

        // X button (left controller) — toggle Translate/Rotate sub-mode in RMRC
        toggleSubModeAction = new InputAction("ToggleSubMode", InputActionType.Button, "<XRController>{LeftHand}/primaryButton");
        toggleSubModeAction.Enable();

        // Right grip trigger — Hand Guide engage
        gripAction = new InputAction("Grip", InputActionType.Value, "<XRController>{RightHand}/grip");
        gripAction.Enable();

        // Right controller position in world space
        controllerPosAction = new InputAction("ControllerPos", InputActionType.Value, "<XRController>{RightHand}/devicePosition");
        controllerPosAction.Enable();

        // Right index trigger — analog gripper control
        gripperTriggerAction = new InputAction("GripperTrigger", InputActionType.Value, "<XRController>{RightHand}/trigger");
        gripperTriggerAction.Enable();

        // Y button (left controller) — toggle EE lock-down
        lockDownAction = new InputAction("LockDown", InputActionType.Button, "<XRController>{LeftHand}/secondaryButton");
        lockDownAction.Enable();
    }

    void DisableConflictingXRIActions()
    {
        string[] conflictingMaps = {
            "XRI Left Locomotion",
            "XRI Right Locomotion"
        };

        foreach (var asset in Resources.FindObjectsOfTypeAll<InputActionAsset>())
        {
            if (asset == inputActions) continue;

            foreach (string mapName in conflictingMaps)
            {
                var map = asset.FindActionMap(mapName);
                if (map != null)
                {
                    map.Disable();
                    Debug.Log($"[RobotController] Disabled conflicting action map: {asset.name}/{mapName}");
                }
            }

            var rightLocomotion = asset.FindActionMap("XRI Right Locomotion");
            if (rightLocomotion != null)
            {
                var jump = rightLocomotion.FindAction("Jump");
                if (jump != null) jump.Disable();
            }

            var interaction = asset.FindActionMap("XRI Left Interaction");
            if (interaction != null)
            {
                var uiScroll = interaction.FindAction("UI Scroll Value");
                if (uiScroll != null) uiScroll.Disable();
                var rotateManip = interaction.FindAction("Rotate Manipulation");
                if (rotateManip != null) rotateManip.Disable();
            }

            var rightInteraction = asset.FindActionMap("XRI Right Interaction");
            if (rightInteraction != null)
            {
                var rotateManip = rightInteraction.FindAction("Rotate Manipulation");
                if (rotateManip != null) rotateManip.Disable();
            }
        }
    }

    void OnJointState(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            for (int j = 0; j < jointNames.Length; j++)
            {
                if (msg.name[i] == jointNames[j])
                    currentPositions[j] = msg.position[i];
            }
        }
        hasReceivedJointState = true;
    }

    void Update()
    {
        if (inputActions == null) return;

        HandleModeSwitch();

        publishTimer += Time.deltaTime;
        if (publishTimer < 1f / publishRate) return;
        publishTimer = 0f;

        UpdateGripper();

        if (mode == ControlMode.RMRC)
            UpdateRMRC();
        else if (mode == ControlMode.DirectJoint)
            UpdateDirectJoint();
        else
            UpdateHandGuide();
    }

    void HandleModeSwitch()
    {
        if (toggleModeAction != null && toggleModeAction.WasPressedThisFrame())
        {
            // Cycle: RMRC → DirectJoint → HandGuide → RMRC
            if (mode == ControlMode.RMRC)
                mode = ControlMode.DirectJoint;
            else if (mode == ControlMode.DirectJoint)
                mode = ControlMode.HandGuide;
            else
                mode = ControlMode.RMRC;

            rmrcSubMode = RMRCSubMode.Translate;
            smoothedJointInput = 0f;
            smoothedLeft = Vector2.zero;
            smoothedRight = Vector2.zero;
            isGripping = false;
            for (int i = 0; i < 6; i++) smoothedQDot[i] = 0;
            Debug.Log($"[RobotController] Mode: {mode}");
        }

        if (mode == ControlMode.RMRC && toggleSubModeAction != null && toggleSubModeAction.WasPressedThisFrame())
        {
            rmrcSubMode = (rmrcSubMode == RMRCSubMode.Translate) ? RMRCSubMode.Rotate : RMRCSubMode.Translate;
            smoothedLeft = Vector2.zero;
            smoothedRight = Vector2.zero;
            Debug.Log($"[RobotController] RMRC sub-mode: {rmrcSubMode}");
        }

        if (lockDownAction != null && lockDownAction.WasPressedThisFrame())
        {
            eeLockedDown = !eeLockedDown;
            Debug.Log($"[RobotController] EE Lock-Down: {(eeLockedDown ? "ON" : "OFF")}");
        }

        if (mode == ControlMode.DirectJoint)
        {
            if (nextJointAction != null && nextJointAction.WasPressedThisFrame())
            {
                smoothedJointInput = 0f;
                selectedJoint = (selectedJoint + 1) % 6;
                Debug.Log($"[RobotController] Selected: {jointNames[selectedJoint]}");
            }
            if (prevJointAction != null && prevJointAction.WasPressedThisFrame())
            {
                smoothedJointInput = 0f;
                selectedJoint = (selectedJoint + 5) % 6;
                Debug.Log($"[RobotController] Selected: {jointNames[selectedJoint]}");
            }
        }
    }

    float SmoothExp(float current, float target, float smoothTime, float dt)
    {
        if (smoothTime <= 0f) return target;
        return Mathf.Lerp(current, target, 1f - Mathf.Exp(-dt / smoothTime));
    }

    void UpdateRMRC()
    {
        if (!hasReceivedJointState) return;

        Vector2 leftStick = leftStickAction != null ? leftStickAction.ReadValue<Vector2>() : Vector2.zero;
        Vector2 rightStick = rightStickAction != null ? rightStickAction.ReadValue<Vector2>() : Vector2.zero;

        float dt = 1f / publishRate;

        // Smooth stick inputs
        smoothedLeft.x = SmoothExp(smoothedLeft.x, leftStick.x, inputSmoothTime, dt);
        smoothedLeft.y = SmoothExp(smoothedLeft.y, leftStick.y, inputSmoothTime, dt);
        smoothedRight.x = SmoothExp(smoothedRight.x, rightStick.x, inputSmoothTime, dt);
        smoothedRight.y = SmoothExp(smoothedRight.y, rightStick.y, inputSmoothTime, dt);

        // Kill small residuals when stick is released
        if (Mathf.Abs(smoothedLeft.x) < 0.005f && Mathf.Abs(leftStick.x) < 0.01f) smoothedLeft.x = 0f;
        if (Mathf.Abs(smoothedLeft.y) < 0.005f && Mathf.Abs(leftStick.y) < 0.01f) smoothedLeft.y = 0f;
        if (Mathf.Abs(smoothedRight.x) < 0.005f && Mathf.Abs(rightStick.x) < 0.01f) smoothedRight.x = 0f;
        if (Mathf.Abs(smoothedRight.y) < 0.005f && Mathf.Abs(rightStick.y) < 0.01f) smoothedRight.y = 0f;

        // Adaptive damping near singularities
        double manip = UR3eKinematics.Manipulability(currentPositions);
        float effectiveDamping = damping;
        if (manip < singularityThreshold && singularityThreshold > 0)
        {
            float ratio = 1f - (float)(manip / singularityThreshold);
            effectiveDamping = Mathf.Lerp(damping, maxDamping, ratio);
        }

        if (rmrcSubMode == RMRCSubMode.Translate)
        {
            vDesired[0] = smoothedRight.y * linearSpeed;   // vx - forward/back
            vDesired[1] = smoothedRight.x * linearSpeed;   // vy - left/right
            vDesired[2] = smoothedLeft.y * linearSpeed;    // vz - up/down

            if (eeLockedDown)
            {
                ComputeLockAngularVelocity();
                UR3eKinematics.ResolveFullVelocity(currentPositions, vDesired, wDesiredLock, qDot, effectiveDamping);
            }
            else
            {
                UR3eKinematics.ResolveLinearVelocity(currentPositions, vDesired, qDot, effectiveDamping);
                qDot[0] += smoothedLeft.x * angularSpeed;
            }
        }
        else
        {
            vDesired[0] = 0; vDesired[1] = 0; vDesired[2] = 0;

            for (int i = 0; i < 6; i++) qDot[i] = 0;
            qDot[3] = smoothedRight.y * jointJogSpeed;  // wrist_1
            qDot[4] = smoothedRight.x * jointJogSpeed;  // wrist_2
            qDot[5] = smoothedLeft.x * jointJogSpeed;   // wrist_3
        }

        ApplyJointSafety();
        ApplyCollisionProtection();
        ApplyOutputSmoothing();

        // Publish
        var msg = new Float64MultiArrayMsg();
        msg.data = new double[6];
        Array.Copy(qDot, msg.data, 6);
        ros.Publish(VELOCITY_TOPIC, msg);
    }

    void UpdateDirectJoint()
    {
        if (!hasReceivedJointState) return;

        Vector2 rightStick = rightStickAction != null ? rightStickAction.ReadValue<Vector2>() : Vector2.zero;

        float dt = 1f / publishRate;

        smoothedJointInput = SmoothExp(smoothedJointInput, rightStick.y, inputSmoothTime, dt);
        if (Mathf.Abs(smoothedJointInput) < 0.01f && Mathf.Abs(rightStick.y) < 0.01f)
            smoothedJointInput = 0f;

        for (int i = 0; i < 6; i++) qDot[i] = 0;
        qDot[selectedJoint] = smoothedJointInput * jointJogSpeed;

        ApplyCollisionProtection();
        ApplyOutputSmoothing();

        var msg = new Float64MultiArrayMsg();
        msg.data = new double[6];
        Array.Copy(qDot, msg.data, 6);
        ros.Publish(VELOCITY_TOPIC, msg);
    }

    void UpdateHandGuide()
    {
        if (!hasReceivedJointState) return;

        if (robotBase == null)
        {
            Debug.LogWarning("[RobotController] HandGuide: robotBase not assigned! Set it to the robot root (e.g. 'ur').");
            PublishZero();
            return;
        }

        float grip = gripAction != null ? gripAction.ReadValue<float>() : 0f;
        Vector3 controllerWorldPos = controllerPosAction != null ? controllerPosAction.ReadValue<Vector3>() : Vector3.zero;

        // Transform controller position into robot base local frame
        Vector3 controllerLocalPos = robotBase.InverseTransformPoint(controllerWorldPos);

        // Grip engage/disengage with hysteresis
        if (grip > 0.5f && !isGripping)
        {
            isGripping = true;
            gripStartControllerLocal = controllerLocalPos;

            // Record current EE position in DH frame
            UR3eKinematics.GetPosition(currentPositions, gripStartEEPos);
            Debug.Log("[RobotController] HandGuide: ENGAGED");
        }
        else if (grip < 0.3f && isGripping)
        {
            isGripping = false;
            Debug.Log("[RobotController] HandGuide: RELEASED");
        }

        if (isGripping)
        {
            Vector3 localDelta = controllerLocalPos - gripStartControllerLocal;

            double targetX = gripStartEEPos[0] - localDelta.z;
            double targetY = gripStartEEPos[1] + localDelta.x;
            double targetZ = gripStartEEPos[2] + localDelta.y;

            var currentEE = new double[3];
            UR3eKinematics.GetPosition(currentPositions, currentEE);

            vDesired[0] = (targetX - currentEE[0]) * positionGain;
            vDesired[1] = (targetY - currentEE[1]) * positionGain;
            vDesired[2] = (targetZ - currentEE[2]) * positionGain;

            double speed = System.Math.Sqrt(vDesired[0] * vDesired[0] + vDesired[1] * vDesired[1] + vDesired[2] * vDesired[2]);
            if (speed > maxTrackingSpeed)
            {
                double s = maxTrackingSpeed / speed;
                vDesired[0] *= s;
                vDesired[1] *= s;
                vDesired[2] *= s;
            }

            double manip = UR3eKinematics.Manipulability(currentPositions);
            float effectiveDamping = damping;
            if (manip < singularityThreshold && singularityThreshold > 0)
            {
                float ratio = 1f - (float)(manip / singularityThreshold);
                effectiveDamping = Mathf.Lerp(damping, maxDamping, ratio);
            }

            if (eeLockedDown)
            {
                ComputeLockAngularVelocity();
                UR3eKinematics.ResolveFullVelocity(currentPositions, vDesired, wDesiredLock, qDot, effectiveDamping);
            }
            else
            {
                UR3eKinematics.ResolveLinearVelocity(currentPositions, vDesired, qDot, effectiveDamping);
            }

            for (int i = 0; i < 6; i++)
                qDot[i] *= jointBias[i];
        }
        else if (eeLockedDown)
        {
            // Not gripping but locked — hold orientation with zero linear velocity
            vDesired[0] = 0; vDesired[1] = 0; vDesired[2] = 0;
            ComputeLockAngularVelocity();
            UR3eKinematics.ResolveFullVelocity(currentPositions, vDesired, wDesiredLock, qDot, damping);
        }
        else
        {
            PublishZero();
            return;
        }

        ApplyJointSafety();
        ApplyCollisionProtection();
        ApplyOutputSmoothing();

        var msg = new Float64MultiArrayMsg();
        msg.data = new double[6];
        Array.Copy(qDot, msg.data, 6);
        ros.Publish(VELOCITY_TOPIC, msg);
    }

    void UpdateGripper()
    {
        float raw = gripperTriggerAction != null ? gripperTriggerAction.ReadValue<float>() : 0f;

        // Apply dead zone — resting finger = fully open
        float mapped = (raw <= gripperDeadZone) ? 0f : (raw - gripperDeadZone) / (1f - gripperDeadZone);

        float dt = 1f / publishRate;
        smoothedGripper = SmoothExp(smoothedGripper, mapped, gripperSmoothTime, dt);
        if (smoothedGripper < 0.005f && mapped < 0.01f) smoothedGripper = 0f;

        // finger_width_controller expects position in metres: 0 = closed, RG2_MAX_WIDTH = open
        double widthMetres = (1f - smoothedGripper) * RG2_MAX_WIDTH;
        var gripMsg = new Float64MultiArrayMsg();
        gripMsg.data = new double[] { widthMetres };
        ros.Publish(GRIPPER_TOPIC, gripMsg);
    }

    void PublishZero()
    {
        for (int i = 0; i < 6; i++) smoothedQDot[i] = 0;
        var msg = new Float64MultiArrayMsg();
        msg.data = new double[6];
        ros.Publish(VELOCITY_TOPIC, msg);
    }

    void ApplyJointSafety()
    {
        // Joint limit protection
        for (int i = 0; i < 6; i++)
        {
            double pos = currentPositions[i];
            double margin = 0.05;
            if (pos <= jointLimitsLower[i] + margin && qDot[i] < 0) qDot[i] = 0;
            if (pos >= jointLimitsUpper[i] - margin && qDot[i] > 0) qDot[i] = 0;
        }

        // Proportional velocity scaling
        double maxAbs = 0;
        for (int i = 0; i < 6; i++)
            maxAbs = Math.Max(maxAbs, Math.Abs(qDot[i]));
        if (maxAbs > maxJointVelocity)
        {
            double scale = maxJointVelocity / maxAbs;
            for (int i = 0; i < 6; i++)
                qDot[i] *= scale;
        }
    }

    void ComputeLockAngularVelocity()
    {
        UR3eKinematics.GetToolZAxis(currentPositions, toolZ);

        double dot = -toolZ[2]; // dot(toolZ, [0,0,-1])
        dot = System.Math.Max(-1.0, System.Math.Min(1.0, dot));
        double angle = System.Math.Acos(dot);

        if (angle < lockDeadZoneDeg * Mathf.Deg2Rad)
        {
            wDesiredLock[0] = 0; wDesiredLock[1] = 0; wDesiredLock[2] = 0;
            return;
        }

        double cx = -toolZ[1];
        double cy = toolZ[0];
        double crossMag = System.Math.Sqrt(cx * cx + cy * cy);

        if (crossMag < 1e-6)
        {
            wDesiredLock[0] = 0; wDesiredLock[1] = 0; wDesiredLock[2] = 0;
            return;
        }

        cx /= crossMag;
        cy /= crossMag;

        double errorAboveDeadZone = angle - lockDeadZoneDeg * Mathf.Deg2Rad;
        double speed = System.Math.Min(errorAboveDeadZone * lockOrientationGain, maxOrientationSpeed);

        wDesiredLock[0] = cx * speed;
        wDesiredLock[1] = cy * speed;
        wDesiredLock[2] = 0;
    }

    void FindCollisionTransforms()
    {
        if (robotBase == null) return;

        string[] linkNames = { "shoulder_link", "upper_arm_link", "forearm_link",
                               "wrist_1_link", "wrist_2_link", "wrist_3_link" };
        jointTransforms = new Transform[6];

        foreach (var t in robotBase.GetComponentsInChildren<Transform>(true))
        {
            for (int i = 0; i < linkNames.Length; i++)
                if (t.name == linkNames[i])
                    jointTransforms[i] = t;
            if (t.name == "tool0")
                tool0Transform = t;
        }

        int found = 0;
        for (int i = 0; i < 6; i++) if (jointTransforms[i] != null) found++;
        Debug.Log($"[RobotController] Collision transforms: {found}/6 joints, tool0={tool0Transform != null}");
    }

    void ApplyCollisionProtection()
    {
        if (jointTransforms == null) return;

        float tableY = (tableTransform != null) ? tableTransform.position.y : tableWorldY;

        // Gather collision check points (no allocation — pre-sized array)
        int n = 0;
        for (int i = 0; i < jointTransforms.Length; i++)
        {
            if (jointTransforms[i] != null)
                collisionCheckPoints[n++] = jointTransforms[i].position;
        }

        // Midpoints of long links (upper arm: joints 0→1, forearm: joints 1→2)
        if (jointTransforms[0] != null && jointTransforms[1] != null)
            collisionCheckPoints[n++] = (jointTransforms[0].position + jointTransforms[1].position) * 0.5f;
        if (jointTransforms[1] != null && jointTransforms[2] != null)
            collisionCheckPoints[n++] = (jointTransforms[1].position + jointTransforms[2].position) * 0.5f;

        // Tool0, gripper tip, and gripper midpoint
        if (tool0Transform != null)
        {
            collisionCheckPoints[n++] = tool0Transform.position;
            // tool0.up = DH Z axis = tool direction (URDF importer maps DH Z → Unity Y)
            Vector3 gripDir = tool0Transform.up;
            collisionCheckPoints[n++] = tool0Transform.position + gripDir * gripperLength;
            collisionCheckPoints[n++] = tool0Transform.position + gripDir * gripperLength * 0.5f;
        }

        // --- Table collision ---
        float minTableDist = float.MaxValue;
        for (int i = 0; i < n; i++)
        {
            float dist = collisionCheckPoints[i].y - tableY;
            if (dist < minTableDist) minTableDist = dist;
        }

        if (minTableDist <= collisionMargin)
        {
            for (int i = 0; i < 6; i++) qDot[i] = 0;
            return;
        }
        if (minTableDist < collisionMargin + collisionSoftZone)
        {
            float scale = (minTableDist - collisionMargin) / collisionSoftZone;
            for (int i = 0; i < 6; i++) qDot[i] *= scale;
        }

        // --- Object collision (layer-based) ---
        if (obstacleLayer.value != 0)
        {
            float minObjDist = float.MaxValue;
            for (int i = 0; i < n; i++)
            {
                int count = Physics.OverlapSphereNonAlloc(
                    collisionCheckPoints[i], obstacleCheckRadius, overlapBuffer, obstacleLayer);
                for (int c = 0; c < count; c++)
                {
                    Vector3 closest = overlapBuffer[c].ClosestPoint(collisionCheckPoints[i]);
                    float dist = Vector3.Distance(collisionCheckPoints[i], closest);
                    if (dist < minObjDist) minObjDist = dist;
                }
            }

            if (minObjDist <= collisionMargin)
            {
                for (int i = 0; i < 6; i++) qDot[i] = 0;
                return;
            }
            if (minObjDist < collisionMargin + collisionSoftZone)
            {
                float scale = (minObjDist - collisionMargin) / collisionSoftZone;
                for (int i = 0; i < 6; i++) qDot[i] *= scale;
            }
        }

        // --- Self-collision: distal joints (wrist) vs proximal (shoulder/upper arm) ---
        float minSelfDist = float.MaxValue;
        for (int far = 3; far < 6; far++)
        {
            if (jointTransforms[far] == null) continue;
            for (int near = 0; near < 2; near++)
            {
                if (jointTransforms[near] == null) continue;
                float dist = Vector3.Distance(jointTransforms[far].position, jointTransforms[near].position);
                if (dist < minSelfDist) minSelfDist = dist;
            }
        }
        if (tool0Transform != null)
        {
            for (int near = 0; near < 2; near++)
            {
                if (jointTransforms[near] == null) continue;
                float dist = Vector3.Distance(tool0Transform.position, jointTransforms[near].position);
                if (dist < minSelfDist) minSelfDist = dist;
            }
        }

        if (minSelfDist <= selfCollisionDistance)
        {
            for (int i = 0; i < 6; i++) qDot[i] = 0;
        }
        else if (minSelfDist < selfCollisionDistance + collisionSoftZone)
        {
            float scale = (minSelfDist - selfCollisionDistance) / collisionSoftZone;
            for (int i = 0; i < 6; i++) qDot[i] *= scale;
        }
    }

    void ApplyOutputSmoothing()
    {
        float dt = 1f / publishRate;
        float alpha = 1f - Mathf.Exp(-dt / outputSmoothTime);
        for (int i = 0; i < 6; i++)
        {
            smoothedQDot[i] += (qDot[i] - smoothedQDot[i]) * alpha;
            qDot[i] = smoothedQDot[i];
        }
    }

    void OnDestroy()
    {
        if (ros != null)
        {
            var stop = new Float64MultiArrayMsg();
            stop.data = new double[6];
            ros.Publish(VELOCITY_TOPIC, stop);
        }

        if (inputActions != null)
        {
            var robotMap = inputActions.FindActionMap("Robot");
            if (robotMap != null) robotMap.Disable();
        }

        if (toggleSubModeAction != null)
        {
            toggleSubModeAction.Disable();
            toggleSubModeAction.Dispose();
        }

        if (gripAction != null)
        {
            gripAction.Disable();
            gripAction.Dispose();
        }

        if (controllerPosAction != null)
        {
            controllerPosAction.Disable();
            controllerPosAction.Dispose();
        }

        if (gripperTriggerAction != null)
        {
            gripperTriggerAction.Disable();
            gripperTriggerAction.Dispose();
        }

        if (lockDownAction != null)
        {
            lockDownAction.Disable();
            lockDownAction.Dispose();
        }
    }
}
