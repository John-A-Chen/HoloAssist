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
    public float positionGain = 2f;
    [Tooltip("Maximum Cartesian speed in Hand Guide mode (m/s).")]
    public float maxTrackingSpeed = 0.15f;

    [Header("Smoothing")]
    [Tooltip("Input smoothing time constant (seconds). Lower = more responsive, higher = smoother.")]
    public float inputSmoothTime = 0.12f;

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

    private ROSConnection ros;
    private int selectedJoint = 0;
    private double[] currentPositions = new double[6];
    private float publishTimer = 0f;
    private bool hasReceivedJointState = false;
    private float smoothedJointInput = 0f;
    private Vector2 smoothedLeft = Vector2.zero;
    private Vector2 smoothedRight = Vector2.zero;

    // Hand Guide state
    private bool isGripping = false;
    private Vector3 gripStartControllerLocal; // controller pos in robot base local frame at grip start
    private double[] gripStartEEPos = new double[3]; // EE pos in DH frame at grip start

    // Joint bias weights for Hand Guide — lower = less movement for that joint
    // Biases toward wrist joints so base/shoulder stay relatively still
    private static readonly double[] jointBias = { 0.3, 0.3, 0.5, 1.0, 1.0, 1.0 };

    // Input actions
    private InputAction leftStickAction;
    private InputAction rightStickAction;
    private InputAction nextJointAction;
    private InputAction prevJointAction;
    private InputAction toggleModeAction;
    private InputAction toggleSubModeAction;
    private InputAction gripAction;
    private InputAction controllerPosAction;

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

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float64MultiArrayMsg>(VELOCITY_TOPIC);
        ros.Subscribe<JointStateMsg>("/joint_states", OnJointState);

        SetupInputActions();
        DisableConflictingXRIActions();
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
            Debug.Log($"[RobotController] Mode: {mode}");
        }

        if (mode == ControlMode.RMRC && toggleSubModeAction != null && toggleSubModeAction.WasPressedThisFrame())
        {
            rmrcSubMode = (rmrcSubMode == RMRCSubMode.Translate) ? RMRCSubMode.Rotate : RMRCSubMode.Translate;
            smoothedLeft = Vector2.zero;
            smoothedRight = Vector2.zero;
            Debug.Log($"[RobotController] RMRC sub-mode: {rmrcSubMode}");
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
            // Build desired linear velocity in robot base frame
            // Right stick: X (forward/back), Y (left/right)
            // Left stick Y: Z (up/down)
            vDesired[0] = smoothedRight.y * linearSpeed;   // vx - forward/back
            vDesired[1] = smoothedRight.x * linearSpeed;   // vy - left/right
            vDesired[2] = smoothedLeft.y * linearSpeed;    // vz - up/down

            // Resolve linear velocity to joint velocities via 3x6 Jacobian pseudoinverse
            UR3eKinematics.ResolveLinearVelocity(currentPositions, vDesired, qDot, effectiveDamping);

            // Add yaw (rotation about base Z) directly to joint 0 (shoulder_pan)
            qDot[0] += smoothedLeft.x * angularSpeed;
        }
        else
        {
            // Rotate sub-mode: directly drive wrist joints for responsive orientation control
            // Right stick Y = wrist_1 (joint 3) — pitch/tilt
            // Right stick X = wrist_2 (joint 4) — roll
            // Left stick X  = wrist_3 (joint 5) — yaw/spin

            // Clear linear velocity display
            vDesired[0] = 0; vDesired[1] = 0; vDesired[2] = 0;

            for (int i = 0; i < 6; i++) qDot[i] = 0;
            qDot[3] = smoothedRight.y * jointJogSpeed;  // wrist_1
            qDot[4] = smoothedRight.x * jointJogSpeed;  // wrist_2
            qDot[5] = smoothedLeft.x * jointJogSpeed;   // wrist_3
        }

        ApplyJointSafety();

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

        var msg = new Float64MultiArrayMsg();
        msg.data = new double[6];
        msg.data[selectedJoint] = smoothedJointInput * jointJogSpeed;

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

        if (!isGripping)
        {
            PublishZero();
            return;
        }

        // Controller delta in robot base local frame (Unity convention: X-right, Y-up, Z-forward)
        Vector3 localDelta = controllerLocalPos - gripStartControllerLocal;

        // Convert Unity local frame → DH frame (Z-up, X-forward)
        // Axis mapping verified on Quest 3: up/down correct, forward/back and left/right were inverted
        double targetX = gripStartEEPos[0] - localDelta.z;
        double targetY = gripStartEEPos[1] + localDelta.x;
        double targetZ = gripStartEEPos[2] + localDelta.y;

        // Current EE position from FK
        var currentEE = new double[3];
        UR3eKinematics.GetPosition(currentPositions, currentEE);

        // Proportional position control → desired Cartesian velocity
        vDesired[0] = (targetX - currentEE[0]) * positionGain;
        vDesired[1] = (targetY - currentEE[1]) * positionGain;
        vDesired[2] = (targetZ - currentEE[2]) * positionGain;

        // Clamp Cartesian speed
        double speed = System.Math.Sqrt(vDesired[0] * vDesired[0] + vDesired[1] * vDesired[1] + vDesired[2] * vDesired[2]);
        if (speed > maxTrackingSpeed)
        {
            double s = maxTrackingSpeed / speed;
            vDesired[0] *= s;
            vDesired[1] *= s;
            vDesired[2] *= s;
        }

        // Adaptive damping near singularities
        double manip = UR3eKinematics.Manipulability(currentPositions);
        float effectiveDamping = damping;
        if (manip < singularityThreshold && singularityThreshold > 0)
        {
            float ratio = 1f - (float)(manip / singularityThreshold);
            effectiveDamping = Mathf.Lerp(damping, maxDamping, ratio);
        }

        // Resolve to joint velocities via Jacobian pseudoinverse
        UR3eKinematics.ResolveLinearVelocity(currentPositions, vDesired, qDot, effectiveDamping);

        // Bias toward wrist joints — scale down base/shoulder velocities
        for (int i = 0; i < 6; i++)
            qDot[i] *= jointBias[i];

        ApplyJointSafety();

        var msg = new Float64MultiArrayMsg();
        msg.data = new double[6];
        Array.Copy(qDot, msg.data, 6);
        ros.Publish(VELOCITY_TOPIC, msg);
    }

    void PublishZero()
    {
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
    }
}
