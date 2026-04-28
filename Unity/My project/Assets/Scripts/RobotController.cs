using System;
using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class RobotController : MonoBehaviour
{
    public enum ControlMode { Servo, DirectJoint }

    [Header("Control Settings")]
    public ControlMode mode = ControlMode.DirectJoint;
    public float linearSpeed = 0.3f;   // m/s for Servo mode
    public float angularSpeed = 0.3f;  // rad/s for Servo mode
    public float jointJogSpeed = 0.5f; // rad/s max for Direct Joint mode
    public float publishRate = 50f;    // Hz

    [Header("Smoothing")]
    [Tooltip("Input smoothing time constant (seconds). Lower = more responsive, higher = smoother.")]
    public float inputSmoothTime = 0.12f;

    [Header("Input")]
    public InputActionAsset inputActions;

    // Public accessors for HUD
    public ControlMode CurrentMode => mode;
    public int SelectedJoint => selectedJoint;
    public string SelectedJointName => jointNames[selectedJoint];
    public bool HasJointState => hasReceivedJointState;

    private ROSConnection ros;
    private int selectedJoint = 0;
    private double[] currentPositions = new double[6];
    public double[] CurrentPositions => currentPositions;
    public string[] JointNames => jointNames;
    private float publishTimer = 0f;
    private bool hasReceivedJointState = false;
    private float smoothedJointInput = 0f;
    private Vector2 smoothedServoLeft = Vector2.zero;
    private Vector2 smoothedServoRight = Vector2.zero;

    [Header("Visualization")]
    [Tooltip("Drag JointTFVisualizer to allow toggling TF axes with X button")]
    public JointTFVisualizer tfVisualizer;

    // Input actions
    private InputAction leftStickAction;
    private InputAction rightStickAction;
    private InputAction nextJointAction;
    private InputAction prevJointAction;
    private InputAction toggleModeAction;
    private InputAction toggleTFAction;

    private static readonly string[] jointNames =
    {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    };

    private const string SERVO_TOPIC = "/servo_node/delta_twist_cmds";
    private const string VELOCITY_TOPIC = "/forward_velocity_controller/commands";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistStampedMsg>(SERVO_TOPIC);
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

        // ToggleTF may not exist in the asset — create dynamically bound to X button
        toggleTFAction = robotMap.FindAction("ToggleTF");
        if (toggleTFAction == null)
        {
            toggleTFAction = new InputAction("ToggleTF", InputActionType.Button,
                "<XRController>{LeftHand}/primaryButton");
            toggleTFAction.Enable();
        }

        robotMap.Enable();
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

        if (mode == ControlMode.Servo)
            UpdateServo();
        else
            UpdateDirectJoint();
    }

    void HandleModeSwitch()
    {
        if (toggleModeAction != null && toggleModeAction.WasPressedThisFrame())
        {
            mode = (mode == ControlMode.Servo) ? ControlMode.DirectJoint : ControlMode.Servo;
            smoothedJointInput = 0f;
            smoothedServoLeft = Vector2.zero;
            smoothedServoRight = Vector2.zero;
            Debug.Log($"[RobotController] Mode: {mode}");
        }

        if (toggleTFAction != null && toggleTFAction.WasPressedThisFrame() && tfVisualizer != null)
        {
            tfVisualizer.Toggle();
            Debug.Log($"[RobotController] TF Axes: {(tfVisualizer.showAxes ? "ON" : "OFF")}");
        }

        if (mode == ControlMode.DirectJoint)
        {
            if (nextJointAction != null && nextJointAction.WasPressedThisFrame())
            {
                smoothedJointInput = 0f; // reset when switching joints
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

    void UpdateServo()
    {
        Vector2 leftStick = leftStickAction != null ? leftStickAction.ReadValue<Vector2>() : Vector2.zero;
        Vector2 rightStick = rightStickAction != null ? rightStickAction.ReadValue<Vector2>() : Vector2.zero;

        float dt = 1f / publishRate;

        smoothedServoLeft.x = SmoothExp(smoothedServoLeft.x, leftStick.x, inputSmoothTime, dt);
        smoothedServoLeft.y = SmoothExp(smoothedServoLeft.y, leftStick.y, inputSmoothTime, dt);
        smoothedServoRight.x = SmoothExp(smoothedServoRight.x, rightStick.x, inputSmoothTime, dt);
        smoothedServoRight.y = SmoothExp(smoothedServoRight.y, rightStick.y, inputSmoothTime, dt);

        var msg = new TwistStampedMsg();
        msg.header.frame_id = "tool0";

        double secs = Time.timeAsDouble;
        msg.header.stamp.sec = (int)secs;
        msg.header.stamp.nanosec = (uint)((secs - (int)secs) * 1e9);

        msg.twist.linear.x = smoothedServoLeft.y * linearSpeed;
        msg.twist.linear.y = smoothedServoLeft.x * linearSpeed;
        msg.twist.linear.z = smoothedServoRight.y * linearSpeed;
        msg.twist.angular.z = smoothedServoRight.x * angularSpeed;

        ros.Publish(SERVO_TOPIC, msg);
    }

    void UpdateDirectJoint()
    {
        if (!hasReceivedJointState) return;

        Vector2 rightStick = rightStickAction != null ? rightStickAction.ReadValue<Vector2>() : Vector2.zero;

        float dt = 1f / publishRate;

        // Smooth stick input for gentle ramp-up/down
        smoothedJointInput = SmoothExp(smoothedJointInput, rightStick.y, inputSmoothTime, dt);
        if (Mathf.Abs(smoothedJointInput) < 0.01f && Mathf.Abs(rightStick.y) < 0.01f)
            smoothedJointInput = 0f;

        // Build velocity command — 6 joints, only the selected one moves
        var msg = new Float64MultiArrayMsg();
        msg.data = new double[6];
        msg.data[selectedJoint] = smoothedJointInput * jointJogSpeed;

        ros.Publish(VELOCITY_TOPIC, msg);
    }

    void OnDestroy()
    {
        // Send zero velocities on shutdown
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

        if (toggleTFAction != null)
            toggleTFAction.Disable();
    }
}
