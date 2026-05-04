using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    // Last arm joint angles received from /joint_states (radians).
    // Order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3.
    // Stays at zero until the first ROS message arrives.
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

    private Dictionary<string, Transform> jointMap = new Dictionary<string, Transform>();
    private Dictionary<string, Quaternion> initialRotations = new Dictionary<string, Quaternion>();
    private Dictionary<string, Vector3> jointAxes = new Dictionary<string, Vector3>();

    // Map from ROS joint names to URDF-imported Unity GameObject names
    private static readonly Dictionary<string, string> rosToUnity = new Dictionary<string, string>
    {
        { "shoulder_pan_joint",  "shoulder_link" },
        { "shoulder_lift_joint", "upper_arm_link" },
        { "elbow_joint",         "forearm_link" },
        { "wrist_1_joint",       "wrist_1_link" },
        { "wrist_2_joint",       "wrist_2_link" },
        { "wrist_3_joint",       "wrist_3_link" },
    };

    // Gripper joints: Unity name → (axis, mimic multiplier relative to finger_joint)
    // finger_joint range: -0.558505 (closed) to 0.785398 (open)
    private static readonly string[] gripperJointNames =
    {
        "left_outer_knuckle",   // finger_joint (primary)
        "left_inner_knuckle",   // mimic × -1
        "left_inner_finger",    // mimic × 1
        "right_outer_knuckle",  // mimic × -1
        "right_inner_knuckle",  // mimic × -1
        "right_inner_finger",   // mimic × 1
    };
    private static readonly float[] gripperMimicMultipliers = { 1f, -1f, 1f, -1f, -1f, 1f };
    private const float FINGER_JOINT_OPEN = -0.558505f;
    private const float FINGER_JOINT_CLOSED = 0.785398f;
    private const float RG2_MAX_WIDTH = 0.11f; // metres

    private float currentFingerWidth = RG2_MAX_WIDTH; // from /joint_states finger_width

    // Joint limits from URDF (radians)
    private static readonly Dictionary<string, (float lower, float upper)> jointLimits = new Dictionary<string, (float, float)>
    {
        { "shoulder_pan_joint",  (-6.2832f, 6.2832f) },
        { "shoulder_lift_joint", (-6.2832f, 6.2832f) },
        { "elbow_joint",         (-3.1416f, 3.1416f) },
        { "wrist_1_joint",       (-6.2832f, 6.2832f) },
        { "wrist_2_joint",       (-6.2832f, 6.2832f) },
        { "wrist_3_joint",       (-6.2832f, 6.2832f) },
    };

    void Start()
    {
        foreach (var body in GetComponentsInChildren<ArticulationBody>(true))
        {
            string name = body.gameObject.name;
            if (body.jointType != ArticulationJointType.FixedJoint)
            {
                jointMap[name] = body.transform;
                // Save the initial rotation set by the URDF importer (rest pose)
                initialRotations[name] = body.transform.localRotation;
                // Extract the actual rotation axis from the ArticulationBody anchor
                // For revolute joints, the primary axis is the X axis of the anchor frame
                Vector3 axis = body.anchorRotation * Vector3.right;
                jointAxes[name] = axis;
                Debug.Log($"Registered joint: {name} axis: {axis} (initial rot: {body.transform.localRotation.eulerAngles})");
            }

            body.enabled = false;
        }

        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("/joint_states", OnJointState);
    }

    void OnJointState(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            if (msg.name[i] == "finger_width")
            {
                currentFingerWidth = (float)msg.position[i];
                continue;
            }

            string rosName = msg.name[i];
            string unityName = rosToUnity.ContainsKey(rosName) ? rosToUnity[rosName] : rosName;
            if (jointMap.TryGetValue(unityName, out Transform joint))
            {
                float angle = (float)msg.position[i];

                if (jointLimits.TryGetValue(rosName, out var limits))
                {
                    angle = Mathf.Clamp(angle, limits.lower, limits.upper);
                }

                if (rosNameToIndex.TryGetValue(rosName, out int idx))
                {
                    LastJointAnglesRad[idx] = angle;
                    HasReceivedJointState = true;
                }

                float angleDeg = angle * Mathf.Rad2Deg;
                joint.localRotation = initialRotations[unityName] * Quaternion.AngleAxis(angleDeg, jointAxes[unityName]);
            }
        }
    }

    void Update()
    {
        // Map finger_width (metres) to finger_joint angle
        float closedFraction = 1f - Mathf.Clamp01(currentFingerWidth / RG2_MAX_WIDTH);
        float fingerAngle = Mathf.Lerp(FINGER_JOINT_OPEN, FINGER_JOINT_CLOSED, closedFraction);

        for (int i = 0; i < gripperJointNames.Length; i++)
        {
            string unityName = gripperJointNames[i];
            if (!jointMap.TryGetValue(unityName, out Transform joint)) continue;

            float angle = fingerAngle * gripperMimicMultipliers[i];
            float angleDeg = angle * Mathf.Rad2Deg;
            joint.localRotation = initialRotations[unityName] * Quaternion.AngleAxis(angleDeg, jointAxes[unityName]);
        }
    }
}
