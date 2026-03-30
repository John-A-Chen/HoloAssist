using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
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
            string rosName = msg.name[i];
            string unityName = rosToUnity.ContainsKey(rosName) ? rosToUnity[rosName] : rosName;
            if (jointMap.TryGetValue(unityName, out Transform joint))
            {
                float angle = (float)msg.position[i];

                // Clamp to joint limits
                if (jointLimits.TryGetValue(rosName, out var limits))
                {
                    angle = Mathf.Clamp(angle, limits.lower, limits.upper);
                }

                float angleDeg = angle * Mathf.Rad2Deg;

                // Compose: initial URDF rotation * joint rotation around extracted axis
                // Negate angle for ROS (right-hand) -> Unity (left-hand) conversion
                joint.localRotation = initialRotations[unityName] * Quaternion.AngleAxis(angleDeg, jointAxes[unityName]);
            }
        }
    }
}
