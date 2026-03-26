using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    private Dictionary<string, ArticulationBody> jointMap = new Dictionary<string, ArticulationBody>();

    void Start()
    {
        // Build a map of joint name → ArticulationBody from all UrdfJoint components in the robot
        foreach (var urdfJoint in GetComponentsInChildren<UrdfJoint>())
        {
            var body = urdfJoint.GetComponent<ArticulationBody>();
            if (body != null && body.jointType != ArticulationJointType.FixedJoint)
            {
                jointMap[urdfJoint.jointName] = body;
                Debug.Log($"Registered joint: {urdfJoint.jointName}");
            }
        }

        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("/joint_states", OnJointState);
    }

    void OnJointState(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            if (jointMap.TryGetValue(msg.name[i], out ArticulationBody body))
            {
                var drive = body.xDrive;
                drive.target = (float)msg.position[i] * Mathf.Rad2Deg;
                body.xDrive = drive;
            }
        }
    }
}
