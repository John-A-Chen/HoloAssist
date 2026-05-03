using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using System.Collections.Generic;

/// <summary>
/// Subscribes to cube pose topics from the perception pipeline (relayed to base_link frame)
/// and positions virtual objects in the Unity scene at the detected locations.
/// Attach to any GameObject. Assign robotBase to the robot's base_link transform.
/// </summary>
public class CubePoseSubscriber : MonoBehaviour
{
    [Header("ROS Topics")]
    [Tooltip("Prefix for relayed cube pose topics (appends /cube_{N}_pose)")]
    public string topicPrefix = "/holoassist/unity";

    [Tooltip("Number of cubes to track (1-based: cube_1 through cube_N)")]
    public int cubeCount = 4;

    [Header("Robot Reference")]
    [Tooltip("The robot base GameObject (base_link) — poses are relative to this")]
    public Transform robotBase;

    [Header("Virtual Objects")]
    [Tooltip("Prefabs to spawn for each cube (index 0 = cube 1). If fewer than cubeCount, last is reused.")]
    public GameObject[] cubePrefabs;

    [Tooltip("Fallback prefab if cubePrefabs is empty (a simple coloured cube)")]
    public GameObject defaultPrefab;

    [Header("Settings")]
    [Tooltip("Seconds without a pose update before hiding the object")]
    public float poseTimeout = 3.0f;

    [Tooltip("Position smoothing (0 = snap, higher = smoother)")]
    [Range(0f, 0.95f)]
    public float smoothing = 0.5f;

    [Tooltip("Scale of spawned virtual objects")]
    public float objectScale = 0.04f;

    private class CubeState
    {
        public GameObject instance;
        public float lastReceived;
        public Vector3 targetPosition;
        public Quaternion targetRotation;
        public bool visible;
        public string name;
    }

    private Dictionary<string, CubeState> cubes = new Dictionary<string, CubeState>();
    private bool useGeneratedPrefab;

    void Start()
    {
        var ros = ROSConnection.GetOrCreateInstance();
        useGeneratedPrefab = (cubePrefabs == null || cubePrefabs.Length == 0) && defaultPrefab == null;

        for (int i = 1; i <= cubeCount; i++)
        {
            string cubeName = $"cube_{i}";
            string topic = $"{topicPrefix}/{cubeName}_pose";

            GameObject instance = CreateVirtualObject(i);

            cubes[cubeName] = new CubeState
            {
                instance = instance,
                lastReceived = -999f,
                targetPosition = Vector3.zero,
                targetRotation = Quaternion.identity,
                visible = false,
                name = cubeName
            };

            string captured = cubeName;
            ros.Subscribe<PoseStampedMsg>(topic, msg => OnCubePose(captured, msg));
            Debug.Log($"[CubePoseSubscriber] Subscribed to {topic}");
        }
    }

    void Update()
    {
        float now = Time.time;

        foreach (var kvp in cubes)
        {
            var state = kvp.Value;
            if (state.instance == null) continue;

            if (now - state.lastReceived > poseTimeout)
            {
                if (state.visible)
                {
                    state.instance.SetActive(false);
                    state.visible = false;
                }
                continue;
            }

            if (!state.visible)
            {
                state.instance.SetActive(true);
                state.visible = true;
                state.instance.transform.position = state.targetPosition;
                state.instance.transform.rotation = state.targetRotation;
            }
            else
            {
                float t = 1f - smoothing;
                state.instance.transform.position = Vector3.Lerp(
                    state.instance.transform.position, state.targetPosition, t);
                state.instance.transform.rotation = Quaternion.Slerp(
                    state.instance.transform.rotation, state.targetRotation, t);
            }
        }
    }

    void OnCubePose(string cubeName, PoseStampedMsg msg)
    {
        if (!cubes.ContainsKey(cubeName)) return;

        var state = cubes[cubeName];

        // ROS base_link coordinates → Unity world coordinates
        // ROS: X-forward, Y-left, Z-up  →  Unity: X-right, Y-up, Z-forward
        Vector3 rosPos = new Vector3(
            (float)msg.pose.position.x,
            (float)msg.pose.position.y,
            (float)msg.pose.position.z);

        // Standard ROS→Unity coordinate swap
        Vector3 unityLocal = new Vector3(-rosPos.y, rosPos.z, rosPos.x);

        Quaternion rosRot = new Quaternion(
            (float)msg.pose.orientation.x,
            (float)msg.pose.orientation.y,
            (float)msg.pose.orientation.z,
            (float)msg.pose.orientation.w);
        Quaternion unityLocalRot = new Quaternion(rosRot.y, -rosRot.z, -rosRot.x, rosRot.w);

        if (robotBase != null)
        {
            state.targetPosition = robotBase.TransformPoint(unityLocal);
            state.targetRotation = robotBase.rotation * unityLocalRot;
        }
        else
        {
            state.targetPosition = unityLocal;
            state.targetRotation = unityLocalRot;
        }

        state.lastReceived = Time.time;
    }

    GameObject CreateVirtualObject(int cubeIndex)
    {
        GameObject prefab = null;

        if (cubePrefabs != null && cubePrefabs.Length > 0)
        {
            int idx = Mathf.Min(cubeIndex - 1, cubePrefabs.Length - 1);
            prefab = cubePrefabs[idx];
        }
        else if (defaultPrefab != null)
        {
            prefab = defaultPrefab;
        }

        GameObject obj;
        if (prefab != null)
        {
            obj = Instantiate(prefab);
        }
        else
        {
            obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obj.transform.localScale = Vector3.one * objectScale;
            var renderer = obj.GetComponent<Renderer>();
            if (renderer != null)
            {
                Color[] colors = { Color.red, Color.blue, Color.green, Color.yellow };
                renderer.material.color = colors[(cubeIndex - 1) % colors.Length];
            }
        }

        obj.name = $"VirtualCube_{cubeIndex}";
        obj.SetActive(false);
        return obj;
    }

    public bool IsCubeVisible(int cubeIndex)
    {
        string key = $"cube_{cubeIndex}";
        return cubes.ContainsKey(key) && cubes[key].visible;
    }

    public Vector3 GetCubePosition(int cubeIndex)
    {
        string key = $"cube_{cubeIndex}";
        if (cubes.ContainsKey(key))
            return cubes[key].targetPosition;
        return Vector3.zero;
    }
}
