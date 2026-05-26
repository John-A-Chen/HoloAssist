using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using System.Collections.Generic;

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

    [Tooltip("Fallback prefab if cubePrefabs is empty")]
    public GameObject defaultPrefab;

    [Header("Settings")]
    [Tooltip("Seconds without a pose update before hiding the object (only applies when persistOnTagLoss = false)")]
    public float poseTimeout = 3.0f;

    [Tooltip("When true, cubes stay visible at their last known pose even after poseTimeout expires — kills the blink-on-occlusion behaviour. When false (legacy), cubes hide after poseTimeout and respawn on next detection.")]
    public bool persistOnTagLoss = false;

    [Tooltip("Position smoothing (0 = snap to perception update, higher = smoother per-frame lerp). Set to 0 for event-driven visual updates that only move when a new pose arrives.")]
    [Range(0f, 0.95f)]
    public float smoothing = 0.5f;

    [Tooltip("Scale of spawned virtual objects")]
    public float objectScale = 0.04f;

    [Header("Pose Averaging (anti-jitter)")]
    [Tooltip("Apply a moving-average filter to incoming poses to reduce per-frame jitter. The existing `smoothing` lerp still runs on top for visual continuity.")]
    public bool useAveraging = true;

    [Range(1, 30)]
    [Tooltip("How many recent position samples to average. Higher = smoother but more lag. 1 = no averaging (just the smoothing lerp).")]
    public int positionAverageWindow = 6;

    [Range(1, 30)]
    [Tooltip("How many recent rotation samples to average via incremental Slerp. 1 = no rotation averaging.")]
    public int rotationAverageWindow = 3;

    [Tooltip("Reject incoming poses that differ from the current averaged position by more than this distance (metres). 0 = no rejection (let outliers in).")]
    public float outlierRejectDistance = 0f;

    [Header("Task System Integration")]

    [Tooltip("Master toggle. When off, spawned cubes are untagged (preserves pre-tagging behaviour). When on, each spawned cube gets an ObjectClass component using the arrays below — so it counts for BinDetector / TaskTracker on drop.")]
    public bool tagSpawnedCubes = false;

    [Tooltip("Class name per cube (index 0 = cube_1). Used by TaskTracker's OneOfEachClass mode + status display. Empty string skips tagging for that one cube. If shorter than cubeCount, missing entries fall back to \"cube_N\".")]
    public string[] cubeClassNames = { "red", "green", "blue", "yellow" };

    [Tooltip("isGood per cube (index 0 = cube_1). True = sorting this into any bin counts as correct. False = counts as wrong (red flash + ✗ counter). If shorter than cubeCount, missing entries default to true.")]
    public bool[] cubeIsGood = { true, true, true, true };

    [Tooltip("If a Collider is missing on the spawned instance, add a BoxCollider sized from objectScale so BinDetector triggers fire on entry. Skip if your prefab already has its own collider.")]
    public bool autoAddCollider = true;

    private class CubeState
    {
        public GameObject instance;
        public float lastReceived;
        public Vector3 targetPosition;
        public Quaternion targetRotation;
        public bool visible;
        public bool stale; // persistOnTagLoss mode: true after poseTimeout, reset on next pose
        public string name;
        public Queue<Vector3> positionBuffer = new Queue<Vector3>();
        public Queue<Quaternion> rotationBuffer = new Queue<Quaternion>();
    }

    private Dictionary<string, CubeState> cubes = new Dictionary<string, CubeState>();

    void Start()
    {
        var ros = ROSConnection.GetOrCreateInstance();

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
                if (persistOnTagLoss)
                {
                    // Persist mode: leave the cube visible at its last known pose.
                    // Clear buffers exactly once on the transition so a future
                    // detection starts a fresh moving average instead of blending
                    // with poses from before the gap.
                    if (state.visible && !state.stale)
                    {
                        state.positionBuffer.Clear();
                        state.rotationBuffer.Clear();
                        state.stale = true;
                    }
                }
                else if (state.visible)
                {
                    state.instance.SetActive(false);
                    state.visible = false;
                    state.positionBuffer.Clear();
                    state.rotationBuffer.Clear();
                }
                continue;
            }
            state.stale = false;

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

        // ROS base_link coordinates -> Unity world coordinates
        // ROS: X-forward, Y-left, Z-up  ->  Unity: X-right, Y-up, Z-forward
        Vector3 rosPos = new Vector3(
            (float)msg.pose.position.x,
            (float)msg.pose.position.y,
            (float)msg.pose.position.z);

        Vector3 unityLocal = new Vector3(-rosPos.y, rosPos.z, rosPos.x);

        Quaternion rosRot = new Quaternion(
            (float)msg.pose.orientation.x,
            (float)msg.pose.orientation.y,
            (float)msg.pose.orientation.z,
            (float)msg.pose.orientation.w);
        Quaternion unityLocalRot = new Quaternion(rosRot.y, -rosRot.z, -rosRot.x, rosRot.w);

        Vector3 worldPos;
        Quaternion worldRot;
        if (robotBase != null)
        {
            worldPos = robotBase.TransformPoint(unityLocal);
            worldRot = robotBase.rotation * unityLocalRot;
        }
        else
        {
            worldPos = unityLocal;
            worldRot = unityLocalRot;
        }

        // Pose averaging — feed the world-space pose into the moving-average
        // filter. The lerp in Update() still smooths from current → averaged
        // target for visual continuity.
        if (useAveraging)
        {
            // Outlier rejection (optional) — drop spikes that diverge wildly
            // from the recent average so a single bad detection doesn't poison
            // the running average. 0 disables.
            if (outlierRejectDistance > 0f && state.positionBuffer.Count > 0)
            {
                Vector3 currentAvg = AveragePosition(state.positionBuffer);
                if (Vector3.Distance(worldPos, currentAvg) > outlierRejectDistance)
                {
                    state.lastReceived = Time.time; // still counts as "alive"
                    return;
                }
            }

            EnqueueClamped(state.positionBuffer, worldPos, positionAverageWindow);
            EnqueueClamped(state.rotationBuffer, worldRot, rotationAverageWindow);

            state.targetPosition = AveragePosition(state.positionBuffer);
            state.targetRotation = AverageRotation(state.rotationBuffer);
        }
        else
        {
            state.targetPosition = worldPos;
            state.targetRotation = worldRot;
        }

        state.lastReceived = Time.time;
    }

    static void EnqueueClamped<T>(Queue<T> buf, T value, int maxCount)
    {
        buf.Enqueue(value);
        while (buf.Count > Mathf.Max(1, maxCount)) buf.Dequeue();
    }

    static Vector3 AveragePosition(Queue<Vector3> buf)
    {
        if (buf.Count == 0) return Vector3.zero;
        Vector3 sum = Vector3.zero;
        foreach (var p in buf) sum += p;
        return sum / buf.Count;
    }

    // Approximate equal-weighted quaternion average via incremental Slerp.
    // For small angle differences (e.g. perception jitter) this converges to
    // a result very close to the true average without the expense of solving
    // for the principal eigenvector. Quaternion.Slerp internally takes the
    // shorter path so sign-ambiguity isn't a problem.
    static Quaternion AverageRotation(Queue<Quaternion> buf)
    {
        if (buf.Count == 0) return Quaternion.identity;
        Quaternion result = Quaternion.identity;
        int i = 0;
        foreach (var q in buf)
        {
            if (i == 0) result = q;
            else result = Quaternion.Slerp(result, q, 1f / (i + 1));
            i++;
        }
        return result;
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
                Color[] colors = { Color.red, Color.green, Color.blue, new Color(1f, 0.5f, 0f) };
                renderer.material.color = colors[(cubeIndex - 1) % colors.Length];
            }
        }

        obj.name = $"VirtualCube_{cubeIndex}";

        // Ensure Collider so BinDetector trigger fires on entry. Skipped if the
        // prefab already provides one (anywhere in its hierarchy).
        if (autoAddCollider && obj.GetComponentInChildren<Collider>() == null)
        {
            var box = obj.AddComponent<BoxCollider>();
            box.size = Vector3.one * objectScale;
        }

        // Tag with ObjectClass so the cube participates in TaskTracker. Prefab-
        // baked ObjectClass wins automatically — we skip if one already exists
        // anywhere in the hierarchy. Empty className entries skip a single cube.
        if (tagSpawnedCubes && obj.GetComponentInChildren<ObjectClass>() == null)
        {
            int idx = cubeIndex - 1;
            string className = (cubeClassNames != null && idx >= 0 && idx < cubeClassNames.Length)
                ? cubeClassNames[idx]
                : $"cube_{cubeIndex}";
            bool isGood = (cubeIsGood != null && idx >= 0 && idx < cubeIsGood.Length)
                ? cubeIsGood[idx]
                : true;
            if (!string.IsNullOrEmpty(className))
            {
                var oc = obj.AddComponent<ObjectClass>();
                oc.className = className;
                oc.isGood = isGood;
            }
        }

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
