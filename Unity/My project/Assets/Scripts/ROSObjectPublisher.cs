using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Tf2;
using RosMessageTypes.Geometry;
using RosMessageTypes.Visualization;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

/// <summary>
/// Publishes a Unity GameObject's pose to ROS as a TF frame and a visualization marker.
/// Attach to any GameObject you want visible in RViz with a coordinate frame.
/// </summary>
public class ROSObjectPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    [Tooltip("TF frame name for this object")]
    public string frameName = "unity_cube";

    [Tooltip("Parent TF frame")]
    public string parentFrame = "base_link";

    [Tooltip("Robot base GameObject in Unity (e.g. base_link). If unset, searches by parentFrame name.")]
    public Transform robotBase;

    [Tooltip("Publish rate in Hz")]
    public float publishRate = 10f;

    [Header("Marker Settings")]
    [Tooltip("Publish a visualization marker (cube shape) in addition to TF")]
    public bool publishMarker = true;

    [Tooltip("Marker color")]
    public Color markerColor = new Color(1f, 0.2f, 0.8f, 0.8f); // pink like in your scene

    [Header("Pose Visualization")]
    [Tooltip("Show XYZ axes on the object in Unity")]
    public bool showPoseAxes = true;

    [Tooltip("Length of pose axes (world units)")]
    public float axisLength = 0.15f;

    [Tooltip("Thickness of pose axes (world units)")]
    public float axisThickness = 0.005f;

    [Tooltip("X axis material (red). Assign from Assets/MRTemplateAssets/Materials/AR/XMaterial")]
    public Material xMaterial;
    [Tooltip("Y axis material (green). Assign from Assets/MRTemplateAssets/Materials/AR/YMaterial")]
    public Material yMaterial;
    [Tooltip("Z axis material (blue). Assign from Assets/MRTemplateAssets/Materials/AR/ZMaterial")]
    public Material zMaterial;

    private ROSConnection ros;
    private float timer;

    private const string TF_TOPIC = "/tf";
    private const string MARKER_TOPIC = "/unity_markers";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TFMessageMsg>(TF_TOPIC);
        if (publishMarker)
            ros.RegisterPublisher<MarkerMsg>(MARKER_TOPIC);

        // Auto-find robot base if not assigned
        if (robotBase == null)
        {
            var baseGO = GameObject.Find(parentFrame);
            if (baseGO != null)
                robotBase = baseGO.transform;
            else
                Debug.LogWarning($"[ROSObjectPublisher] Could not find '{parentFrame}' GameObject. Using world origin.");
        }

        if (showPoseAxes)
            CreatePoseAxes();
    }

    /// <summary>
    /// Creates pose axes as child cylinder GameObjects.
    /// Call this to programmatically add axes to any object.
    /// </summary>
    public void CreatePoseAxes()
    {
        // Scale from world units to local space (cube is 0.1 scale)
        float localLen = axisLength / transform.lossyScale.x;
        float localThick = axisThickness / transform.lossyScale.x;

        CreateAxisCylinder("X_Axis", xMaterial, Vector3.right, localLen, localThick);
        CreateAxisCylinder("Y_Axis", yMaterial, Vector3.up, localLen, localThick);
        CreateAxisCylinder("Z_Axis", zMaterial, Vector3.forward, localLen, localThick);
    }

    void CreateAxisCylinder(string name, Material mat, Vector3 dir, float len, float thick)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        go.name = name;
        go.transform.SetParent(transform, false);

        // Remove collider so it doesn't interfere with grabbing
        Destroy(go.GetComponent<Collider>());

        go.transform.localPosition = dir * (len / 2f);

        if (dir == Vector3.right)
            go.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
        else if (dir == Vector3.forward)
            go.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);

        // Cylinder is 2 units tall by default
        go.transform.localScale = new Vector3(thick, len / 2f, thick);

        if (mat != null)
            go.GetComponent<Renderer>().material = mat;
    }

    void Update()
    {
        if (ros == null)
        {
            Debug.LogWarning("[ROSObjectPublisher] ROS connection is null!");
            return;
        }

        timer += Time.deltaTime;
        if (timer < 1f / publishRate) return;
        timer = 0f;

        PublishTF();
        if (publishMarker)
            PublishMarker();
    }

    // Get pose relative to robot base (or world if no base found)
    void GetRelativePose(out Vector3 pos, out Quaternion rot)
    {
        if (robotBase != null)
        {
            // Compute position and rotation relative to the robot base
            pos = robotBase.InverseTransformPoint(transform.position);
            rot = Quaternion.Inverse(robotBase.rotation) * transform.rotation;
        }
        else
        {
            pos = transform.position;
            rot = transform.rotation;
        }
    }

    // Convert Unity relative pose to ROS coordinates
    void UnityToROS(Vector3 uPos, Quaternion uRot,
        out double rx, out double ry, out double rz,
        out double qx, out double qy, out double qz, out double qw)
    {
        // Unity (X-right, Y-up, Z-forward) -> ROS (X-forward, Y-left, Z-up)
        rx = uPos.z;
        ry = -uPos.x;
        rz = uPos.y;
        qx = -uRot.z;
        qy = uRot.x;
        qz = -uRot.y;
        qw = uRot.w;
    }

    void PublishTF()
    {
        GetRelativePose(out Vector3 pos, out Quaternion rot);
        UnityToROS(pos, rot,
            out double rx, out double ry, out double rz,
            out double qx, out double qy, out double qz, out double qw);

        var tf = new TransformStampedMsg();
        tf.header.frame_id = parentFrame;
        tf.header.stamp = GetTimeStamp();
        tf.child_frame_id = frameName;

        tf.transform.translation.x = rx;
        tf.transform.translation.y = ry;
        tf.transform.translation.z = rz;
        tf.transform.rotation.x = qx;
        tf.transform.rotation.y = qy;
        tf.transform.rotation.z = qz;
        tf.transform.rotation.w = qw;

        var msg = new TFMessageMsg();
        msg.transforms = new TransformStampedMsg[] { tf };
        ros.Publish(TF_TOPIC, msg);
    }

    void PublishMarker()
    {
        GetRelativePose(out Vector3 pos, out Quaternion rot);
        UnityToROS(pos, rot,
            out double rx, out double ry, out double rz,
            out double qx, out double qy, out double qz, out double qw);

        var marker = new MarkerMsg();
        marker.header.frame_id = parentFrame;
        marker.header.stamp = GetTimeStamp();
        marker.ns = "unity_objects";
        marker.id = 1;
        marker.type = MarkerMsg.CUBE;
        marker.action = MarkerMsg.ADD;

        marker.pose.position.x = rx;
        marker.pose.position.y = ry;
        marker.pose.position.z = rz;
        marker.pose.orientation.x = qx;
        marker.pose.orientation.y = qy;
        marker.pose.orientation.z = qz;
        marker.pose.orientation.w = qw;

        // Use the GameObject's local scale for marker size
        Vector3 s = transform.lossyScale;
        marker.scale.x = s.z;
        marker.scale.y = s.x;
        marker.scale.z = s.y;

        marker.color.r = markerColor.r;
        marker.color.g = markerColor.g;
        marker.color.b = markerColor.b;
        marker.color.a = markerColor.a;

        ros.Publish(MARKER_TOPIC, marker);
    }

    TimeMsg GetTimeStamp()
    {
        // Use wall clock time so RViz doesn't reject as stale
        double secs = System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
        var stamp = new TimeMsg();
        stamp.sec = (int)secs;
        stamp.nanosec = (uint)((secs - (long)secs) * 1e9);
        return stamp;
    }

}
