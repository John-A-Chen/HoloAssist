using UnityEngine;

public class SpatialMarkers : MonoBehaviour
{
    [Header("References")]
    public RobotController controller;

    [Header("End-Effector Axes")]
    public float axisLength = 0.08f;
    public float axisThickness = 0.003f;
    public float originRadius = 0.006f;

    [Header("Velocity Arrow")]
    public bool showVelocityArrow = true;
    public float arrowMaxLength = 0.12f;
    public float arrowThickness = 0.004f;

    private Transform toolTransform;
    private GameObject axesGroup;
    private GameObject velocityArrow;
    private Transform arrowShaft;
    private Transform arrowHead;
    private bool initialized = false;

    void LateUpdate()
    {
        if (!initialized)
        {
            TryInitialize();
            if (!initialized) return;
        }

        if (showVelocityArrow && controller != null)
            UpdateVelocityArrow();
    }

    void TryInitialize()
    {
        // Find tool0 in the robot hierarchy
        var allTransforms = GetComponentsInChildren<Transform>();
        foreach (var t in allTransforms)
        {
            if (t.name == "tool0")
            {
                toolTransform = t;
                break;
            }
        }

        if (toolTransform == null) return;

        CreateAxes();
        if (showVelocityArrow)
            CreateVelocityArrow();

        initialized = true;
    }

    void CreateAxes()
    {
        axesGroup = new GameObject("EEAxes");
        axesGroup.transform.SetParent(toolTransform, false);

        // Origin sphere
        var origin = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        origin.transform.SetParent(axesGroup.transform, false);
        origin.transform.localScale = Vector3.one * (originRadius * 2f);
        SetMaterial(origin, Color.white);
        Destroy(origin.GetComponent<Collider>());

        // X axis (red)
        CreateAxisCylinder(axesGroup.transform, Vector3.right, Color.red);
        // Y axis (green)
        CreateAxisCylinder(axesGroup.transform, Vector3.up, Color.green);
        // Z axis (blue)
        CreateAxisCylinder(axesGroup.transform, Vector3.forward, Color.blue);
    }

    void CreateAxisCylinder(Transform parent, Vector3 direction, Color color)
    {
        var cyl = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        cyl.transform.SetParent(parent, false);

        // Unity cylinder is 2m tall along local Y, so scale Y = desired half-length
        cyl.transform.localScale = new Vector3(axisThickness, axisLength / 2f, axisThickness);

        // Offset so the cylinder starts at origin and extends along the direction
        cyl.transform.localPosition = direction * (axisLength / 2f);

        // Rotate so cylinder's local Y aligns with the desired direction
        if (direction == Vector3.right)
            cyl.transform.localRotation = Quaternion.Euler(0f, 0f, -90f);
        else if (direction == Vector3.forward)
            cyl.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
        // Vector3.up needs no rotation (cylinder default)

        SetMaterial(cyl, color);
        Destroy(cyl.GetComponent<Collider>());
    }

    void CreateVelocityArrow()
    {
        velocityArrow = new GameObject("VelocityArrow");
        velocityArrow.transform.SetParent(transform, false);

        // Shaft
        var shaft = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        shaft.transform.SetParent(velocityArrow.transform, false);
        shaft.transform.localScale = new Vector3(arrowThickness, arrowMaxLength / 2f, arrowThickness);
        shaft.transform.localPosition = new Vector3(0f, arrowMaxLength / 2f, 0f);
        SetMaterial(shaft, Color.yellow);
        Destroy(shaft.GetComponent<Collider>());
        arrowShaft = shaft.transform;

        // Head (small sphere as arrowhead)
        var head = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        head.transform.SetParent(velocityArrow.transform, false);
        head.transform.localScale = Vector3.one * (arrowThickness * 3f);
        head.transform.localPosition = new Vector3(0f, arrowMaxLength, 0f);
        SetMaterial(head, Color.yellow);
        Destroy(head.GetComponent<Collider>());
        arrowHead = head.transform;

        velocityArrow.SetActive(false);
    }

    void UpdateVelocityArrow()
    {
        if (velocityArrow == null || controller.VDesired == null) return;

        double[] v = controller.VDesired;
        float mag = Mathf.Sqrt((float)(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));

        if (mag < 0.001f)
        {
            velocityArrow.SetActive(false);
            return;
        }

        velocityArrow.SetActive(true);

        // Convert ROS base frame (X-fwd, Y-left, Z-up) to Unity (X-right, Y-up, Z-fwd)
        // Standard URDF importer convention: Unity.x = -ROS.y, Unity.y = ROS.z, Unity.z = ROS.x
        Vector3 velUnity = new Vector3((float)v[1], (float)v[2], -(float)v[0]);

        // Position at end-effector in robot local space
        velocityArrow.transform.localPosition = transform.InverseTransformPoint(toolTransform.position);

        // Orient arrow along velocity direction (arrow built along local Y)
        velocityArrow.transform.localRotation = Quaternion.FromToRotation(Vector3.up, velUnity.normalized);

        // Scale shaft length proportional to velocity magnitude
        float normalizedMag = Mathf.Clamp01(mag / controller.linearSpeed);
        float length = normalizedMag * arrowMaxLength;
        arrowShaft.localScale = new Vector3(arrowThickness, length / 2f, arrowThickness);
        arrowShaft.localPosition = new Vector3(0f, length / 2f, 0f);
        arrowHead.localPosition = new Vector3(0f, length, 0f);
    }

    void SetMaterial(GameObject obj, Color color)
    {
        var renderer = obj.GetComponent<Renderer>();
        // Try URP Unlit first (Quest 3 project), fallback to built-in
        var shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null)
            shader = Shader.Find("Unlit/Color");

        var mat = new Material(shader);
        mat.color = color;

        // URP Unlit uses _BaseColor
        if (mat.HasProperty("_BaseColor"))
            mat.SetColor("_BaseColor", color);

        renderer.material = mat;
    }
}
