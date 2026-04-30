using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Draws XYZ axes on each UR3e joint link, tool0, and all non-static
/// objects in the scene. Toggle visibility at runtime.
/// Attach to the root robot GameObject (e.g. "ur3e_robot").
/// </summary>
public class JointTFVisualizer : MonoBehaviour
{
    [Header("Axes Settings")]
    public float axisLength = 0.08f;
    public float axisThickness = 0.003f;

    [Header("Environment Object Axes")]
    [Tooltip("Axis length for non-robot objects in the scene")]
    public float envAxisLength = 0.12f;
    public float envAxisThickness = 0.004f;

    [Header("Colors")]
    public Color xColor = new Color(1f, 0.2f, 0.2f, 1f); // Red
    public Color yColor = new Color(0.2f, 1f, 0.2f, 1f); // Green
    public Color zColor = new Color(0.3f, 0.3f, 1f, 1f);  // Blue

    [Header("Linked Objects")]
    [Tooltip("Other objects with ROSObjectPublisher whose axes should toggle together")]
    public ROSObjectPublisher[] linkedPublishers;

    [Header("Auto-Discovery")]
    [Tooltip("Automatically find and add axes to all non-static objects with renderers")]
    public bool autoDiscoverMovingObjects = true;

    [Header("Visibility")]
    public bool showAxes = false;

    private static readonly string[] linkNames =
    {
        "base_link",
        "shoulder_link",
        "upper_arm_link",
        "forearm_link",
        "wrist_1_link",
        "wrist_2_link",
        "wrist_3_link",
        "tool0"
    };

    // Tags to skip during auto-discovery
    private static readonly HashSet<string> skipNames = new HashSet<string>
    {
        // Robot link names (already handled)
        "base_link", "base", "base_link_inertia", "shoulder_link",
        "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link",
        "wrist_3_link", "flange", "tool0",
        // UI / system objects
        "RobotDataPanel", "RobotHUD", "RadialMenu",
        "Main Camera", "Camera",
        // XR system
        "Left Controller", "Right Controller",
        "LeftHand Controller", "RightHand Controller",
    };

    private List<GameObject> axisObjects = new List<GameObject>();
    private List<Transform> discoveredObjects = new List<Transform>();
    private bool built = false;

    void Start()
    {
        BuildAxes();
        if (autoDiscoverMovingObjects)
            DiscoverMovingObjects();
        SetVisible(showAxes);
    }

    void BuildAxes()
    {
        if (built) return;

        foreach (string linkName in linkNames)
        {
            Transform link = FindDeepChild(transform, linkName);
            if (link == null)
            {
                Debug.LogWarning($"[JointTFVisualizer] Could not find link: {linkName}");
                continue;
            }

            CreateAxisSet(link, axisLength, axisThickness);
        }

        built = true;
    }

    void DiscoverMovingObjects()
    {
        var allRenderers = FindObjectsByType<MeshRenderer>(FindObjectsSortMode.None);

        foreach (var renderer in allRenderers)
        {
            GameObject go = renderer.gameObject;

            // Skip static objects
            if (go.isStatic) continue;

            // Skip if it's part of the robot hierarchy
            if (go.transform.IsChildOf(transform)) continue;

            // Skip known system objects by name
            if (skipNames.Contains(go.name)) continue;

            // Skip if parent is in skip list (e.g. children of controllers)
            bool skipParent = false;
            Transform p = go.transform.parent;
            while (p != null)
            {
                if (skipNames.Contains(p.name)) { skipParent = true; break; }
                // Skip anything under XR Interaction Setup / MR Interaction Setup
                if (p.name.Contains("Interaction Setup")) { skipParent = true; break; }
                if (p.name.Contains("Controller")) { skipParent = true; break; }
                p = p.parent;
            }
            if (skipParent) continue;

            // Skip sub-meshes: only add axes to the root of a model, not each child mesh
            // (if parent has a MeshRenderer too and isn't static, skip this child)
            if (go.transform.parent != null &&
                go.transform.parent.GetComponent<MeshRenderer>() != null &&
                !go.transform.parent.gameObject.isStatic)
                continue;

            // Skip if this object already has TF axes (from ROSObjectPublisher)
            if (go.GetComponent<ROSObjectPublisher>() != null) continue;

            // Skip objects that are clearly UI/visualization
            if (go.GetComponent<TMPro.TextMeshPro>() != null) continue;
            if (go.name.StartsWith("TF_") || go.name.StartsWith("Btn_") ||
                go.name.StartsWith("Ring_") || go.name.StartsWith("HUD")) continue;

            // This is a non-static environment object — add axes
            CreateAxisSet(go.transform, envAxisLength, envAxisThickness);
            discoveredObjects.Add(go.transform);
            Debug.Log($"[JointTFVisualizer] Auto-discovered moving object: {go.name}");
        }
    }

    void CreateAxisSet(Transform parent, float length, float thickness)
    {
        CreateAxisCylinder(parent, "TF_X", Vector3.right, xColor, length, thickness);
        CreateAxisCylinder(parent, "TF_Y", Vector3.up, yColor, length, thickness);
        CreateAxisCylinder(parent, "TF_Z", Vector3.forward, zColor, length, thickness);
    }

    void CreateAxisCylinder(Transform parent, string name, Vector3 dir, Color color,
        float length, float thickness)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        go.name = name;
        go.transform.SetParent(parent, false);

        Destroy(go.GetComponent<Collider>());

        // Scale from world to local space
        float scale = parent.lossyScale.x;
        if (scale < 0.001f) scale = 1f;
        float localLen = length / scale;
        float localThick = thickness / scale;

        go.transform.localPosition = dir * (localLen / 2f);

        if (dir == Vector3.right)
            go.transform.localRotation = Quaternion.Euler(0f, 0f, 90f);
        else if (dir == Vector3.forward)
            go.transform.localRotation = Quaternion.Euler(90f, 0f, 0f);

        // Cylinder is 2 units tall by default
        go.transform.localScale = new Vector3(localThick, localLen / 2f, localThick);

        var mat = new Material(Shader.Find("Universal Render Pipeline/Unlit"));
        mat.SetColor("_BaseColor", color);
        go.GetComponent<Renderer>().material = mat;

        axisObjects.Add(go);
    }

    Transform FindDeepChild(Transform parent, string name)
    {
        if (parent.name == name) return parent;
        foreach (Transform child in parent)
        {
            var result = FindDeepChild(child, name);
            if (result != null) return result;
        }
        return null;
    }

    public void SetVisible(bool visible)
    {
        showAxes = visible;
        foreach (var obj in axisObjects)
        {
            if (obj != null)
                obj.SetActive(visible);
        }

        // Toggle linked publishers (e.g. cube axes)
        if (linkedPublishers != null)
        {
            foreach (var pub in linkedPublishers)
                if (pub != null) pub.SetAxesVisible(visible);
        }
    }

    public void Toggle()
    {
        SetVisible(!showAxes);
    }

    /// <summary>
    /// Returns list of auto-discovered non-static objects.
    /// </summary>
    public List<Transform> GetDiscoveredObjects()
    {
        return discoveredObjects;
    }

    void Update()
    {
        // Sync inspector checkbox changes at runtime
        bool anyActive = axisObjects.Count > 0 && axisObjects[0] != null && axisObjects[0].activeSelf;
        if (anyActive != showAxes)
            SetVisible(showAxes);
    }
}
