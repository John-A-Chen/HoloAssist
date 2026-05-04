using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Detects when objects are placed inside a bin using trigger colliders.
/// Attach to a bin GameObject. Requires a trigger collider on the bin.
/// Reports status to RobotDataPanel if assigned.
/// </summary>
public class BinDetector : MonoBehaviour
{
    [Header("Settings")]
    [Tooltip("Display name for this bin")]
    public string binName = "Bin";

    [Tooltip("Layers to detect (default = Everything)")]
    public LayerMask detectLayers = ~0;

    [Header("References")]
    [Tooltip("BinStatusPanel to update with bin status (auto-found if blank)")]
    public BinStatusPanel statusPanel;

    [Header("Auto-Sized Trigger Zone")]
    [Tooltip("Trigger zone size as multiple of mesh bounds (1 = exact bounds)")]
    public float triggerSizeMultiplier = 1.2f;

    [Tooltip("Vertical offset of trigger zone (positive = up)")]
    public float triggerYOffset = 0.05f;

    [Header("Container Colliders (physical walls + floor)")]
    [Tooltip("Auto-generate box colliders along bin walls + floor so dropped objects stay inside.")]
    public bool buildContainerColliders = true;

    [Tooltip("Wall / floor thickness in metres.")]
    public float wallThickness = 0.01f;

    [Tooltip("Distance to inset walls toward the bin centre (m). Bring flush with the inside of the mesh if needed.")]
    public float wallInset = 0.0f;

    [Tooltip("Disable any existing convex MeshCollider on the bin mesh — convex meshes block objects from entering the bin.")]
    public bool disableBlockingMeshColliders = true;

    // Current objects inside the bin
    private HashSet<Collider> objectsInBin = new HashSet<Collider>();

    public int ObjectCount => objectsInBin.Count;
    public bool HasObjects => objectsInBin.Count > 0;
    public string Status => HasObjects ? $"{objectsInBin.Count} object(s)" : "Empty";

    void Start()
    {
        // Auto-find BinStatusPanel if not assigned
        if (statusPanel == null)
        {
            statusPanel = FindFirstObjectByType<BinStatusPanel>();
            if (statusPanel != null)
                Debug.Log($"[BinDetector] {binName}: auto-found BinStatusPanel");
            else
                Debug.LogWarning($"[BinDetector] {binName}: no BinStatusPanel in scene");
        }

        Bounds worldBounds = ComputeMeshBounds();

        // Always create a separate trigger collider sized to bin mesh bounds
        // This avoids conflicts with any existing physics colliders on the bin
        SetupTriggerZone(worldBounds);

        if (disableBlockingMeshColliders)
            DisableBlockingMeshColliders();

        if (buildContainerColliders)
            BuildContainerColliders(worldBounds);
    }

    Bounds ComputeMeshBounds()
    {
        Bounds worldBounds = new Bounds(transform.position, Vector3.one * 0.3f); // fallback
        var renderers = GetComponentsInChildren<Renderer>();
        bool hasBounds = false;
        foreach (var r in renderers)
        {
            // Skip our own helper objects
            if (r.gameObject.name.Contains("BinTrigger") ||
                r.gameObject.name.Contains("BinContainerColliders") ||
                r.gameObject.name.StartsWith("TF_")) continue;

            if (!hasBounds)
            {
                worldBounds = r.bounds;
                hasBounds = true;
            }
            else
            {
                worldBounds.Encapsulate(r.bounds);
            }
        }
        return worldBounds;
    }

    void SetupTriggerZone(Bounds worldBounds)
    {
        // Convert world bounds to local space (handle parent scaling)
        Vector3 localCenter = transform.InverseTransformPoint(worldBounds.center);
        Vector3 localSize = transform.InverseTransformVector(worldBounds.size);
        localSize = new Vector3(Mathf.Abs(localSize.x), Mathf.Abs(localSize.y), Mathf.Abs(localSize.z));

        // Create child trigger object
        var triggerObj = new GameObject("BinTrigger");
        triggerObj.transform.SetParent(transform, false);
        triggerObj.transform.localPosition = Vector3.zero;
        triggerObj.transform.localRotation = Quaternion.identity;
        triggerObj.transform.localScale = Vector3.one;

        var trigger = triggerObj.AddComponent<BoxCollider>();
        trigger.center = localCenter + new Vector3(0, triggerYOffset / Mathf.Max(transform.lossyScale.y, 0.001f), 0);
        trigger.size = localSize * triggerSizeMultiplier;
        trigger.isTrigger = true;

        // Need a Rigidbody on the trigger object for trigger events
        var rb = triggerObj.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;

        // Relay events back to this BinDetector
        var relay = triggerObj.AddComponent<BinTriggerRelay>();
        relay.parent = this;

        Debug.Log($"[BinDetector] {binName}: trigger zone center={trigger.center} size={trigger.size}");
    }

    void BuildContainerColliders(Bounds worldBounds)
    {
        Vector3 localCenter = transform.InverseTransformPoint(worldBounds.center);
        Vector3 localSize = transform.InverseTransformVector(worldBounds.size);
        localSize = new Vector3(Mathf.Abs(localSize.x), Mathf.Abs(localSize.y), Mathf.Abs(localSize.z));

        float wt = Mathf.Max(0.001f, wallThickness);
        float innerX = Mathf.Max(0.001f, localSize.x - 2f * wallInset);
        float innerZ = Mathf.Max(0.001f, localSize.z - 2f * wallInset);
        float halfX = innerX * 0.5f;
        float halfZ = innerZ * 0.5f;
        float wallHeight = localSize.y;
        float bottomY = localCenter.y - localSize.y * 0.5f;

        var container = new GameObject("BinContainerColliders");
        container.transform.SetParent(transform, false);
        container.transform.localPosition = Vector3.zero;
        container.transform.localRotation = Quaternion.identity;
        container.transform.localScale = Vector3.one;
        container.layer = gameObject.layer;

        // Floor — sits at the bottom of the bin volume
        AddWall(container.transform, "Floor",
            new Vector3(localCenter.x, bottomY + wt * 0.5f, localCenter.z),
            new Vector3(innerX, wt, innerZ));

        // Side walls — full bin height. Top is open so things can be dropped in.
        AddWall(container.transform, "Wall_PosX",
            new Vector3(localCenter.x + halfX - wt * 0.5f, localCenter.y, localCenter.z),
            new Vector3(wt, wallHeight, innerZ));

        AddWall(container.transform, "Wall_NegX",
            new Vector3(localCenter.x - halfX + wt * 0.5f, localCenter.y, localCenter.z),
            new Vector3(wt, wallHeight, innerZ));

        AddWall(container.transform, "Wall_PosZ",
            new Vector3(localCenter.x, localCenter.y, localCenter.z + halfZ - wt * 0.5f),
            new Vector3(innerX, wallHeight, wt));

        AddWall(container.transform, "Wall_NegZ",
            new Vector3(localCenter.x, localCenter.y, localCenter.z - halfZ + wt * 0.5f),
            new Vector3(innerX, wallHeight, wt));

        Debug.Log($"[BinDetector] {binName}: container colliders built (inner {innerX:F3}×{wallHeight:F3}×{innerZ:F3})");
    }

    void AddWall(Transform parent, string name, Vector3 localCenter, Vector3 size)
    {
        var go = new GameObject(name);
        go.transform.SetParent(parent, false);
        go.transform.localPosition = localCenter;
        go.transform.localRotation = Quaternion.identity;
        go.layer = gameObject.layer;
        var box = go.AddComponent<BoxCollider>();
        box.center = Vector3.zero;
        box.size = size;
        box.isTrigger = false;
    }

    void DisableBlockingMeshColliders()
    {
        // A convex MeshCollider on the bin mesh fills the bin's volume — dynamic objects
        // dropped from above bounce off the top instead of falling inside. Disable any we find.
        var colliders = GetComponentsInChildren<MeshCollider>();
        foreach (var mc in colliders)
        {
            // Skip helper objects we own.
            Transform t = mc.transform;
            if (t.name == "BinTrigger" ||
                (t.parent != null && t.parent.name == "BinContainerColliders"))
                continue;

            if (mc.convex && mc.enabled)
            {
                mc.enabled = false;
                Debug.Log($"[BinDetector] {binName}: disabled convex MeshCollider on '{mc.gameObject.name}' (would block bin interior).");
            }
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (!ShouldDetect(other)) return;

        objectsInBin.Add(other);
        Debug.Log($"[BinDetector] {other.gameObject.name} entered {binName} ({objectsInBin.Count} objects)");
        UpdatePanel();
    }

    void OnTriggerExit(Collider other)
    {
        if (objectsInBin.Remove(other))
        {
            Debug.Log($"[BinDetector] {other.gameObject.name} left {binName} ({objectsInBin.Count} objects)");
            UpdatePanel();
        }
    }

    public void OnChildTriggerEnter(Collider other)
    {
        if (!ShouldDetect(other)) return;
        objectsInBin.Add(other);
        Debug.Log($"[BinDetector] {other.gameObject.name} entered {binName} ({objectsInBin.Count} objects)");
        UpdatePanel();
    }

    public void OnChildTriggerExit(Collider other)
    {
        if (objectsInBin.Remove(other))
        {
            Debug.Log($"[BinDetector] {other.gameObject.name} left {binName} ({objectsInBin.Count} objects)");
            UpdatePanel();
        }
    }

    bool ShouldDetect(Collider other)
    {
        // Don't detect self or children
        if (other.transform.IsChildOf(transform)) return false;
        // Don't detect any other bin's walls/mesh — adjacent bins' container colliders
        // can overlap our 1.2× trigger zone otherwise.
        if (other.GetComponentInParent<BinDetector>() != null) return false;
        // Don't detect trigger colliders
        if (other.isTrigger) return false;
        // Layer check
        if ((detectLayers & (1 << other.gameObject.layer)) == 0) return false;
        return true;
    }

    void UpdatePanel()
    {
        if (statusPanel != null)
            statusPanel.SetBinStatus(binName, Status);
    }

    void OnDestroy()
    {
        objectsInBin.Clear();
    }

    void OnDrawGizmosSelected()
    {
        // Visualize the trigger zone in the editor
        var triggerChild = transform.Find("BinTrigger");
        if (triggerChild != null)
        {
            var box = triggerChild.GetComponent<BoxCollider>();
            if (box != null)
            {
                Gizmos.color = HasObjects ? new Color(0, 1, 0, 0.3f) : new Color(1, 1, 0, 0.3f);
                Gizmos.matrix = transform.localToWorldMatrix;
                Gizmos.DrawCube(box.center, box.size);
                Gizmos.color = HasObjects ? Color.green : Color.yellow;
                Gizmos.DrawWireCube(box.center, box.size);
            }
        }

        // Visualize container walls + floor
        var container = transform.Find("BinContainerColliders");
        if (container != null)
        {
            Gizmos.color = new Color(0.3f, 0.6f, 1f, 0.9f);
            Gizmos.matrix = transform.localToWorldMatrix;
            for (int i = 0; i < container.childCount; i++)
            {
                var wall = container.GetChild(i);
                var box = wall.GetComponent<BoxCollider>();
                if (box == null) continue;
                Gizmos.DrawWireCube(wall.localPosition + box.center, box.size);
            }
        }
    }
}

/// <summary>
/// Relay trigger events from a child trigger collider back to the parent BinDetector.
/// </summary>
public class BinTriggerRelay : MonoBehaviour
{
    [HideInInspector] public BinDetector parent;

    void OnTriggerEnter(Collider other)
    {
        if (parent != null) parent.OnChildTriggerEnter(other);
    }

    void OnTriggerExit(Collider other)
    {
        if (parent != null) parent.OnChildTriggerExit(other);
    }
}
