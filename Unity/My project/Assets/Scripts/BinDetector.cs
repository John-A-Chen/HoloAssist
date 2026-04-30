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

        // Always create a separate trigger collider sized to bin mesh bounds
        // This avoids conflicts with any existing physics colliders on the bin
        SetupTriggerZone();
    }

    void SetupTriggerZone()
    {
        // Get mesh bounds from any renderer (in self or children)
        Bounds worldBounds = new Bounds(transform.position, Vector3.one * 0.3f); // fallback
        var renderers = GetComponentsInChildren<Renderer>();
        bool hasBounds = false;
        foreach (var r in renderers)
        {
            // Skip our own trigger and any axis cylinders we added
            if (r.gameObject.name.Contains("BinTrigger") ||
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
