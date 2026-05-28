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

    // Cumulative count — incremented on every new entry, never decremented.
    private int totalDeposited = 0;
    private int correctCount = 0;
    private int wrongCount = 0;

    public int ObjectCount => objectsInBin.Count;
    public int TotalCount => totalDeposited;
    public int CorrectCount => correctCount;
    public int WrongCount => wrongCount;
    public bool HasObjects => objectsInBin.Count > 0;
    public string Status => HasObjects ? $"{objectsInBin.Count} object(s)" : "Empty";

    /// <summary>Reset the cumulative scores for this bin (e.g. start of new game).</summary>
    public void ResetScore()
    {
        totalDeposited = 0;
        correctCount = 0;
        wrongCount = 0;
    }

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
        // Build a WORLD-axis-aligned trigger child so the Y-offset shifts the box
        // UPWARD in world space (not along the bin's local Y, which may point
        // sideways after FBX import rotation). Same strategy as BuildContainerColliders.
        var triggerObj = new GameObject("BinTrigger");
        triggerObj.transform.position = worldBounds.center;
        triggerObj.transform.rotation = Quaternion.identity;
        triggerObj.transform.localScale = Vector3.one;
        triggerObj.transform.SetParent(transform, worldPositionStays: true);

        var trigger = triggerObj.AddComponent<BoxCollider>();
        trigger.center = new Vector3(0f, triggerYOffset, 0f);
        trigger.size = worldBounds.size * triggerSizeMultiplier;
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
        // Build a WORLD-axis-aligned container so the floor stays at the bottom and
        // side walls stay vertical regardless of the bin GameObject's rotation
        // (e.g. FBX import rotation -90 X from Z-up source data) and scale
        // (e.g. FBX imported at 0.15× scale).
        //
        // Strategy: set the container's world transform first, then re-parent with
        // worldPositionStays=true so the bin's rotation/scale are NOT inherited.
        var container = new GameObject("BinContainerColliders");
        container.transform.position = worldBounds.center;
        container.transform.rotation = Quaternion.identity;
        container.transform.localScale = Vector3.one;
        container.transform.SetParent(transform, worldPositionStays: true);
        container.layer = gameObject.layer;

        float wt = Mathf.Max(0.001f, wallThickness);
        Vector3 size = worldBounds.size;
        float innerX = Mathf.Max(0.001f, size.x - 2f * wallInset);
        float innerZ = Mathf.Max(0.001f, size.z - 2f * wallInset);
        float halfX = innerX * 0.5f;
        float halfZ = innerZ * 0.5f;
        float wallHeight = size.y;
        float halfY = size.y * 0.5f;

        // All wall positions are in the container's world-aligned local space
        // (origin at world bounds center). Floor at world -Y, side walls vertical.

        AddWall(container.transform, "Floor",
            new Vector3(0f, -halfY + wt * 0.5f, 0f),
            new Vector3(innerX, wt, innerZ));

        // Side walls — full bin height in world Y. Top open.
        AddWall(container.transform, "Wall_PosX",
            new Vector3(halfX - wt * 0.5f, 0f, 0f),
            new Vector3(wt, wallHeight, innerZ));

        AddWall(container.transform, "Wall_NegX",
            new Vector3(-halfX + wt * 0.5f, 0f, 0f),
            new Vector3(wt, wallHeight, innerZ));

        AddWall(container.transform, "Wall_PosZ",
            new Vector3(0f, 0f, halfZ - wt * 0.5f),
            new Vector3(innerX, wallHeight, wt));

        AddWall(container.transform, "Wall_NegZ",
            new Vector3(0f, 0f, -halfZ + wt * 0.5f),
            new Vector3(innerX, wallHeight, wt));

        Debug.Log($"[BinDetector] {binName}: container colliders built (inner {innerX:F3}×{wallHeight:F3}×{innerZ:F3}) world-aligned");
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
        HandleEntry(other);
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
        HandleEntry(other);
    }

    void HandleEntry(Collider other)
    {
        bool isNew = objectsInBin.Add(other);
        if (!isNew) { UpdatePanel(); return; }

        totalDeposited++;

        // Classify: needs ObjectClass on the entering object's hierarchy.
        // Untagged objects are ignored (no correct/wrong counted, no banner/confetti).
        var oc = ObjectClass.FindOn(other);
        if (oc == null)
        {
            Debug.Log($"[BinDetector] {other.gameObject.name} entered {binName} (untagged — ignored)");
            UpdatePanel();
            return;
        }

        if (oc.isGood)
        {
            correctCount++;
            Debug.Log($"[BinDetector] CORRECT: {oc.className} entered {binName} (✓{correctCount}, ✗{wrongCount})");
            ConfettiBlaster.FireIfEnabled();
            if (EventBanner.Instance != null) EventBanner.Instance.FlashCorrect();
        }
        else
        {
            wrongCount++;
            Debug.Log($"[BinDetector] WRONG: {oc.className} entered {binName} (✓{correctCount}, ✗{wrongCount})");
            if (EventBanner.Instance != null) EventBanner.Instance.FlashWrong();
        }

        if (TaskTracker.Instance != null)
            TaskTracker.Instance.OnSort(this, oc, oc.isGood);

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
        // Visualize the trigger zone in the editor.
        // Trigger lives in world-aligned space (rotation = identity), so use its
        // own localToWorldMatrix, not the bin's.
        var triggerChild = transform.Find("BinTrigger");
        if (triggerChild != null)
        {
            var box = triggerChild.GetComponent<BoxCollider>();
            if (box != null)
            {
                Gizmos.color = HasObjects ? new Color(0, 1, 0, 0.3f) : new Color(1, 1, 0, 0.3f);
                Gizmos.matrix = triggerChild.localToWorldMatrix;
                Gizmos.DrawCube(box.center, box.size);
                Gizmos.color = HasObjects ? Color.green : Color.yellow;
                Gizmos.DrawWireCube(box.center, box.size);
            }
        }

        // Visualize container walls + floor.
        // Walls live in the container's world-aligned local space (rotation = identity),
        // so use the container's localToWorldMatrix, not the bin's.
        var container = transform.Find("BinContainerColliders");
        if (container != null)
        {
            Gizmos.color = new Color(0.3f, 0.6f, 1f, 0.9f);
            Gizmos.matrix = container.localToWorldMatrix;
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
