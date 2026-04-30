using System.Collections.Generic;
using UnityEngine;

public class MeshCollisionGuard : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Robot root GameObject (e.g. 'ur3e_rg2'). All child colliders are auto-discovered.")]
    public Transform robotBase;

    [Header("Table")]
    [Tooltip("Table surface object — uses its Y position as collision boundary.")]
    public Transform tableTransform;
    [Tooltip("Fallback table height (world Y) if tableTransform not set.")]
    public float tableWorldY = 0f;
    [Tooltip("Hard stop distance (m) above table.")]
    public float tableMargin = 0.01f;
    [Tooltip("Gradual slowdown zone (m) above the hard margin.")]
    public float tableSoftZone = 0.05f;

    [Header("Self-Collision")]
    [Tooltip("Hard stop distance (m) between proximal and distal links.")]
    public float selfMargin = 0.005f;
    [Tooltip("Gradual slowdown zone (m) above self-collision margin.")]
    public float selfSoftZone = 0.03f;

    [Header("Debug")]
    public bool showDebug = true;

    public float VelocityScale { get; private set; } = 1f;
    public bool IsBlocked => VelocityScale <= 0f;

    private List<Collider> proximalColliders = new List<Collider>();
    private List<Collider> distalColliders = new List<Collider>();
    private List<Collider> allRobotColliders = new List<Collider>();

    private static readonly HashSet<string> proximalNames = new HashSet<string> {
        "base_link", "shoulder_link", "upper_arm_link"
    };

    private static readonly HashSet<string> distalNames = new HashSet<string> {
        "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0",
        "onrobot_rg2_base_link", "left_outer_knuckle", "left_inner_knuckle",
        "left_inner_finger", "right_outer_knuckle", "right_inner_knuckle",
        "right_inner_finger"
    };

    void Start()
    {
        if (robotBase == null)
        {
            Debug.LogWarning("[MeshCollisionGuard] robotBase not assigned — collision protection disabled");
            return;
        }
        DiscoverColliders();
    }

    void DiscoverColliders()
    {
        foreach (var col in robotBase.GetComponentsInChildren<Collider>(true))
        {
            if (col is MeshCollider mc && !mc.convex)
                mc.convex = true;

            string linkName = FindParentLinkName(col.transform);
            if (linkName == null) continue;

            allRobotColliders.Add(col);

            if (proximalNames.Contains(linkName))
                proximalColliders.Add(col);
            else if (distalNames.Contains(linkName))
                distalColliders.Add(col);
        }

        Debug.Log($"[MeshCollisionGuard] Found {allRobotColliders.Count} colliders — proximal: {proximalColliders.Count}, distal: {distalColliders.Count}");
    }

    string FindParentLinkName(Transform t)
    {
        while (t != null && t != robotBase.parent)
        {
            if (proximalNames.Contains(t.name) || distalNames.Contains(t.name))
                return t.name;
            t = t.parent;
        }
        return null;
    }

    public float ComputeVelocityScale()
    {
        if (allRobotColliders.Count == 0) { VelocityScale = 1f; return 1f; }

        float scale = 1f;
        float tableY = (tableTransform != null) ? tableTransform.position.y : tableWorldY;

        // --- Table collision: check lowest point of each collider mesh ---
        foreach (var col in allRobotColliders)
        {
            if (!col.enabled) continue;
            float dist = col.bounds.min.y - tableY;

            if (dist <= tableMargin)
            {
                if (showDebug) Debug.Log($"[MeshCollisionGuard] TABLE HARD STOP — {col.name} at {col.bounds.min.y:F3}, table at {tableY:F3}");
                VelocityScale = 0f;
                return 0f;
            }
            if (dist < tableMargin + tableSoftZone)
            {
                float s = (dist - tableMargin) / tableSoftZone;
                if (s < scale) scale = s;
            }
        }

        // --- Self-collision: proximal vs distal links ---
        foreach (var pCol in proximalColliders)
        {
            if (!pCol.enabled) continue;
            foreach (var dCol in distalColliders)
            {
                if (!dCol.enabled) continue;

                // Fast AABB reject
                Bounds pBounds = pCol.bounds;
                Bounds dBounds = dCol.bounds;
                float expandedMargin = selfMargin + selfSoftZone;
                pBounds.Expand(expandedMargin * 2f);
                if (!pBounds.Intersects(dBounds)) continue;

                // Precise distance via ClosestPoint (works on convex colliders)
                Vector3 onD = dCol.ClosestPoint(pCol.bounds.center);
                Vector3 onP = pCol.ClosestPoint(onD);
                float dist = Vector3.Distance(onP, onD);

                if (dist <= selfMargin)
                {
                    if (showDebug) Debug.Log($"[MeshCollisionGuard] SELF-COLLISION STOP — {pCol.name} <-> {dCol.name}, dist={dist:F4}");
                    VelocityScale = 0f;
                    return 0f;
                }
                if (dist < selfMargin + selfSoftZone)
                {
                    float s = (dist - selfMargin) / selfSoftZone;
                    if (s < scale) scale = s;
                }
            }
        }

        VelocityScale = scale;
        return scale;
    }

    void OnDrawGizmos()
    {
        if (!showDebug || allRobotColliders == null) return;

        float tableY = (tableTransform != null) ? tableTransform.position.y : tableWorldY;
        float hardY = tableY + tableMargin;
        float softY = hardY + tableSoftZone;

        // Draw table planes
        Vector3 center = transform.position;
        center.y = hardY;
        Gizmos.color = new Color(1f, 0.2f, 0.2f, 0.3f);
        Gizmos.DrawCube(center, new Vector3(1f, 0.001f, 1f));
        center.y = softY;
        Gizmos.color = new Color(1f, 1f, 0f, 0.15f);
        Gizmos.DrawCube(center, new Vector3(1f, 0.001f, 1f));

        // Colour collider bounds by zone
        foreach (var col in allRobotColliders)
        {
            if (col == null || !col.enabled) continue;
            float dist = col.bounds.min.y - tableY;
            if (dist <= tableMargin)
                Gizmos.color = Color.red;
            else if (dist < tableMargin + tableSoftZone)
                Gizmos.color = new Color(1f, 0.5f, 0f);
            else
                Gizmos.color = Color.green;

            Gizmos.DrawWireCube(col.bounds.center, col.bounds.size);
        }
    }
}
