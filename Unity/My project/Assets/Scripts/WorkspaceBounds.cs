using UnityEngine;

/// <summary>
/// Generates invisible box colliders forming a play-area boundary so dropped
/// objects (cube, sorted items, etc.) stay confined inside the workspace.
///
/// Attach to any empty GameObject. Set <see cref="center"/> + <see cref="size"/>
/// in the Inspector to define the volume. The component creates a child
/// "WorkspaceWalls" GameObject containing 4 walls + a floor (and optionally a
/// ceiling) as <see cref="BoxCollider"/>s. No renderers — invisible at runtime.
///
/// A wireframe gizmo is drawn in the Scene view so you can place + resize
/// visually. Use the right-click context menu "Rebuild Walls" to regenerate
/// colliders after editing dimensions in the Inspector.
/// </summary>
public class WorkspaceBounds : MonoBehaviour
{
    [Header("Bounds (local space — moves with this transform)")]
    [Tooltip("Center of the workspace volume.")]
    public Vector3 center = Vector3.zero;

    [Tooltip("Size of the workspace volume in metres (X width, Y height, Z depth).")]
    public Vector3 size = new Vector3(2f, 2f, 2f);

    [Header("Walls")]
    [Tooltip("Wall / floor thickness in metres. Thicker = more reliable for fast-moving objects.")]
    public float wallThickness = 0.05f;

    [Tooltip("If true, adds a ceiling so objects can't be thrown above the workspace.")]
    public bool includeCeiling = false;

    [Header("Visuals")]
    [Tooltip("Gizmo wireframe colour in the Scene view (does not appear at runtime).")]
    public Color gizmoColor = new Color(0.3f, 0.85f, 1f, 1f);

    [Tooltip("Also draw the gizmo when the object is not selected.")]
    public bool drawGizmoAlways = true;

    const string WallsRootName = "WorkspaceWalls";

    void Awake()
    {
        Build();
    }

    /// <summary>
    /// Re-create the wall colliders from the current Inspector values.
    /// Right-click the component header in the Inspector → "Rebuild Walls"
    /// to refresh after changing center / size in the editor.
    /// </summary>
    [ContextMenu("Rebuild Walls")]
    public void Build()
    {
        Transform wallsRoot = transform.Find(WallsRootName);
        if (wallsRoot == null)
        {
            var go = new GameObject(WallsRootName);
            go.transform.SetParent(transform, false);
            wallsRoot = go.transform;
        }
        else
        {
            for (int i = wallsRoot.childCount - 1; i >= 0; i--)
            {
                if (Application.isPlaying)
                    Destroy(wallsRoot.GetChild(i).gameObject);
                else
                    DestroyImmediate(wallsRoot.GetChild(i).gameObject);
            }
        }

        wallsRoot.localPosition = Vector3.zero;
        wallsRoot.localRotation = Quaternion.identity;
        wallsRoot.localScale = Vector3.one;
        wallsRoot.gameObject.hideFlags = HideFlags.DontSave;

        float wt = Mathf.Max(0.001f, wallThickness);
        Vector3 s = new Vector3(
            Mathf.Max(0.001f, size.x),
            Mathf.Max(0.001f, size.y),
            Mathf.Max(0.001f, size.z));

        float halfX = s.x * 0.5f;
        float halfY = s.y * 0.5f;
        float halfZ = s.z * 0.5f;

        // Floor — sits at the bottom of the volume.
        AddWall(wallsRoot, "Floor",
            new Vector3(center.x, center.y - halfY + wt * 0.5f, center.z),
            new Vector3(s.x, wt, s.z));

        if (includeCeiling)
        {
            AddWall(wallsRoot, "Ceiling",
                new Vector3(center.x, center.y + halfY - wt * 0.5f, center.z),
                new Vector3(s.x, wt, s.z));
        }

        // Side walls — span the full Y so objects can't escape vertically along
        // the seams. Thickness is along the wall's normal.
        AddWall(wallsRoot, "Wall_PosX",
            new Vector3(center.x + halfX - wt * 0.5f, center.y, center.z),
            new Vector3(wt, s.y, s.z));

        AddWall(wallsRoot, "Wall_NegX",
            new Vector3(center.x - halfX + wt * 0.5f, center.y, center.z),
            new Vector3(wt, s.y, s.z));

        AddWall(wallsRoot, "Wall_PosZ",
            new Vector3(center.x, center.y, center.z + halfZ - wt * 0.5f),
            new Vector3(s.x, s.y, wt));

        AddWall(wallsRoot, "Wall_NegZ",
            new Vector3(center.x, center.y, center.z - halfZ + wt * 0.5f),
            new Vector3(s.x, s.y, wt));
    }

    void AddWall(Transform parent, string name, Vector3 localCenter, Vector3 wallSize)
    {
        var go = new GameObject(name);
        go.transform.SetParent(parent, false);
        go.transform.localPosition = localCenter;
        go.transform.localRotation = Quaternion.identity;
        go.layer = gameObject.layer;
        var box = go.AddComponent<BoxCollider>();
        box.center = Vector3.zero;
        box.size = wallSize;
        box.isTrigger = false;
    }

    void OnDrawGizmos()
    {
        if (drawGizmoAlways) DrawBoundsGizmo(0.8f);
    }

    void OnDrawGizmosSelected()
    {
        DrawBoundsGizmo(1f);
    }

    void DrawBoundsGizmo(float intensity)
    {
        Gizmos.matrix = transform.localToWorldMatrix;

        // Solid translucent floor tint so the play area is obvious in the Scene view.
        {
            var fillColor = gizmoColor;
            fillColor.a = 0.12f * intensity;
            Gizmos.color = fillColor;
            Vector3 floorCenter = new Vector3(center.x, center.y - size.y * 0.5f, center.z);
            Vector3 floorSize = new Vector3(size.x, 0.001f, size.z);
            Gizmos.DrawCube(floorCenter, floorSize);
        }

        // Bright wireframe of the full bounds.
        var line = gizmoColor;
        line.a = Mathf.Clamp01(line.a * intensity);
        Gizmos.color = line;
        Gizmos.DrawWireCube(center, size);

        // Tick marks at the floor corners so the play area is easy to spot at a glance.
        Vector3 s = size * 0.5f;
        float t = Mathf.Min(s.x, s.z) * 0.1f;
        Vector3 floorY = new Vector3(0, -s.y, 0);
        Vector3[] corners = {
            center + floorY + new Vector3( s.x, 0,  s.z),
            center + floorY + new Vector3(-s.x, 0,  s.z),
            center + floorY + new Vector3( s.x, 0, -s.z),
            center + floorY + new Vector3(-s.x, 0, -s.z),
        };
        foreach (var p in corners)
        {
            Gizmos.DrawLine(p, p + Vector3.up * t);
        }
    }
}
