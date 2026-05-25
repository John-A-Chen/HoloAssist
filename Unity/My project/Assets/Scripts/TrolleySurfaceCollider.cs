using UnityEngine;

/// <summary>
/// Flat invisible BoxCollider that sits just below the trolley tabletop so dropped
/// objects (cubes, balls) come to rest on top of the trolley instead of falling
/// through the mesh gaps. Independent of the existing trolley colliders —
/// place this on its own child GameObject so the digital-twin colliders stay untouched.
///
/// Setup:
///   1. Create an empty GameObject as a child of the trolley:
///      e.g. RobotRig/Trolley/Surface_Top
///   2. Move it to the desired height — just BELOW the visible tabletop surface
///      (a few mm under is fine; objects will rest at this Y).
///   3. Add this component.
///   4. Tune `size` to match the tabletop footprint (X = forward/back, Z = sideways,
///      Y = collider thickness — keep small, e.g. 0.02m).
///   5. Toggle ON/OFF from the radial menu page 4.
///
/// The gizmo always draws (even when the collider is disabled) so you can position
/// it precisely in the editor: green = collider active, red = inactive.
/// </summary>
[ExecuteAlways]
public class TrolleySurfaceCollider : MonoBehaviour
{
    public static TrolleySurfaceCollider Instance { get; private set; }

    [Header("Surface")]
    [Tooltip("Box dimensions in metres (X = forward/back, Z = sideways, Y = thin slab thickness).")]
    public Vector3 size = new Vector3(0.6f, 0.02f, 0.6f);

    [Tooltip("Local offset from this GameObject's origin.")]
    public Vector3 offset = Vector3.zero;

    [Tooltip("Whether the surface is active on scene start.")]
    public bool enabledOnStart = true;

    [Header("Gizmo")]
    public Color gizmoEnabledColor = new Color(0.2f, 1f, 0.4f, 0.30f);
    public Color gizmoDisabledColor = new Color(1f, 0.4f, 0.2f, 0.20f);
    [Tooltip("Draw the gizmo even when the collider is disabled (so you can still see/position it).")]
    public bool drawGizmoWhenDisabled = true;

    private BoxCollider boxCollider;

    public bool IsEnabled => boxCollider != null && boxCollider.enabled;

    void Awake()
    {
        if (Application.isPlaying)
        {
            if (Instance != null && Instance != this)
            {
                Debug.LogWarning($"[TrolleySurfaceCollider] Multiple instances — keeping '{Instance.gameObject.name}', destroying '{gameObject.name}'.");
                Destroy(this);
                return;
            }
            Instance = this;
        }

        EnsureCollider();
        ApplyShape();
        if (Application.isPlaying) boxCollider.enabled = enabledOnStart;
    }

    void OnDestroy()
    {
        if (Instance == this) Instance = null;
    }

    void OnValidate()
    {
        EnsureCollider();
        ApplyShape();
    }

    void EnsureCollider()
    {
        if (boxCollider == null) boxCollider = GetComponent<BoxCollider>();
        if (boxCollider == null) boxCollider = gameObject.AddComponent<BoxCollider>();
        boxCollider.isTrigger = false;
    }

    void ApplyShape()
    {
        if (boxCollider == null) return;
        boxCollider.center = offset;
        boxCollider.size = size;
    }

    public void Toggle() => SetColliderEnabled(!IsEnabled);

    public void SetColliderEnabled(bool enable)
    {
        EnsureCollider();
        if (boxCollider == null) return;
        boxCollider.enabled = enable;
        Debug.Log($"[TrolleySurfaceCollider] {(enable ? "ON" : "OFF")}");
    }

    void OnDrawGizmos()
    {
        bool enabled = boxCollider != null ? boxCollider.enabled : true;
        if (!enabled && !drawGizmoWhenDisabled) return;

        Color fill = enabled ? gizmoEnabledColor : gizmoDisabledColor;
        Color line = new Color(fill.r, fill.g, fill.b, 1f);

        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.color = fill;
        Gizmos.DrawCube(offset, size);
        Gizmos.color = line;
        Gizmos.DrawWireCube(offset, size);
    }
}
