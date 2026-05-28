using UnityEngine;

/// <summary>
/// Marks a GameObject as a task object for the sorting system. Attach to anything
/// you want bins to track (sphere, prefab spawns, manually placed cubes).
///
/// - className is a free string label (e.g. "red", "tomato", "bomb"). Used for
///   display and for per-class completion modes in TaskTracker.
/// - isGood determines correct/wrong classification when dropped into a bin:
///     true  → green flash + confetti + counted as correct
///     false → red flash + counted as wrong
///
/// Objects without this component are ignored by BinDetector (no feedback fires).
/// </summary>
public class ObjectClass : MonoBehaviour
{
    [Tooltip("Display label. Colour match (\"red\", \"blue\") or specific object (\"bomb\", \"tomato\").")]
    public string className = "object";

    [Tooltip("Good = sorting this is a correct action. Bad = wrong action (shouldn't be sorted into a bin).")]
    public bool isGood = true;

    /// <summary>
    /// Walks up from the collider to find the owning ObjectClass.
    /// Returns null if neither the collider nor any ancestor has one.
    /// </summary>
    public static ObjectClass FindOn(Collider c)
    {
        if (c == null) return null;
        return c.GetComponentInParent<ObjectClass>();
    }
}
