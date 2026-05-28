using UnityEngine;

public class CollisionDebugVisualizer : MonoBehaviour
{
    [Tooltip("Reference to MeshCollisionGuard for debug drawing.")]
    public MeshCollisionGuard collisionGuard;

    void OnGUI()
    {
        if (collisionGuard == null) return;

        float scale = collisionGuard.VelocityScale;
        Color c = scale <= 0f ? Color.red : (scale < 1f ? Color.yellow : Color.green);
        string label = scale <= 0f ? "COLLISION BLOCKED" : (scale < 1f ? $"SLOWING ({scale:P0})" : "CLEAR");

        GUI.color = c;
        GUI.Label(new Rect(10, 10, 300, 30), $"Collision: {label}");
        GUI.color = Color.white;
    }
}
