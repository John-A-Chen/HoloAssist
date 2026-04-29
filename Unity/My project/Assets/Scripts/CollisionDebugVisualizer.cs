using UnityEngine;

/// <summary>
/// Debug visualiser for collision protection. Draws joint positions, gripper tip,
/// link midpoints, table boundary, and soft zone using Unity world-space coordinates.
/// Attach to any GameObject; assign the RobotController reference.
/// </summary>
public class CollisionDebugVisualizer : MonoBehaviour
{
    [Tooltip("Reference to RobotController for collision data.")]
    public RobotController robotController;

    [Tooltip("Size of the spheres drawn at collision check points.")]
    public float sphereRadius = 0.015f;

    [Tooltip("Table plane visual extent (meters from robot base).")]
    public float tablePlaneExtent = 0.5f;

    private static readonly Color[] linkColors = {
        Color.yellow,   // shoulder
        Color.green,    // upper arm
        Color.cyan,     // forearm
        Color.blue,     // wrist 1
        Color.magenta,  // wrist 2
        Color.red       // wrist 3
    };

    void Update()
    {
        if (robotController == null || !robotController.HasJointState) return;
        if (robotController.CollisionJoints == null) return;

        DrawCollisionDebug();
    }

    void DrawCollisionDebug()
    {
        Transform[] joints = robotController.CollisionJoints;
        Transform tool0 = robotController.Tool0;
        float tableY = robotController.TableY;
        float hardLimit = tableY + robotController.collisionMargin;
        float softLimit = hardLimit + robotController.collisionSoftZone;
        float gripLen = robotController.gripperLength;

        // Draw joint positions and skeleton
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] == null) continue;
            Vector3 pos = joints[i].position;
            Color zoneColor = GetZoneColor(pos.y, hardLimit, softLimit);
            DrawWireSphere(pos, sphereRadius, linkColors[i]);
            // Small vertical indicator showing collision zone status
            Debug.DrawRay(pos + Vector3.right * sphereRadius * 1.5f, Vector3.up * 0.02f, zoneColor);

            if (i > 0 && joints[i - 1] != null)
                Debug.DrawLine(joints[i - 1].position, pos, Color.grey);
        }

        // Midpoints of long links
        if (joints[0] != null && joints[1] != null)
            DrawCheckPoint((joints[0].position + joints[1].position) * 0.5f, hardLimit, softLimit);
        if (joints[1] != null && joints[2] != null)
            DrawCheckPoint((joints[1].position + joints[2].position) * 0.5f, hardLimit, softLimit);

        // Tool0 + gripper
        if (tool0 != null)
        {
            DrawWireSphere(tool0.position, sphereRadius, Color.white);
            if (joints[5] != null)
                Debug.DrawLine(joints[5].position, tool0.position, Color.grey);

            Vector3 gripDir = tool0.up;
            Vector3 tipPos = tool0.position + gripDir * gripLen;
            Vector3 midGripPos = tool0.position + gripDir * gripLen * 0.5f;

            DrawCheckPoint(tipPos, hardLimit, softLimit);
            DrawCheckPoint(midGripPos, hardLimit, softLimit);
            Debug.DrawLine(tool0.position, tipPos, Color.white);
        }

        // Table planes
        Vector3 center = (joints[0] != null) ? joints[0].position : transform.position;
        center.y = hardLimit;
        DrawHorizontalGrid(center, tablePlaneExtent, new Color(1f, 0.2f, 0.2f, 0.8f));
        center.y = softLimit;
        DrawHorizontalGrid(center, tablePlaneExtent, new Color(1f, 1f, 0f, 0.5f));

        // Drop lines from each joint to table
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] == null) continue;
            Vector3 pos = joints[i].position;
            Vector3 tablePoint = new Vector3(pos.x, hardLimit, pos.z);
            Color dropColor = GetZoneColor(pos.y, hardLimit, softLimit);
            dropColor.a = 0.3f;
            Debug.DrawLine(pos, tablePoint, dropColor);
        }

    }

    void DrawCheckPoint(Vector3 pos, float hardLimit, float softLimit)
    {
        Color c = GetZoneColor(pos.y, hardLimit, softLimit);
        DrawWireSphere(pos, sphereRadius * 0.6f, c);
    }

    Color GetZoneColor(float y, float hardLimit, float softLimit)
    {
        if (y <= hardLimit) return Color.red;
        if (y <= softLimit) return new Color(1f, 0.5f, 0f);
        return Color.green;
    }

    void DrawHorizontalGrid(Vector3 center, float extent, Color color)
    {
        int lines = 5;
        float step = extent * 2f / lines;
        for (int i = 0; i <= lines; i++)
        {
            float offset = -extent + i * step;
            Vector3 a = center + new Vector3(offset, 0, -extent);
            Vector3 b = center + new Vector3(offset, 0, extent);
            Debug.DrawLine(a, b, color);

            a = center + new Vector3(-extent, 0, offset);
            b = center + new Vector3(extent, 0, offset);
            Debug.DrawLine(a, b, color);
        }
    }

    void DrawWireSphere(Vector3 center, float radius, Color color)
    {
        int segments = 12;
        for (int axis = 0; axis < 3; axis++)
        {
            for (int i = 0; i < segments; i++)
            {
                float a0 = i * 2f * Mathf.PI / segments;
                float a1 = (i + 1) * 2f * Mathf.PI / segments;
                float c0 = Mathf.Cos(a0) * radius, s0 = Mathf.Sin(a0) * radius;
                float c1 = Mathf.Cos(a1) * radius, s1 = Mathf.Sin(a1) * radius;

                Vector3 o0, o1;
                if (axis == 0) { o0 = new Vector3(c0, s0, 0); o1 = new Vector3(c1, s1, 0); }
                else if (axis == 1) { o0 = new Vector3(c0, 0, s0); o1 = new Vector3(c1, 0, s1); }
                else { o0 = new Vector3(0, c0, s0); o1 = new Vector3(0, c1, s1); }

                Debug.DrawLine(center + o0, center + o1, color);
            }
        }
    }
}
