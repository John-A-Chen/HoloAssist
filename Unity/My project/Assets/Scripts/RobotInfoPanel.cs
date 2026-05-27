using UnityEngine;
using TMPro;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

/// <summary>
/// World-space XR data panel — built on RobotHUD's proven /joint_states subscription
/// pattern but laid out as multi-row text like RobotDataPanel.
///
/// Self-contained: subscribes directly to /joint_states and /tcp_pose_broadcaster/pose
/// over the ROS-TCP bridge. No JointStateSubscriber dependency, no picker logic,
/// no Transform-based FK. If a topic reaches the Quest, the corresponding row
/// updates.
///
/// Use this instead of RobotDataPanel when you need the multi-row panel layout
/// but want the HUD's update reliability.
/// </summary>
[ExecuteInEditMode]
public class RobotInfoPanel : MonoBehaviour
{
    [Header("References (optional)")]
    [Tooltip("Drag the RobotController to show mode / sub-mode info. Optional.")]
    public RobotController controller;

    [Header("Layout")]
    public float distanceFromCamera = 1.8f;
    public Vector3 offset = new Vector3(-0.45f, 0.0f, 0f);
    public float followSpeed = 2.5f;

    [Header("Panel Dimensions")]
    public float panelWidth = 0.8f;
    public float headerHeight = 0.06f;
    public float rowHeight = 0.045f;
    public float fontSize = 0.25f;
    public float headerFontSize = 0.3f;
    public float padding = 0.015f;

    [Header("Colors")]
    public Color panelColor = new Color(0.05f, 0.05f, 0.12f, 0.9f);
    public Color headerColor = new Color(0.1f, 0.15f, 0.3f, 0.95f);
    public Color titleColor = new Color(0.6f, 0.8f, 1.0f, 1f);
    public Color labelColor = new Color(0.6f, 0.65f, 0.7f, 1f);
    public Color valueColor = Color.white;
    public Color accentColor = new Color(0.2f, 0.6f, 1.0f, 1f);

    // Data state — populated by direct ROS subscriptions, read by the renderer.
    private static readonly string[] rosJointNames =
    {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    };
    private static readonly string[] displayJointNames =
    {
        "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3",
    };
    private float[] jointAnglesDeg = new float[6];
    private Vector3 eePosROS = Vector3.zero;
    private Vector3 eeRotEulerROS = Vector3.zero;
    private bool jointSubscribed = false;
    private bool eePoseSubscribed = false;
    private float lastJointMsgRealtime = -1f;
    private float lastEEPoseMsgRealtime = -1f;

    // Renderer state
    private Transform cam;
    private Dictionary<string, TextMeshPro> cells = new Dictionary<string, TextMeshPro>();
    private bool built = false;
    private TMP_FontAsset cachedFont;

    const float FreshnessSeconds = 1.0f;

    // ────────────────────────────────────────────────────────────────────
    // Lifecycle
    // ────────────────────────────────────────────────────────────────────

    void OnEnable()
    {
        // Tear down any stale children from a previous build / recompile.
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            var child = transform.GetChild(i).gameObject;
            if (Application.isPlaying) Destroy(child); else DestroyImmediate(child);
        }
        cells.Clear();
        built = false;
    }

    void Start()
    {
        if (!built) Build();
        if (Application.isPlaying)
        {
            TrySubscribeJointStates();
            TrySubscribeEEPose();
        }
    }

    void LateUpdate()
    {
        if (!built) Build();
        if (!Application.isPlaying) return;

        // Self-heal subscriptions — ROSConnection may not be ready at Start.
        if (!jointSubscribed) TrySubscribeJointStates();
        if (!eePoseSubscribed) TrySubscribeEEPose();

        if (cam == null) cam = FindXRCamera();

        RenderText();

        if (cam != null) FollowCamera();
    }

    // ────────────────────────────────────────────────────────────────────
    // ROS subscriptions — identical pattern to RobotHUD which is proven working.
    // ────────────────────────────────────────────────────────────────────

    void TrySubscribeJointStates()
    {
        if (jointSubscribed) return;
        var ros = ROSConnection.GetOrCreateInstance();
        if (ros == null) return;
        ros.Subscribe<JointStateMsg>("/joint_states", OnJointState);
        jointSubscribed = true;
        Debug.Log("[RobotInfoPanel] Subscribed to /joint_states");
    }

    void TrySubscribeEEPose()
    {
        if (eePoseSubscribed) return;
        var ros = ROSConnection.GetOrCreateInstance();
        if (ros == null) return;
        ros.Subscribe<PoseStampedMsg>("/tcp_pose_broadcaster/pose", OnEEPose);
        eePoseSubscribed = true;
        Debug.Log("[RobotInfoPanel] Subscribed to /tcp_pose_broadcaster/pose");
    }

    void OnJointState(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            for (int j = 0; j < rosJointNames.Length; j++)
            {
                if (msg.name[i] == rosJointNames[j])
                {
                    jointAnglesDeg[j] = (float)(msg.position[i] * Mathf.Rad2Deg);
                    break;
                }
            }
        }
        lastJointMsgRealtime = Time.realtimeSinceStartup;
    }

    void OnEEPose(PoseStampedMsg msg)
    {
        eePosROS = new Vector3(
            (float)msg.pose.position.x,
            (float)msg.pose.position.y,
            (float)msg.pose.position.z);
        var q = new Quaternion(
            (float)msg.pose.orientation.x,
            (float)msg.pose.orientation.y,
            (float)msg.pose.orientation.z,
            (float)msg.pose.orientation.w);
        eeRotEulerROS = q.eulerAngles;
        lastEEPoseMsgRealtime = Time.realtimeSinceStartup;
    }

    // ────────────────────────────────────────────────────────────────────
    // Rendering
    // ────────────────────────────────────────────────────────────────────

    void RenderText()
    {
        float now = Time.realtimeSinceStartup;
        bool jointsFresh = lastJointMsgRealtime > 0f && (now - lastJointMsgRealtime) < FreshnessSeconds;
        bool eeFresh = lastEEPoseMsgRealtime > 0f && (now - lastEEPoseMsgRealtime) < FreshnessSeconds;

        // Connection / status row
        string status;
        if (jointsFresh || eeFresh) status = "Connected";
        else if (jointSubscribed || eePoseSubscribed) status = "Waiting for ROS";
        else status = "Disconnected";
        SetCell("connection", status);

        // Mode row (optional, requires controller)
        if (controller != null)
            SetCell("mode", controller.CurrentMode.ToString());

        // Joint angles
        for (int i = 0; i < displayJointNames.Length; i++)
        {
            SetCell($"joint_{i}", $"{jointAnglesDeg[i]:F1}°");
        }

        // EE pose (ROS frame: X forward, Y left, Z up)
        SetCell("ee_pos", $"{eePosROS.x:F3}, {eePosROS.y:F3}, {eePosROS.z:F3}");
        SetCell("ee_rot", $"{eeRotEulerROS.x:F1}, {eeRotEulerROS.y:F1}, {eeRotEulerROS.z:F1}");
    }

    /// <summary>
    /// Write text to a cell by key. If the cached TMP is destroyed, trigger a
    /// rebuild on the next frame so dead refs don't permanently freeze the row.
    /// </summary>
    void SetCell(string key, string value)
    {
        if (!cells.TryGetValue(key, out var tmp))
            return;
        if (tmp == null)
        {
            // Dead reference — force a full rebuild next frame.
            cells.Clear();
            built = false;
            return;
        }
        // SetText() always triggers a TMP mesh refresh, unlike .text which can
        // silently skip the regen when TMP's layout invalidation didn't fire.
        tmp.SetText(value);
    }

    /// <summary>Refresh externally — e.g. from RadialMenu after a mode click.</summary>
    public void ForceRefresh()
    {
        if (!Application.isPlaying || !built) return;
        RenderText();
    }

    // ────────────────────────────────────────────────────────────────────
    // Panel construction
    // ────────────────────────────────────────────────────────────────────

    void Build()
    {
        if (built) return;

        cam = FindXRCamera();
        cachedFont = Resources.Load<TMP_FontAsset>("Fonts & Materials/LiberationSans SDF");
        if (cachedFont == null)
        {
            var fonts = Resources.FindObjectsOfTypeAll<TMP_FontAsset>();
            if (fonts.Length > 0) cachedFont = fonts[0];
        }

        // Compute panel height: header + connection + (mode?) + joint header + 6 joints + ee header + 2 ee rows + padding.
        int statusRows = (controller != null) ? 2 : 1;
        int dataRows = statusRows + displayJointNames.Length + 2; // + ee_pos + ee_rot
        float panelHeight = headerHeight + padding * 4f + dataRows * rowHeight + rowHeight * 2f; // headers + spacing

        float zBg = 0.01f;
        float zText = -0.02f;

        // Background
        CreateQuad("Background", new Vector3(0, 0, zBg),
            new Vector2(panelWidth, panelHeight), panelColor, 2950);

        // Header
        float topY = panelHeight / 2f - headerHeight / 2f;
        CreateQuad("Header", new Vector3(0, topY, zBg - 0.005f),
            new Vector2(panelWidth, headerHeight), headerColor, 3000);

        // Title text in header
        var title = CreateText("Title", new Vector3(0, topY, zText),
            headerFontSize, titleColor, TextAlignmentOptions.Center, panelWidth - padding * 2f);
        title.SetText("ROBOT INFO");
        title.fontStyle = FontStyles.Bold;

        float yPos = topY - headerHeight / 2f - padding;

        // Status section
        CreateRow("connection", "Status", "Disconnected", ref yPos);
        if (controller != null)
            CreateRow("mode", "Mode", "—", ref yPos);
        yPos -= padding;

        // Joint angles section header
        var jhdr = CreateText("joint_hdr",
            new Vector3(0, yPos, zText), fontSize * 0.85f, accentColor,
            TextAlignmentOptions.Center, panelWidth - padding * 2f);
        jhdr.SetText("JOINT ANGLES (deg)");
        yPos -= rowHeight;

        for (int i = 0; i < displayJointNames.Length; i++)
            CreateRow($"joint_{i}", displayJointNames[i], "0.0°", ref yPos);

        yPos -= padding;

        // EE pose section header
        var ehdr = CreateText("ee_hdr",
            new Vector3(0, yPos, zText), fontSize * 0.85f, accentColor,
            TextAlignmentOptions.Center, panelWidth - padding * 2f);
        ehdr.SetText("END EFFECTOR (base_link, ROS)");
        yPos -= rowHeight;

        CreateRow("ee_pos", "Position", "0, 0, 0", ref yPos);
        CreateRow("ee_rot", "Rotation", "0, 0, 0", ref yPos);

        if (cam != null) FollowCamera();
        built = true;

        if (Application.isPlaying) RenderText();
    }

    void CreateRow(string key, string label, string initialValue, ref float yPos)
    {
        float zText = -0.02f;
        float colWidth = panelWidth - padding * 2f;
        float labelW = colWidth * 0.45f;
        float valueW = colWidth * 0.55f;
        float labelX = -panelWidth / 2f + padding + labelW / 2f;
        float valueX = panelWidth / 2f - padding - valueW / 2f;

        var lblTmp = CreateText($"{key}_label", new Vector3(labelX, yPos, zText),
            fontSize, labelColor, TextAlignmentOptions.MidlineLeft, labelW);
        lblTmp.SetText(label);

        var valTmp = CreateText($"{key}", new Vector3(valueX, yPos, zText),
            fontSize, valueColor, TextAlignmentOptions.MidlineRight, valueW);
        valTmp.SetText(initialValue);

        cells[key] = valTmp;
        cells[$"{key}_label"] = lblTmp;

        yPos -= rowHeight;
    }

    TextMeshPro CreateText(string name, Vector3 localPos, float size, Color color,
                           TextAlignmentOptions alignment, float width)
    {
        var obj = new GameObject(name);
        obj.transform.SetParent(transform, false);
        obj.transform.localPosition = localPos;
        var tmp = obj.AddComponent<TextMeshPro>();
        if (cachedFont != null) tmp.font = cachedFont;
        tmp.fontSize = size;
        tmp.color = color;
        tmp.alignment = alignment;
        tmp.enableWordWrapping = false;
        tmp.overflowMode = TextOverflowModes.Overflow;
        tmp.rectTransform.sizeDelta = new Vector2(width, rowHeight);
        if (tmp.fontSharedMaterial != null)
        {
            var mat = new Material(tmp.fontSharedMaterial);
            mat.renderQueue = 3500;
            tmp.fontMaterial = mat;
        }
        return tmp;
    }

    void CreateQuad(string name, Vector3 localPos, Vector2 size, Color color, int renderQueue)
    {
        var quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quad.name = name;
        quad.transform.SetParent(transform, false);
        quad.transform.localPosition = localPos;
        quad.transform.localScale = new Vector3(size.x, size.y, 1f);
        var col = quad.GetComponent<Collider>();
        if (col != null)
        {
            if (Application.isPlaying) Destroy(col); else DestroyImmediate(col);
        }

        var shader = Shader.Find("Universal Render Pipeline/Unlit")
                  ?? Shader.Find("Unlit/Color")
                  ?? Shader.Find("Sprites/Default");
        var mat = new Material(shader);
        mat.SetFloat("_Surface", 1);
        mat.SetFloat("_Blend", 0);
        mat.SetColor("_BaseColor", color);
        mat.color = color;
        mat.SetFloat("_ZWrite", 0);
        mat.SetOverrideTag("RenderType", "Transparent");
        mat.renderQueue = renderQueue;
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        quad.GetComponent<Renderer>().material = mat;
    }

    // ────────────────────────────────────────────────────────────────────
    // Camera follow
    // ────────────────────────────────────────────────────────────────────

    void FollowCamera()
    {
        if (cam == null) return;
        Vector3 forward = cam.forward;
        Vector3 right = cam.right;
        Vector3 up = cam.up;
        Vector3 targetPos = cam.position
            + forward * distanceFromCamera
            + right * offset.x
            + up * offset.y
            + forward * offset.z;
        Quaternion targetRot = Quaternion.LookRotation(forward, up);

        if (!Application.isPlaying)
        {
            transform.position = targetPos;
            transform.rotation = targetRot;
        }
        else
        {
            float t = followSpeed * Time.deltaTime;
            transform.position = Vector3.Lerp(transform.position, targetPos, t);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, t);
        }
    }

    Transform FindXRCamera()
    {
        if (Camera.main != null) return Camera.main.transform;
        string[] xrCameraNames = { "Main Camera", "CenterEyeAnchor", "Camera", "XR Camera" };
        foreach (var n in xrCameraNames)
        {
            var obj = GameObject.Find(n);
            if (obj != null && obj.GetComponent<Camera>() != null) return obj.transform;
        }
        foreach (var c in FindObjectsOfType<Camera>())
        {
            if (c.enabled && c.gameObject.activeInHierarchy) return c.transform;
        }
        return null;
    }
}
