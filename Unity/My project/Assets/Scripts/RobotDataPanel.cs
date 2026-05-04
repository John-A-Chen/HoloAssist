using UnityEngine;
using TMPro;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// <summary>
/// World-space XR panel that displays robot data (joint states, end-effector pose, etc.).
/// Floats in the user's view and follows the camera smoothly.
/// Initially shows placeholder data — later connect to ROS topics.
/// </summary>
[ExecuteInEditMode]
public class RobotDataPanel : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Drag the GameObject with RobotController to auto-read joint data")]
    public RobotController robotController;

    [Tooltip("Drag the tool0 GameObject (end-effector link) to read its pose")]
    public Transform endEffectorTransform;

    [Tooltip("Drag base_link to compute end-effector pose relative to robot base")]
    public Transform robotBase;

    [Tooltip("Drag the JointTFVisualizer to enable toggle button")]
    public JointTFVisualizer tfVisualizer;

    [Tooltip("Drag the PassthroughToggle so the panel can show MR/VR state")]
    public PassthroughToggle passthroughToggle;

    [Header("Layout")]
    [Tooltip("Distance from camera")]
    public float distanceFromCamera = 1.8f;

    [Tooltip("Offset from center of view (left side by default)")]
    public Vector3 offset = new Vector3(-0.45f, 0.0f, 0f);

    [Tooltip("Camera follow speed")]
    public float followSpeed = 2.5f;

    [Header("Panel Dimensions")]
    public float panelWidth = 0.8f;
    public float panelHeight = 0.75f; // auto-calculated at runtime
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

    private Transform cam;
    private Dictionary<string, TextMeshPro> valueTexts = new Dictionary<string, TextMeshPro>();
    private Dictionary<string, string> binStatuses = new Dictionary<string, string>();
    private bool built;

    // ROS joint name → index mapping (matches RobotController order)
    private static readonly string[] rosJointNames =
    {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    };
    private bool rosSubscribed = false;

    // Data fields — set these from external scripts or ROS subscribers
    [HideInInspector] public string[] jointNames = { "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3" };
    [HideInInspector] public float[] jointAngles = new float[6];
    [HideInInspector] public Vector3 endEffectorPos;
    [HideInInspector] public Vector3 endEffectorRot;
    [HideInInspector] public string connectionStatus = "Disconnected";

    void OnEnable()
    {
        // Destroy old children to avoid duplicates on recompile/re-enable
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            if (Application.isPlaying)
                Destroy(transform.GetChild(i).gameObject);
            else
                DestroyImmediate(transform.GetChild(i).gameObject);
        }
        valueTexts.Clear();
        built = false;
    }

    void Start()
    {
        Rebuild();
        if (Application.isPlaying)
            SubscribeToJointStates();
    }

    void SubscribeToJointStates()
    {
        if (rosSubscribed) return;
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("/joint_states", OnJointState);
        rosSubscribed = true;
    }

    void OnJointState(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            for (int j = 0; j < rosJointNames.Length; j++)
            {
                if (msg.name[i] == rosJointNames[j])
                    jointAngles[j] = (float)(msg.position[i] * Mathf.Rad2Deg);
            }
        }
        connectionStatus = "Connected";
    }

    void Rebuild()
    {
        if (built) return;
        cam = FindXRCamera();
        BuildPanel();
        if (cam != null)
            UpdatePosition(true);
        built = true;
    }

    void BuildPanel()
    {
        // Large Z separation to prevent Z-fighting at all angles (especially when panel is tilted)
        float zBg = 0.01f;
        float zQuad = -0.005f;
        float zText = -0.02f;

        // Calculate total height: header + connection + status + joints + EE + toggle + padding
        int statusRows = 4; // mode, gripper, ee_lock, passthrough
        int dataRows = 1 + statusRows + jointNames.Length + 2; // connection + status + joints + ee_pos + ee_rot
        int sectionHeaders = 3; // "STATUS" + "JOINT ANGLES" + "END EFFECTOR"
        int separators = 4; // connection|status, status|joints, joints|ee, ee|toggle
        int toggleRows = 1;
        float totalHeight = headerHeight + padding * 2  // header + accent line gap
            + (dataRows + sectionHeaders + toggleRows) * rowHeight  // all rows
            + separators * padding * 2                   // separator gaps
            + padding * 2;                               // top/bottom margin
        panelHeight = totalHeight;

        // Build top-down from top edge, placing elements relative to center
        float yPos = panelHeight / 2f;

        // Header bar
        yPos -= headerHeight / 2f;
        CreateQuad("Header", new Vector3(0f, yPos, zQuad),
            new Vector2(panelWidth, headerHeight), headerColor, zQuad);
        CreateText("title", "ROBOT STATUS", new Vector3(0f, yPos, zText),
            headerFontSize, titleColor, TextAlignmentOptions.Center, panelWidth - padding * 2);
        yPos -= headerHeight / 2f + padding;

        // Accent line under header
        CreateQuad("AccentLine", new Vector3(0f, yPos, zQuad),
            new Vector2(panelWidth - padding * 2, 0.003f), accentColor, zQuad);
        yPos -= padding;

        // Connection status row
        yPos -= rowHeight / 2f;
        CreateRow("connection", "Status", connectionStatus, ref yPos);

        // Separator
        yPos -= padding;
        CreateQuad("Sep_Conn", new Vector3(0f, yPos, zQuad),
            new Vector2(panelWidth - padding * 4, 0.001f),
            new Color(0.3f, 0.3f, 0.4f, 0.5f), zQuad);
        yPos -= padding;

        // Section: Status (mirrors RobotHUD)
        yPos -= rowHeight / 2f;
        CreateText("status_header", "STATUS", new Vector3(0f, yPos, zText),
            fontSize * 0.85f, accentColor, TextAlignmentOptions.Center, panelWidth - padding * 2);
        yPos -= rowHeight;

        CreateRow("mode", "Mode", "RMRC", ref yPos);
        CreateRow("gripper", "Gripper", "0%", ref yPos);
        CreateRow("ee_lock", "EE Lock", "OFF", ref yPos);
        CreateRow("passthrough", "View", "MR", ref yPos);

        // Separator
        yPos -= padding;
        CreateQuad("Sep_Status", new Vector3(0f, yPos, zQuad),
            new Vector2(panelWidth - padding * 4, 0.001f),
            new Color(0.3f, 0.3f, 0.4f, 0.5f), zQuad);
        yPos -= padding;

        // Section: Joint Angles
        yPos -= rowHeight / 2f;
        CreateText("joint_header", "JOINT ANGLES (deg)", new Vector3(0f, yPos, zText),
            fontSize * 0.85f, accentColor, TextAlignmentOptions.Center, panelWidth - padding * 2);
        yPos -= rowHeight;

        for (int i = 0; i < jointNames.Length; i++)
        {
            CreateRow($"joint_{i}", jointNames[i], "0.0", ref yPos);
        }

        // Separator
        yPos -= padding;
        CreateQuad("Sep2", new Vector3(0f, yPos, zQuad),
            new Vector2(panelWidth - padding * 4, 0.001f),
            new Color(0.3f, 0.3f, 0.4f, 0.5f), zQuad);
        yPos -= padding;

        // Section: End Effector
        yPos -= rowHeight / 2f;
        CreateText("ee_header", "END EFFECTOR", new Vector3(0f, yPos, zText),
            fontSize * 0.85f, accentColor, TextAlignmentOptions.Center, panelWidth - padding * 2);
        yPos -= rowHeight;

        CreateRow("ee_pos", "Position", "0, 0, 0", ref yPos);
        CreateRow("ee_rot", "Rotation", "0, 0, 0", ref yPos);

        // Separator
        yPos -= padding;
        CreateQuad("Sep3", new Vector3(0f, yPos, zQuad),
            new Vector2(panelWidth - padding * 4, 0.001f),
            new Color(0.3f, 0.3f, 0.4f, 0.5f), zQuad);
        yPos -= padding;

        // Toggle TF Axes button
        yPos -= rowHeight / 2f;
        CreateRow("tf_toggle", "TF Axes", "OFF", ref yPos);
        if (valueTexts.ContainsKey("tf_toggle"))
            valueTexts["tf_toggle"].color = new Color(1f, 0.4f, 0.4f, 1f); // red for OFF

        // Now create background sized to actual content
        CreateQuad("Background", Vector3.zero,
            new Vector2(panelWidth, panelHeight), panelColor, zBg);
    }

    void CreateRow(string key, string label, string value, ref float yPos)
    {
        float halfW = panelWidth / 2f;
        float labelX = -halfW + padding;
        float valueX = halfW - padding;
        float zText = -0.05f;
        float colWidth = halfW - padding;

        var labelTmp = CreateText($"{key}_label", label,
            new Vector3(labelX + colWidth / 2f, yPos, zText),
            fontSize, labelColor, TextAlignmentOptions.Left, colWidth);

        var valTmp = CreateText($"{key}_value", value,
            new Vector3(valueX - colWidth / 2f, yPos, zText),
            fontSize, valueColor, TextAlignmentOptions.Right, colWidth);

        valueTexts[key] = valTmp;
        valueTexts[$"{key}_label"] = labelTmp;
        yPos -= rowHeight;
    }

    TextMeshPro CreateText(string name, string content, Vector3 localPos,
        float size, Color color, TextAlignmentOptions align, float width)
    {
        var obj = new GameObject(name);
        obj.transform.SetParent(transform, false);
        obj.transform.localPosition = localPos;

        var tmp = obj.AddComponent<TextMeshPro>();
        tmp.text = content;
        tmp.fontSize = size;
        tmp.color = color;
        tmp.alignment = align;
        tmp.enableWordWrapping = false;
        tmp.overflowMode = TextOverflowModes.Overflow;
        tmp.rectTransform.sizeDelta = new Vector2(width, rowHeight);
        tmp.fontStyle = FontStyles.Normal;

        return tmp;
    }

    void CreateQuad(string name, Vector3 localPos, Vector2 size, Color color, float zOffset)
    {
        var quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quad.name = name;
        quad.transform.SetParent(transform, false);
        quad.transform.localPosition = localPos;
        quad.transform.localScale = new Vector3(size.x, size.y, 1f);

        if (Application.isPlaying)
            Destroy(quad.GetComponent<Collider>());
        else
            DestroyImmediate(quad.GetComponent<Collider>());

        var mat = new Material(Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default"));
        mat.SetFloat("_Surface", 1);
        mat.SetFloat("_Blend", 0);
        mat.SetColor("_BaseColor", color);
        mat.SetFloat("_ZWrite", 0);
        mat.SetOverrideTag("RenderType", "Transparent");
        // Background uses lower queue, accents/separators higher so they always render on top of bg
        mat.renderQueue = name == "Background" ? 2950 : 3000;
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        quad.GetComponent<Renderer>().material = mat;
    }

    void LateUpdate()
    {
        if (!built) Rebuild();

        // Only follow camera and update data during Play mode
        if (!Application.isPlaying) return;

        // Self-healing subscribe — covers edge cases where Start() didn't run in
        // Play mode (e.g. "Disable Domain Reload" project setting) or where
        // ROSConnection wasn't ready yet when Start() fired. SubscribeToJointStates
        // is idempotent (rosSubscribed guard), so calling it every frame is cheap.
        if (!rosSubscribed) SubscribeToJointStates();

        // Re-acquire the XR camera if the cached ref went null (e.g. scene reload,
        // XR rig recreated). Camera.main is unreliable in the MR Template because
        // the eye-anchor camera often isn't tagged MainCamera.
        if (cam == null) cam = FindXRCamera();

        // Always refresh data text — even if the panel can't currently follow the
        // camera, the labels themselves should still reflect live state.
        UpdateData();

        if (cam != null) UpdatePosition(false);
    }

    Transform FindXRCamera()
    {
        if (Camera.main != null) return Camera.main.transform;

        // MR Template's XR camera typically isn't tagged MainCamera. Try common names.
        string[] xrCameraNames = { "Main Camera", "CenterEyeAnchor", "Camera", "XR Camera" };
        foreach (var n in xrCameraNames)
        {
            var obj = GameObject.Find(n);
            if (obj != null && obj.GetComponent<Camera>() != null)
                return obj.transform;
        }

        // Last resort: any active+enabled camera in the scene.
        foreach (var c in FindObjectsOfType<Camera>())
        {
            if (c.enabled && c.gameObject.activeInHierarchy)
                return c.transform;
        }
        return null;
    }

    void UpdatePosition(bool snap)
    {
        Vector3 targetPos = cam.position + cam.forward * distanceFromCamera
                          + cam.right * offset.x + cam.up * offset.y;
        Quaternion targetRot = Quaternion.LookRotation(targetPos - cam.position);

        if (snap)
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

    void UpdateData()
    {
        // Joint angles come directly from OnJointState (ROS subscription).
        // Fall back to RobotController only if not yet subscribed (e.g. in Editor).
        if (!rosSubscribed && robotController != null && robotController.HasJointState)
        {
            var positions = robotController.CurrentPositions;
            for (int i = 0; i < jointAngles.Length && i < positions.Length; i++)
                jointAngles[i] = (float)(positions[i] * Mathf.Rad2Deg);
            connectionStatus = "Connected";
        }
        else if (!rosSubscribed && robotController != null)
        {
            connectionStatus = robotController.HasJointState ? "Connected" : "Disconnected";
        }

        // Auto-read end-effector pose relative to base_link
        if (endEffectorTransform != null && robotBase != null)
        {
            endEffectorPos = robotBase.InverseTransformPoint(endEffectorTransform.position);
            Quaternion relRot = Quaternion.Inverse(robotBase.rotation) * endEffectorTransform.rotation;
            endEffectorRot = relRot.eulerAngles;
        }

        // Connection status
        if (valueTexts.ContainsKey("connection"))
            valueTexts["connection"].text = connectionStatus;

        // Joint angles + selected joint highlight
        int selectedJoint = robotController != null ? robotController.SelectedJoint : -1;
        bool isJointMode = robotController != null &&
            robotController.CurrentMode == RobotController.ControlMode.DirectJoint;

        for (int i = 0; i < jointNames.Length && i < jointAngles.Length; i++)
        {
            string key = $"joint_{i}";
            string labelKey = $"joint_{i}_label";

            if (valueTexts.ContainsKey(key))
                valueTexts[key].text = $"{jointAngles[i]:F1}\u00B0";

            // Highlight selected joint (only in Direct Joint mode)
            bool highlight = isJointMode && (i == selectedJoint);
            Color jointColor = highlight
                ? new Color(1f, 0.85f, 0.2f, 1f)  // gold/yellow when selected
                : labelColor;
            Color jointValueColor = highlight ? new Color(1f, 0.95f, 0.6f, 1f) : valueColor;

            if (valueTexts.ContainsKey(labelKey))
            {
                valueTexts[labelKey].color = jointColor;
                valueTexts[labelKey].text = highlight ? $"> {jointNames[i]}" : jointNames[i];
            }
            if (valueTexts.ContainsKey(key))
                valueTexts[key].color = jointValueColor;
        }

        // End effector
        if (valueTexts.ContainsKey("ee_pos"))
            valueTexts["ee_pos"].text = $"{endEffectorPos.x:F3}, {endEffectorPos.y:F3}, {endEffectorPos.z:F3}";

        if (valueTexts.ContainsKey("ee_rot"))
            valueTexts["ee_rot"].text = $"{endEffectorRot.x:F1}, {endEffectorRot.y:F1}, {endEffectorRot.z:F1}";

        // TF toggle state
        if (valueTexts.ContainsKey("tf_toggle") && tfVisualizer != null)
        {
            bool on = tfVisualizer.showAxes;
            valueTexts["tf_toggle"].text = on ? "ON" : "OFF";
            valueTexts["tf_toggle"].color = on
                ? new Color(0.2f, 1f, 0.4f, 1f)   // green
                : new Color(1f, 0.4f, 0.4f, 1f);   // red
        }

        // Status section — mirrors what the floating RobotHUD shows so the data
        // panel is also a single-glance "where am I" reference.
        UpdateStatusSection();
    }

    /// <summary>
    /// Refresh the data panel's text immediately. Called by RadialMenu after a
    /// page-2 click so the user doesn't have to wait for the next LateUpdate.
    /// </summary>
    public void ForceRefresh()
    {
        if (!Application.isPlaying) return;
        if (!built) return;
        UpdateData();
    }

    void UpdateStatusSection()
    {
        // Mode row
        if (valueTexts.ContainsKey("mode") && robotController != null)
        {
            string modeText;
            Color modeColor;
            switch (robotController.CurrentMode)
            {
                case RobotController.ControlMode.HandGuide:
                    bool tracking = robotController.IsHandGuideActive;
                    modeText = tracking ? "HAND GUIDE  ●  TRACKING" : "HAND GUIDE  ○  READY";
                    modeColor = tracking ? new Color(1f, 0.4f, 0.4f) : new Color(0.9f, 0.6f, 0.9f);
                    break;
                case RobotController.ControlMode.DirectJoint:
                    int idx = robotController.SelectedJoint;
                    string j = (idx >= 0 && idx < jointNames.Length) ? jointNames[idx] : "?";
                    modeText = $"DIRECT JOINT ({idx + 1}/6 {j})";
                    modeColor = new Color(1f, 0.8f, 0.3f);
                    break;
                default: // RMRC
                    bool translate = robotController.CurrentRMRCSubMode == RobotController.RMRCSubMode.Translate;
                    modeText = translate ? "RMRC  TRANSLATE" : "RMRC  ROTATE";
                    modeColor = translate ? new Color(0.4f, 0.9f, 0.4f) : new Color(0.5f, 0.7f, 1f);
                    break;
            }
            valueTexts["mode"].text = modeText;
            valueTexts["mode"].color = modeColor;
        }

        // Gripper percentage
        if (valueTexts.ContainsKey("gripper") && robotController != null)
        {
            float g = robotController.GripperValue;
            int pct = Mathf.RoundToInt(g * 100f);
            valueTexts["gripper"].text = $"{pct}%";
            valueTexts["gripper"].color =
                  (g < 0.05f) ? new Color(0.5f, 0.8f, 0.5f)
                : (g > 0.9f)  ? new Color(1f, 0.4f, 0.3f)
                              : new Color(1f, 0.85f, 0.4f);
        }

        // EE Lock state
        if (valueTexts.ContainsKey("ee_lock") && robotController != null)
        {
            bool locked = robotController.IsEELockedDown;
            valueTexts["ee_lock"].text = locked ? "ON" : "OFF";
            valueTexts["ee_lock"].color = locked
                ? new Color(0.4f, 0.8f, 1f)
                : new Color(0.6f, 0.65f, 0.7f);
        }

        // Passthrough state (MR / VR)
        if (valueTexts.ContainsKey("passthrough"))
        {
            bool mr = passthroughToggle != null && passthroughToggle.PassthroughEnabled;
            valueTexts["passthrough"].text = mr ? "MR" : "VR";
            valueTexts["passthrough"].color = mr
                ? new Color(0.4f, 0.9f, 0.4f)
                : new Color(0.6f, 0.65f, 0.95f);
        }
    }

    /// <summary>
    /// Call this to update joint data from external source (e.g. ROS subscriber).
    /// </summary>
    public void SetJointData(string[] names, float[] angles)
    {
        jointNames = names;
        jointAngles = angles;
    }

    /// <summary>
    /// Call this to update end-effector pose from external source.
    /// </summary>
    public void SetEndEffectorPose(Vector3 position, Vector3 rotationEuler)
    {
        endEffectorPos = position;
        endEffectorRot = rotationEuler;
    }

    /// <summary>
    /// Call this to update connection status text.
    /// </summary>
    public void SetConnectionStatus(string status)
    {
        connectionStatus = status;
    }
}
