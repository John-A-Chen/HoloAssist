using UnityEngine;
using TMPro;

[ExecuteInEditMode]
public class RobotHUD : MonoBehaviour
{
    [Header("References")]
    public RobotController controller;

    [Header("Layout")]
    public float distanceFromCamera = 1.5f;
    public Vector3 offset = new Vector3(0.3f, -0.12f, 0f);
    public float followSpeed = 2.5f;
    public float panelWidth = 0.55f;
    public float panelHeight = 0.14f;
    public float headerHeight = 0.035f;
    public float padding = 0.01f;

    [Header("Style — matches RobotDataPanel")]
    public Color panelColor = new Color(0.05f, 0.05f, 0.12f, 0.9f);
    public Color headerColor = new Color(0.1f, 0.15f, 0.3f, 0.95f);
    public Color titleColor = new Color(0.6f, 0.8f, 1.0f, 1f);
    public Color accentColor = new Color(0.2f, 0.6f, 1.0f, 1f);

    private TextMeshPro titleLabel;
    private TextMeshPro controlsLabel;
    private TextMeshPro gripperLabel;
    private Transform cam;
    private TMP_FontAsset cachedFont;

    private static readonly string[] jointDisplayNames =
    {
        "Shoulder Pan",
        "Shoulder Lift",
        "Elbow",
        "Wrist 1",
        "Wrist 2",
        "Wrist 3"
    };

    private bool built = false;

    void OnEnable()
    {
        // Destroy old children to avoid duplicates on recompile/re-enable in editor
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            if (Application.isPlaying)
                Destroy(transform.GetChild(i).gameObject);
            else
                DestroyImmediate(transform.GetChild(i).gameObject);
        }
        built = false;
        BuildHUD();
    }

    void Start()
    {
        if (!built) BuildHUD();
    }

    void BuildHUD()
    {
        if (built) return;

        // Camera.main is often null in XR (camera not tagged MainCamera)
        cam = FindXRCamera();
        if (cam == null && Application.isPlaying)
        {
            Debug.LogError("[RobotHUD] No camera found! HUD will not render.");
            return;
        }

        // Cache a TMP font — needed when creating TextMeshPro at runtime
        cachedFont = Resources.Load<TMP_FontAsset>("Fonts & Materials/LiberationSans SDF");
        if (cachedFont == null)
        {
            // Try finding any loaded TMP font
            var fonts = Resources.FindObjectsOfTypeAll<TMP_FontAsset>();
            if (fonts.Length > 0) cachedFont = fonts[0];
        }

        // Background quad (dark blue, low render queue so it stays behind text)
        CreateQuad("HUDBackground", Vector3.zero,
            new Vector2(panelWidth, panelHeight), panelColor, 2950);

        // Header bar
        float topY = panelHeight / 2f - headerHeight / 2f;
        CreateQuad("HUDHeader", new Vector3(0f, topY, -0.005f),
            new Vector2(panelWidth, headerHeight), headerColor, 3000);

        // Accent line under header
        float accentY = topY - headerHeight / 2f - padding * 0.5f;
        CreateQuad("HUDAccent", new Vector3(0f, accentY, -0.01f),
            new Vector2(panelWidth - padding * 2f, 0.002f), accentColor, 3000);

        // Title (in header bar) — sized for compact panel
        titleLabel = CreateLabel("HUDTitle", new Vector3(0f, topY, -0.02f), 0.22f);
        titleLabel.fontStyle = FontStyles.Bold;
        titleLabel.color = titleColor;

        // Controls label (middle)
        controlsLabel = CreateLabel("HUDControls", new Vector3(0f, 0f, -0.02f), 0.16f);
        controlsLabel.color = new Color(0.85f, 0.88f, 0.92f);

        // Gripper label (bottom)
        gripperLabel = CreateLabel("HUDGripper", new Vector3(0f, -panelHeight / 2f + padding + 0.022f, -0.02f), 0.18f);

        if (cam != null) UpdatePosition(true);
        built = true;
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
            if (Application.isPlaying) Destroy(col);
            else DestroyImmediate(col);
        }

        var shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default");
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

    Transform FindXRCamera()
    {
        // Try tagged main camera first
        if (Camera.main != null) return Camera.main.transform;

        // Search for XR camera by typical hierarchy names
        string[] xrCameraNames = { "Main Camera", "CenterEyeAnchor", "Camera", "XR Camera" };
        foreach (var name in xrCameraNames)
        {
            var obj = GameObject.Find(name);
            if (obj != null && obj.GetComponent<Camera>() != null)
                return obj.transform;
        }

        // Last resort: find any enabled camera
        foreach (var c in FindObjectsOfType<Camera>())
        {
            if (c.enabled && c.gameObject.activeInHierarchy)
                return c.transform;
        }

        return null;
    }

    Material CreateBackgroundMaterial()
    {
        // Try URP Unlit first, fall back to built-in shaders
        string[] shaderNames = {
            "Universal Render Pipeline/Unlit",
            "Unlit/Color",
            "UI/Default"
        };

        Shader shader = null;
        foreach (var name in shaderNames)
        {
            shader = Shader.Find(name);
            if (shader != null) break;
        }

        if (shader == null)
        {
            Debug.LogWarning("[RobotHUD] No suitable shader found for background, using fallback.");
            shader = Shader.Find("Hidden/InternalErrorShader");
        }

        var mat = new Material(shader);

        if (shader.name.Contains("Universal Render Pipeline"))
        {
            mat.SetFloat("_Surface", 1);
            mat.SetFloat("_Blend", 0);
            mat.SetColor("_BaseColor", new Color(0.05f, 0.05f, 0.1f, 0.85f));
            mat.SetFloat("_ZWrite", 0);
            mat.SetOverrideTag("RenderType", "Transparent");
            mat.renderQueue = 3000;
            mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        }
        else
        {
            mat.color = new Color(0.05f, 0.05f, 0.1f, 0.85f);
            mat.renderQueue = 3000;
        }

        return mat;
    }

    TextMeshPro CreateLabel(string name, Vector3 localPos, float fontSize)
    {
        var obj = new GameObject(name);
        obj.transform.SetParent(transform, false);
        obj.transform.localPosition = localPos;
        var tmp = obj.AddComponent<TextMeshPro>();
        if (cachedFont != null) tmp.font = cachedFont;
        tmp.fontSize = fontSize;
        tmp.alignment = TextAlignmentOptions.Center;
        tmp.enableWordWrapping = false;
        tmp.overflowMode = TextOverflowModes.Overflow;
        tmp.color = Color.white;
        // Match RobotDataPanel: width = panel - 2*padding, height = rowHeight (~0.045)
        tmp.rectTransform.sizeDelta = new Vector2(panelWidth - padding * 2f, 0.045f);
        // Force text to render on top of all panel quads
        if (tmp.fontSharedMaterial != null)
        {
            var mat = new Material(tmp.fontSharedMaterial);
            mat.renderQueue = 3500;
            tmp.fontMaterial = mat;
        }
        return tmp;
    }

    void LateUpdate()
    {
        // Only follow camera and update text in Play mode (so panel stays where placed in editor)
        if (!Application.isPlaying) return;
        if (controller == null || cam == null) return;
        UpdatePosition(false);
        UpdateText();
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

    void UpdateText()
    {
        if (controller.CurrentMode == RobotController.ControlMode.HandGuide)
        {
            bool active = controller.IsHandGuideActive;
            titleLabel.text = active ? "HAND GUIDE  ●  TRACKING" : "HAND GUIDE  ○  READY";
            titleLabel.color = active ? new Color(1f, 0.4f, 0.4f) : new Color(0.9f, 0.6f, 0.9f);

            controlsLabel.text = "R-Grip: Hold to move robot  |  Y: Options  |  Menu: cycle mode";
        }
        else if (controller.CurrentMode == RobotController.ControlMode.DirectJoint)
        {
            int idx = controller.SelectedJoint;
            string jointName = (idx >= 0 && idx < jointDisplayNames.Length)
                ? jointDisplayNames[idx] : controller.SelectedJointName;

            titleLabel.text = $"DIRECT JOINT  |  {jointName}  ({idx + 1}/6)";
            titleLabel.color = new Color(1f, 0.8f, 0.3f);

            controlsLabel.text = "R-Stick Y: Jog  |  A/B: Switch Joint  |  Y: Options";
        }
        else if (controller.CurrentRMRCSubMode == RobotController.RMRCSubMode.Translate)
        {
            titleLabel.text = "RMRC  TRANSLATE";
            titleLabel.color = new Color(0.4f, 0.9f, 0.4f);

            controlsLabel.text = "R-Stick: XY  |  L-Stick: Up/Down + Yaw  |  Y: Options (RMRC Mode)";
        }
        else
        {
            titleLabel.text = "RMRC  ROTATE";
            titleLabel.color = new Color(0.5f, 0.7f, 1f);

            controlsLabel.text = "R-Stick: Pitch/Roll  |  L-Stick X: Yaw  |  Y: Options (RMRC Mode)";
        }

        // Gripper + lock status — shown in all modes
        float g = controller.GripperValue;
        int pct = Mathf.RoundToInt(g * 100f);
        string bar = new string('|', Mathf.RoundToInt(g * 10f)).PadRight(10, '.');
        string lockIcon = controller.IsEELockedDown ? "  |  LOCK ▼" : "";
        gripperLabel.text = $"Gripper [{bar}] {pct}%{lockIcon}";
        Color gripColor = (g < 0.05f) ? new Color(0.5f, 0.8f, 0.5f)
                         : (g > 0.9f)  ? new Color(1f, 0.4f, 0.3f)
                         : new Color(1f, 0.85f, 0.4f);
        gripperLabel.color = controller.IsEELockedDown ? new Color(0.4f, 0.8f, 1f) : gripColor;
    }
}
