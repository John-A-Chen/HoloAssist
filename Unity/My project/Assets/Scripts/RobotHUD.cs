using UnityEngine;
using TMPro;

public class RobotHUD : MonoBehaviour
{
    [Header("References")]
    public RobotController controller;

    [Header("Layout")]
    public float distanceFromCamera = 1.5f;
    public Vector3 offset = new Vector3(0.3f, -0.2f, 0f);
    public float followSpeed = 3f;
    public float panelWidth = 2.5f;
    public float panelHeight = 0.25f;

    private TextMeshPro titleLabel;
    private TextMeshPro controlsLabel;
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

    void Start()
    {
        // Camera.main is often null in XR (camera not tagged MainCamera)
        cam = FindXRCamera();
        if (cam == null)
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

        // Background quad
        var bg = GameObject.CreatePrimitive(PrimitiveType.Quad);
        bg.transform.SetParent(transform, false);
        bg.transform.localScale = new Vector3(panelWidth + 0.04f, panelHeight + 0.02f, 1f);
        var bgMat = CreateBackgroundMaterial();
        bg.GetComponent<Renderer>().material = bgMat;
        Destroy(bg.GetComponent<Collider>());

        // Title label (top line)
        titleLabel = CreateLabel("HUDTitle", new Vector3(0f, 0.06f, -0.001f), 0.35f);
        titleLabel.fontStyle = FontStyles.Bold;

        // Controls label (below title)
        controlsLabel = CreateLabel("HUDControls", new Vector3(0f, -0.03f, -0.001f), 0.22f);
        controlsLabel.color = new Color(0.85f, 0.88f, 0.92f);

        UpdatePosition(true);
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
        tmp.rectTransform.sizeDelta = new Vector2(panelWidth, 0.12f);
        return tmp;
    }

    void LateUpdate()
    {
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

            controlsLabel.text = "Right Grip: Hold to move robot  |  Menu: RMRC Mode";
        }
        else if (controller.CurrentMode == RobotController.ControlMode.DirectJoint)
        {
            int idx = controller.SelectedJoint;
            string jointName = (idx >= 0 && idx < jointDisplayNames.Length)
                ? jointDisplayNames[idx] : controller.SelectedJointName;

            titleLabel.text = $"DIRECT JOINT  |  {jointName}  ({idx + 1}/6)";
            titleLabel.color = new Color(1f, 0.8f, 0.3f);

            controlsLabel.text = "R-Stick Y: Jog  |  A/B: Switch Joint  |  Menu: Hand Guide";
        }
        else if (controller.CurrentRMRCSubMode == RobotController.RMRCSubMode.Translate)
        {
            titleLabel.text = "RMRC  TRANSLATE";
            titleLabel.color = new Color(0.4f, 0.9f, 0.4f);

            controlsLabel.text = "R-Stick: XY Move  |  L-Stick Y: Up/Down  |  L-Stick X: Yaw  |  X: Rotate  |  Menu: Joint";
        }
        else
        {
            titleLabel.text = "RMRC  ROTATE";
            titleLabel.color = new Color(0.5f, 0.7f, 1f);

            controlsLabel.text = "R-Stick Y: Pitch  |  R-Stick X: Roll  |  L-Stick X: Yaw  |  X: Translate  |  Menu: Joint";
        }
    }
}
