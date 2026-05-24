using UnityEngine;
using TMPro;
using System.Collections.Generic;

/// <summary>
/// Semi-transparent cylindrical banner that floats above the robot/trolley.
/// Visible from all angles (4 text labels at 90° intervals around the cylinder).
/// Flashes green when a cube enters a bin. Toggleable from radial menu page 3.
///
/// Setup:
///   1. Create empty GameObject in the scene at the desired height above the rig.
///      (Parent it to RobotRig or trolley if you want it to follow them.)
///   2. Add this component.
///   3. Optional: assign the radial menu's confettiBlaster pattern — auto-binds.
///
/// Not grabbable — colliders are stripped from the generated cylinder.
/// </summary>
public class EventBanner : MonoBehaviour
{
    public static EventBanner Instance { get; private set; }

    [Header("Cylinder")]
    public float radius = 0.5f;
    public float height = 0.25f;
    public Color baseColor = new Color(0.1f, 0.3f, 0.6f, 0.45f);
    public Color flashColor = new Color(0.2f, 1f, 0.35f, 0.75f);

    [Header("Flash")]
    [Tooltip("How long the green flash takes to fade back to base colour.")]
    public float flashDuration = 1.2f;

    [Header("Text")]
    [TextArea]
    public string bannerText = "HOLOASSIST";
    public Color textColor = Color.white;
    public Color flashTextColor = new Color(0.7f, 1f, 0.75f);
    public float textFontSize = 0.6f;
    [Tooltip("Number of text labels arranged around the cylinder.")]
    public int textInstances = 4;

    [Header("Behaviour")]
    [Tooltip("Banner GameObject starts active/visible.")]
    public bool visibleOnStart = true;

    public bool IsVisible => gameObject.activeSelf;

    private Renderer cylRenderer;
    private List<TextMeshPro> labels = new List<TextMeshPro>();
    private float flashEndTime = -1f;

    void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Debug.LogWarning($"[EventBanner] Multiple instances — keeping '{Instance.gameObject.name}', destroying '{gameObject.name}'.");
            Destroy(this);
            return;
        }
        Instance = this;
        Build();
        gameObject.SetActive(visibleOnStart);
    }

    void OnDestroy()
    {
        if (Instance == this) Instance = null;
    }

    void Build()
    {
        // Cylinder primitive — Unity cylinder is 2m tall, 1m diameter at scale 1.
        var cyl = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        cyl.name = "BannerCylinder";
        cyl.transform.SetParent(transform, false);
        Destroy(cyl.GetComponent<Collider>());
        cyl.transform.localScale = new Vector3(radius * 2f, height * 0.5f, radius * 2f);

        var shader = Shader.Find("Universal Render Pipeline/Unlit")
                     ?? Shader.Find("Unlit/Color")
                     ?? Shader.Find("Sprites/Default");
        var mat = new Material(shader);
        mat.SetFloat("_Surface", 1);
        mat.SetFloat("_Blend", 0);
        mat.SetColor("_BaseColor", baseColor);
        mat.color = baseColor;
        mat.SetFloat("_ZWrite", 0);
        mat.SetFloat("_Cull", 0); // render both sides
        mat.SetOverrideTag("RenderType", "Transparent");
        mat.renderQueue = 3000;
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");

        cylRenderer = cyl.GetComponent<Renderer>();
        cylRenderer.material = mat;
        cylRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        cylRenderer.receiveShadows = false;

        // Text labels at 90° intervals, each facing outward.
        for (int i = 0; i < textInstances; i++)
        {
            float angleDeg = (360f / textInstances) * i;
            float angleRad = angleDeg * Mathf.Deg2Rad;

            var tObj = new GameObject($"BannerLabel_{i}");
            tObj.transform.SetParent(transform, false);
            // Position just outside the cylinder surface
            tObj.transform.localPosition = new Vector3(
                Mathf.Sin(angleRad) * (radius + 0.01f),
                0f,
                Mathf.Cos(angleRad) * (radius + 0.01f));
            // Rotate so text's +Z (outward face) points away from cylinder axis
            tObj.transform.localRotation = Quaternion.Euler(0f, angleDeg, 0f);

            var tmp = tObj.AddComponent<TextMeshPro>();
            tmp.text = bannerText;
            tmp.fontSize = textFontSize;
            tmp.color = textColor;
            tmp.alignment = TextAlignmentOptions.Center;
            tmp.fontStyle = FontStyles.Bold;
            tmp.enableWordWrapping = true;
            tmp.overflowMode = TextOverflowModes.Overflow;
            tmp.rectTransform.sizeDelta = new Vector2(radius * 2f, height * 0.9f);

            if (tmp.fontSharedMaterial != null)
            {
                var lblMat = new Material(tmp.fontSharedMaterial);
                lblMat.renderQueue = 3500;
                tmp.fontMaterial = lblMat;
            }

            labels.Add(tmp);
        }
    }

    void LateUpdate()
    {
        if (flashEndTime > 0f)
        {
            float remaining = flashEndTime - Time.time;
            if (remaining <= 0f)
            {
                flashEndTime = -1f;
                ApplyColor(baseColor, textColor);
            }
            else
            {
                float t = Mathf.Clamp01(remaining / flashDuration);
                ApplyColor(Color.Lerp(baseColor, flashColor, t), Color.Lerp(textColor, flashTextColor, t));
            }
        }
    }

    void ApplyColor(Color cylinderColor, Color labelTint)
    {
        if (cylRenderer != null)
        {
            cylRenderer.material.SetColor("_BaseColor", cylinderColor);
            cylRenderer.material.color = cylinderColor;
        }
        foreach (var l in labels)
            if (l != null) l.color = labelTint;
    }

    /// <summary>Trigger a green pulse (used on bin events).</summary>
    public void Flash()
    {
        flashEndTime = Time.time + flashDuration;
    }

    public void SetText(string text)
    {
        bannerText = text;
        foreach (var l in labels)
            if (l != null) l.text = text;
    }

    public void Toggle() => SetVisible(!gameObject.activeSelf);

    public void SetVisible(bool visible)
    {
        gameObject.SetActive(visible);
        Debug.Log($"[EventBanner] {(visible ? "shown" : "hidden")}");
    }

    /// <summary>Static convenience — flash the banner if one exists and is visible.</summary>
    public static void NotifyBinEvent()
    {
        if (Instance != null && Instance.gameObject.activeSelf)
            Instance.Flash();
    }
}
