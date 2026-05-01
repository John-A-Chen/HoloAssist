using UnityEngine;
using TMPro;
using System.Collections.Generic;

/// <summary>
/// World-space XR panel that displays bin detection status.
/// Styled to match RobotDataPanel. Floats in the user's view.
/// Auto-discovers all BinDetector components in the scene.
/// </summary>
[ExecuteInEditMode]
public class BinStatusPanel : MonoBehaviour
{
    [Header("Layout")]
    public float distanceFromCamera = 1.8f;
    public Vector3 offset = new Vector3(0.45f, 0.0f, 0f); // right side (opposite of data panel)
    public float followSpeed = 2.5f;

    [Header("Panel Dimensions")]
    public float panelWidth = 0.5f;
    public float headerHeight = 0.06f;
    public float rowHeight = 0.045f;
    public float fontSize = 0.25f;
    public float headerFontSize = 0.3f;
    public float padding = 0.015f;

    [Header("Colors — matches RobotDataPanel")]
    public Color panelColor = new Color(0.05f, 0.05f, 0.12f, 0.9f);
    public Color headerColor = new Color(0.1f, 0.15f, 0.3f, 0.95f);
    public Color titleColor = new Color(0.6f, 0.8f, 1.0f, 1f);
    public Color labelColor = new Color(0.6f, 0.65f, 0.7f, 1f);
    public Color valueColor = Color.white;
    public Color accentColor = new Color(0.2f, 0.6f, 1.0f, 1f);
    public Color emptyColor = new Color(0.6f, 0.6f, 0.6f, 1f);
    public Color occupiedColor = new Color(0.2f, 1f, 0.4f, 1f);

    private Transform cam;
    private Dictionary<string, TextMeshPro> valueTexts = new Dictionary<string, TextMeshPro>();
    private BinDetector[] detectors;
    private bool built;
    private float panelHeight;

    void OnEnable()
    {
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
    }

    void Rebuild()
    {
        if (built) return;
        cam = Camera.main != null ? Camera.main.transform : null;

        // Find all bin detectors
        detectors = FindObjectsByType<BinDetector>(FindObjectsSortMode.None);

        BuildPanel();
        if (cam != null)
            UpdatePosition(true);
        built = true;
    }

    void BuildPanel()
    {
        float zBg = 0f;
        float zQuad = -0.02f;
        float zText = -0.05f;

        // Calculate height
        int binRows = Mathf.Max(detectors != null ? detectors.Length : 0, 1);
        panelHeight = headerHeight + padding * 2 // header + accent
            + binRows * rowHeight
            + padding * 2; // margins

        float yPos = panelHeight / 2f;

        // Header
        yPos -= headerHeight / 2f;
        CreateQuad("Header", new Vector3(0f, yPos, zQuad),
            new Vector2(panelWidth, headerHeight), headerColor);
        CreateText("title", "BIN STATUS", new Vector3(0f, yPos, zText),
            headerFontSize, titleColor, TextAlignmentOptions.Center, panelWidth - padding * 2);
        yPos -= headerHeight / 2f + padding;

        // Accent line
        CreateQuad("AccentLine", new Vector3(0f, yPos, zQuad),
            new Vector2(panelWidth - padding * 2, 0.003f), accentColor);
        yPos -= padding;

        // Bin rows
        if (detectors == null || detectors.Length == 0)
        {
            yPos -= rowHeight / 2f;
            CreateRow("bin_none", "No bins found", "--", ref yPos);
        }
        else
        {
            for (int i = 0; i < detectors.Length; i++)
            {
                yPos -= (i == 0) ? rowHeight / 2f : 0f;
                string key = $"bin_{i}";
                CreateRow(key, detectors[i].binName, "Empty", ref yPos);
                if (valueTexts.ContainsKey(key))
                    valueTexts[key].color = emptyColor;
            }
        }

        // Background
        CreateQuad("Background", Vector3.zero,
            new Vector2(panelWidth, panelHeight), panelColor);
    }

    void CreateRow(string key, string label, string value, ref float yPos)
    {
        float halfW = panelWidth / 2f;
        float labelX = -halfW + padding;
        float valueX = halfW - padding;
        float zText = -0.05f;
        float colWidth = halfW - padding;

        CreateText($"{key}_label", label,
            new Vector3(labelX + colWidth / 2f, yPos, zText),
            fontSize, labelColor, TextAlignmentOptions.Left, colWidth);

        var valTmp = CreateText($"{key}_value", value,
            new Vector3(valueX - colWidth / 2f, yPos, zText),
            fontSize, valueColor, TextAlignmentOptions.Right, colWidth);

        valueTexts[key] = valTmp;
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

        return tmp;
    }

    void CreateQuad(string name, Vector3 localPos, Vector2 size, Color color)
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
        mat.renderQueue = name == "Background" ? 2950 : 3000;
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        quad.GetComponent<Renderer>().material = mat;
    }

    void LateUpdate()
    {
        if (!built) Rebuild();
        if (!Application.isPlaying) return;
        if (cam == null) return;

        UpdatePosition(false);
        UpdateData();
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
        if (detectors == null) return;

        for (int i = 0; i < detectors.Length; i++)
        {
            string key = $"bin_{i}";
            if (!valueTexts.ContainsKey(key) || detectors[i] == null) continue;

            var txt = valueTexts[key];
            if (detectors[i].HasObjects)
            {
                txt.text = detectors[i].Status;
                txt.color = occupiedColor;
            }
            else
            {
                txt.text = "Empty";
                txt.color = emptyColor;
            }
        }
    }

    /// <summary>
    /// Called by BinDetector to update status immediately.
    /// </summary>
    public void SetBinStatus(string binName, string status)
    {
        // Find matching row by bin name
        if (detectors == null) return;
        for (int i = 0; i < detectors.Length; i++)
        {
            if (detectors[i].binName == binName)
            {
                string key = $"bin_{i}";
                if (valueTexts.ContainsKey(key))
                {
                    valueTexts[key].text = status;
                    valueTexts[key].color = status == "Empty" ? emptyColor : occupiedColor;
                }
                return;
            }
        }
    }
}
