using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

/// <summary>
/// World-space panel for configuring TaskTracker's completion mode.
/// Four buttons (three modes + Reset) arranged horizontally. Pointed at with
/// the right controller, fired with the right trigger — same interaction
/// pattern as RadialMenu.
///
/// Toggle from the radial menu (page 4, "Task Cfg" button) or via SetVisible().
///
/// [ExecuteAlways] makes the panel build its visual layout in the Editor so
/// you can position / scale it without entering Play mode. Input polling and
/// the auto SetActive(visibleOnStart) only fire when the game is running.
/// </summary>
[ExecuteAlways]
public class TaskConfigPanel : MonoBehaviour
{
    public static TaskConfigPanel Instance { get; private set; }

    [Header("Layout")]
    [Tooltip("If true, the panel follows the camera at distanceFromCamera. PanelPlacer disables this on grab so the panel stays where placed.")]
    public bool followCamera = true;
    public float distanceFromCamera = 1.5f;
    public Vector3 offset = new Vector3(0f, -0.15f, 0f);
    public float followSpeed = 2.5f;
    [Tooltip("Width is honoured; height is auto-computed from row content.")]
    public Vector2 panelSize = new Vector2(0.70f, 0.42f);
    public Vector2 buttonSize = new Vector2(0.145f, 0.085f);
    public Vector2 targetButtonSize = new Vector2(0.06f, 0.055f);
    public Vector2 timerButtonSize = new Vector2(0.16f, 0.055f);
    public float padding = 0.015f;
    public float headerHeight = 0.06f;
    public float footerHeight = 0.04f;

    [Header("Style (matches RadialMenu)")]
    public Color panelColor = new Color(0.05f, 0.05f, 0.12f, 0.9f);
    public Color headerColor = new Color(0.1f, 0.15f, 0.3f, 0.95f);
    public Color titleColor = new Color(0.6f, 0.8f, 1.0f, 1f);
    public Color accentColor = new Color(0.2f, 0.6f, 1.0f, 1f);
    public Color buttonOffColor = new Color(0.15f, 0.15f, 0.25f, 0.9f);
    public Color buttonOnColor = new Color(0.15f, 0.7f, 0.3f, 1.0f);
    public Color buttonResetColor = new Color(0.95f, 0.22f, 0.22f, 1.0f);
    public Color highlightColor = new Color(0.25f, 0.35f, 0.55f, 0.95f);
    public Color labelColor = Color.white;
    public float fontSize = 0.20f;
    public float titleFontSize = 0.28f;
    public float footerFontSize = 0.17f;
    public float targetValueFontSize = 0.24f;

    [Header("Behaviour")]
    public bool visibleOnStart = false;

    private class CfgButton
    {
        public string label;
        public Action onClick;
        public Func<bool> isActive; // returns true when this mode is the current
        public bool isReset;
        public int row;             // 0 = mode row, 1 = target row
        public Vector2 size;        // custom per-button size
        public GameObject bgQuad;
        public TextMeshPro labelText;
        public bool isHovered;       // set by XRSimpleInteractable hoverEntered/Exited
    }

    private Transform cam;
    private GameObject root;
    private List<CfgButton> buttons = new List<CfgButton>();
    private bool built = false;
    private TextMeshPro statusText;
    private TextMeshPro targetLabelText;
    private TextMeshPro targetValueText;

    public bool IsVisible => gameObject.activeSelf;

    void Awake()
    {
        // Only register singleton at runtime — in edit mode multiple instances
        // (e.g. when duplicating in the scene) are fine and the static ref
        // shouldn't fight the editor.
        if (!Application.isPlaying) return;

        if (Instance != null && Instance != this)
        {
            Debug.LogWarning($"[TaskConfigPanel] Multiple instances — keeping '{Instance.gameObject.name}', destroying '{gameObject.name}'.");
            Destroy(this);
            return;
        }
        Instance = this;
    }

    void OnEnable()
    {
        // Destroy ALL children, not just the cached `root` reference. Unity
        // serialises procedural GameObjects created under [ExecuteAlways], so
        // a scene save + reload leaves the previous build's TaskCfgRoot in
        // place while our private `root` field comes back null — without this
        // loop we stack new builds on top of old ones (two TaskCfgRoots).
        //
        // Use DestroyImmediate unconditionally. During play-mode transitions
        // `Application.isPlaying` can report `true` while Unity is still in
        // edit-mode for destroy purposes — Destroy() logs "Destroy may not be
        // called from edit mode!" and the child persists. DestroyImmediate
        // works in both states and is safe for these procedural UI quads.
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            var child = transform.GetChild(i).gameObject;
            if (child != null) DestroyImmediate(child);
        }
        root = null;
        buttons.Clear();
        built = false;
        targetLabelText = null;
        targetValueText = null;
        statusText = null;
    }

    void OnDestroy()
    {
        if (Instance == this) Instance = null;
    }

    void Start()
    {
        cam = Camera.main != null ? Camera.main.transform : null;

        // Always build the visual structure so it shows up in the editor.
        Build();

        // Runtime-only: initial SetActive + camera snap.
        if (Application.isPlaying)
        {
            gameObject.SetActive(visibleOnStart);
            if (visibleOnStart) SnapToCamera();
        }
    }

    void Build()
    {
        if (built) return;
        root = new GameObject("TaskCfgRoot");
        root.transform.SetParent(transform, false);

        // Auto-compute panel height so it always wraps row content. Width is
        // honoured from inspector. Required height = title + each row + footer
        // + paddings between them.
        float titleH = headerHeight;
        float requiredH = padding
            + titleH + padding
            + buttonSize.y + padding          // row 0 — modes
            + targetButtonSize.y + padding    // row 1 — target
            + timerButtonSize.y + padding     // row 2 — timer
            + footerHeight + padding;
        if (panelSize.y < requiredH) panelSize.y = requiredH;

        float halfH = panelSize.y / 2f;

        // Background
        CreateQuad(root.transform, "Background", new Vector3(0, 0, 0f),
            new Vector2(panelSize.x, panelSize.y), panelColor, 2950);
        CreateQuad(root.transform, "Header",
            new Vector3(0, halfH - titleH / 2f, -0.001f),
            new Vector2(panelSize.x - padding * 2f, titleH), headerColor, 3000);

        var titleObj = new GameObject("Title");
        titleObj.transform.SetParent(root.transform, false);
        titleObj.transform.localPosition = new Vector3(0, halfH - titleH / 2f, -0.003f);
        var title = titleObj.AddComponent<TextMeshPro>();
        title.text = "TASK MODE";
        title.fontSize = titleFontSize;
        title.color = titleColor;
        title.alignment = TextAlignmentOptions.Center;
        title.fontStyle = FontStyles.Bold;
        title.enableWordWrapping = false;
        title.rectTransform.sizeDelta = new Vector2(panelSize.x, titleH);
        if (title.fontSharedMaterial != null)
        {
            var m = new Material(title.fontSharedMaterial); m.renderQueue = 3500;
            title.fontMaterial = m;
        }

        // Accent line
        CreateQuad(root.transform, "Accent",
            new Vector3(0, halfH - titleH - 0.002f, -0.001f),
            new Vector2(panelSize.x * 0.85f, 0.002f), accentColor, 3010);

        // Row 0 — mode buttons + reset
        AddButton("Fixed\nTarget", row: 0, size: buttonSize,
            onClick: () => TaskTracker.Instance?.SetMode(TaskTracker.CompletionMode.FixedTarget),
            isActive: () => TaskTracker.Instance != null && TaskTracker.Instance.mode == TaskTracker.CompletionMode.FixedTarget);

        AddButton("Sort\nAll", row: 0, size: buttonSize,
            onClick: () => TaskTracker.Instance?.SetMode(TaskTracker.CompletionMode.SortAllGood),
            isActive: () => TaskTracker.Instance != null && TaskTracker.Instance.mode == TaskTracker.CompletionMode.SortAllGood);

        AddButton("One\nEach", row: 0, size: buttonSize,
            onClick: () => TaskTracker.Instance?.SetMode(TaskTracker.CompletionMode.OneOfEachClass),
            isActive: () => TaskTracker.Instance != null && TaskTracker.Instance.mode == TaskTracker.CompletionMode.OneOfEachClass);

        AddButton("Reset", row: 0, size: buttonSize,
            onClick: () => TaskTracker.Instance?.ResetTask(),
            isActive: () => false,
            isReset: true);

        // Row 1 — target count adjuster: [-]  N  [+]
        AddButton("-", row: 1, size: targetButtonSize,
            onClick: () =>
            {
                if (TaskTracker.Instance == null) return;
                TaskTracker.Instance.SetTarget(TaskTracker.Instance.targetCount - 1);
            },
            isActive: () => false);

        AddButton("+", row: 1, size: targetButtonSize,
            onClick: () =>
            {
                if (TaskTracker.Instance == null) return;
                TaskTracker.Instance.SetTarget(TaskTracker.Instance.targetCount + 1);
            },
            isActive: () => false);

        // "TARGET" label + current value text (not buttons — placed in LayoutButtons)
        var labelObj = new GameObject("TargetLabel");
        labelObj.transform.SetParent(root.transform, false);
        targetLabelText = labelObj.AddComponent<TextMeshPro>();
        targetLabelText.text = "TARGET";
        targetLabelText.fontSize = fontSize * 0.85f;
        targetLabelText.color = new Color(0.6f, 0.7f, 0.85f);
        targetLabelText.alignment = TextAlignmentOptions.Right;
        targetLabelText.fontStyle = FontStyles.Bold;
        targetLabelText.enableWordWrapping = false;
        targetLabelText.rectTransform.sizeDelta = new Vector2(0.13f, targetButtonSize.y);
        ApplyTextRenderQueue(targetLabelText);

        var valObj = new GameObject("TargetValue");
        valObj.transform.SetParent(root.transform, false);
        targetValueText = valObj.AddComponent<TextMeshPro>();
        targetValueText.text = "?";
        targetValueText.fontSize = targetValueFontSize;
        targetValueText.color = new Color(1f, 0.85f, 0.3f);
        targetValueText.alignment = TextAlignmentOptions.Center;
        targetValueText.fontStyle = FontStyles.Bold;
        targetValueText.enableWordWrapping = false;
        targetValueText.rectTransform.sizeDelta = new Vector2(0.06f, targetButtonSize.y);
        ApplyTextRenderQueue(targetValueText);

        // Row 2 — Timer toggle + Timer reset
        AddButton("Timer", row: 2, size: timerButtonSize,
            onClick: () => EventBanner.Instance?.ToggleTimer(),
            isActive: () => EventBanner.Instance != null && EventBanner.Instance.IsTimerVisible);

        AddButton("Reset\nTimer", row: 2, size: timerButtonSize,
            onClick: () => EventBanner.Instance?.ResetTimer(),
            isActive: () => false,
            isReset: true);

        LayoutButtons();

        // Status footer text — current mode summary, updated each frame.
        float footY = -halfH + padding + footerHeight / 2f;
        var footObj = new GameObject("StatusFoot");
        footObj.transform.SetParent(root.transform, false);
        footObj.transform.localPosition = new Vector3(0, footY, -0.003f);
        statusText = footObj.AddComponent<TextMeshPro>();
        statusText.text = "—";
        statusText.fontSize = footerFontSize;
        statusText.color = new Color(0.7f, 0.85f, 1f);
        statusText.alignment = TextAlignmentOptions.Center;
        statusText.fontStyle = FontStyles.Bold;
        statusText.enableWordWrapping = false;
        statusText.rectTransform.sizeDelta = new Vector2(panelSize.x - padding * 2f, footerHeight);
        ApplyTextRenderQueue(statusText);

        built = true;
    }

    void AddButton(string label, Action onClick, Func<bool> isActive,
        int row = 0, Vector2 size = default, bool isReset = false)
    {
        Vector2 actualSize = size == default ? buttonSize : size;
        var btn = new CfgButton
        {
            label = label,
            onClick = onClick,
            isActive = isActive,
            isReset = isReset,
            row = row,
            size = actualSize
        };

        btn.bgQuad = CreateQuad(root.transform, $"Btn_{label.Replace("\n", "_")}",
            Vector3.zero, actualSize, isReset ? buttonResetColor : buttonOffColor, 3030);

        // Make the button XR-ray clickable. Each button is its own interactable
        // so the XR ray picks the button (specific) over PanelPlacer's panel-wide
        // XRGrabInteractable (general) — same pattern as CoachingPanel page nav.
        // Only do this at runtime — XRI components don't behave well in edit mode.
        if (Application.isPlaying)
        {
            var col = btn.bgQuad.AddComponent<BoxCollider>();
            // Local space: the Quad's transform scale already encodes the
            // world-space button size, so the collider only needs unit dims.
            col.size = new Vector3(1f, 1f, 0.1f);
            col.isTrigger = false;

            var interactable = btn.bgQuad.AddComponent<XRSimpleInteractable>();
            interactable.colliders.Add(col);
            interactable.selectMode = InteractableSelectMode.Single;

            var captured = btn;
            interactable.hoverEntered.AddListener(_ => captured.isHovered = true);
            interactable.hoverExited.AddListener(_ => captured.isHovered = false);
            interactable.selectEntered.AddListener(_ =>
            {
                Debug.Log($"[TaskConfigPanel] Click → '{captured.label.Replace("\n", " ")}'");
                captured.onClick?.Invoke();
            });
        }

        var labelObj = new GameObject($"Lbl_{label.Replace("\n", "_")}");
        labelObj.transform.SetParent(root.transform, false);
        btn.labelText = labelObj.AddComponent<TextMeshPro>();
        btn.labelText.text = label;
        btn.labelText.fontSize = fontSize;
        btn.labelText.color = labelColor;
        btn.labelText.alignment = TextAlignmentOptions.Center;
        btn.labelText.fontStyle = FontStyles.Bold;
        btn.labelText.enableWordWrapping = true;
        btn.labelText.rectTransform.sizeDelta = new Vector2(actualSize.x, actualSize.y);
        ApplyTextRenderQueue(btn.labelText);

        buttons.Add(btn);
    }

    void ApplyTextRenderQueue(TextMeshPro tmp)
    {
        // TMP can race in edit mode and report a null fontSharedMaterial briefly —
        // fall back to nudging the renderer's sorting order so labels still draw on
        // top of the button bg quads (render queue 3030 in our CreateQuad calls).
        var mr = tmp.GetComponent<MeshRenderer>();
        if (mr != null) mr.sortingOrder = 10;

        if (tmp.fontSharedMaterial != null)
        {
            var m = new Material(tmp.fontSharedMaterial);
            m.renderQueue = 3500;
            tmp.fontMaterial = m;
        }
    }

    void LayoutButtons()
    {
        if (buttons.Count == 0) return;

        float halfH = panelSize.y / 2f;
        float titleH = headerHeight;
        // Row 0 (modes) — just below the title strip
        float row0Y = halfH - titleH - padding - buttonSize.y / 2f;
        // Row 1 (target) — below mode row
        float row1Y = row0Y - buttonSize.y / 2f - padding - targetButtonSize.y / 2f;
        // Row 2 (timer) — below target row
        float row2Y = row1Y - targetButtonSize.y / 2f - padding - timerButtonSize.y / 2f;

        // Row 0 — mode buttons, evenly distributed
        var row0 = buttons.FindAll(b => b.row == 0);
        float totalW0 = 0f;
        foreach (var b in row0) totalW0 += b.size.x;
        totalW0 += Mathf.Max(0, row0.Count - 1) * padding;
        float x0 = -totalW0 / 2f;
        foreach (var b in row0)
        {
            float bx = x0 + b.size.x / 2f;
            b.bgQuad.transform.localPosition = new Vector3(bx, row0Y, -0.002f);
            b.labelText.transform.localPosition = new Vector3(bx, row0Y, -0.004f);
            x0 += b.size.x + padding;
        }

        // Row 1 — TARGET label, [-], value, [+]   (tight horizontal cluster)
        var row1 = buttons.FindAll(b => b.row == 1);
        CfgButton minusBtn = row1.Count > 0 ? row1[0] : null;
        CfgButton plusBtn = row1.Count > 1 ? row1[1] : null;

        float labelW = 0.13f;
        float valueW = 0.06f;
        float btnW = targetButtonSize.x;
        float gap = 0.008f;
        float groupW = labelW + gap + btnW + gap + valueW + gap + btnW;
        float xStart = -groupW / 2f;

        float labelX = xStart + labelW / 2f;
        float minusX = xStart + labelW + gap + btnW / 2f;
        float valueX = xStart + labelW + gap + btnW + gap + valueW / 2f;
        float plusX = xStart + labelW + gap + btnW + gap + valueW + gap + btnW / 2f;

        if (targetLabelText != null)
            targetLabelText.transform.localPosition = new Vector3(labelX, row1Y, -0.004f);
        if (minusBtn != null)
        {
            minusBtn.bgQuad.transform.localPosition = new Vector3(minusX, row1Y, -0.002f);
            minusBtn.labelText.transform.localPosition = new Vector3(minusX, row1Y, -0.004f);
        }
        if (targetValueText != null)
            targetValueText.transform.localPosition = new Vector3(valueX, row1Y, -0.004f);
        if (plusBtn != null)
        {
            plusBtn.bgQuad.transform.localPosition = new Vector3(plusX, row1Y, -0.002f);
            plusBtn.labelText.transform.localPosition = new Vector3(plusX, row1Y, -0.004f);
        }

        // Row 2 — Timer toggle + Reset, evenly distributed
        var row2 = buttons.FindAll(b => b.row == 2);
        float totalW2 = 0f;
        foreach (var b in row2) totalW2 += b.size.x;
        totalW2 += Mathf.Max(0, row2.Count - 1) * padding;
        float x2 = -totalW2 / 2f;
        foreach (var b in row2)
        {
            float bx = x2 + b.size.x / 2f;
            b.bgQuad.transform.localPosition = new Vector3(bx, row2Y, -0.002f);
            b.labelText.transform.localPosition = new Vector3(bx, row2Y, -0.004f);
            x2 += b.size.x + padding;
        }
    }

    GameObject CreateQuad(Transform parent, string name, Vector3 localPos, Vector2 size, Color color, int renderQueue)
    {
        var quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quad.name = name;
        quad.transform.SetParent(parent, false);
        quad.transform.localPosition = localPos;
        quad.transform.localScale = new Vector3(size.x, size.y, 1f);
        Destroy(quad.GetComponent<Collider>());

        var shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color");
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
        return quad;
    }

    void Update()
    {
        // Lazy rebuild in BOTH modes. OnEnable destroys all children every time
        // the GameObject is activated (e.g. radial-menu toggle), and Start only
        // runs once per script lifetime — so without this we'd end up with an
        // empty panel after the first toggle-off-on cycle.
        if (!built) Build();

        if (!Application.isPlaying) return;

        if (cam == null) cam = Camera.main != null ? Camera.main.transform : null;

        // Optional camera follow (off by default — use PanelPlacer for manual placement).
        if (followCamera && cam != null)
        {
            Vector3 targetPos = cam.position + cam.forward * distanceFromCamera
                                + cam.right * offset.x + cam.up * offset.y;
            Quaternion targetRot = Quaternion.LookRotation(targetPos - cam.position);
            float t = followSpeed * Time.deltaTime;
            transform.position = Vector3.Lerp(transform.position, targetPos, t);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, t);
        }

        SyncButtonVisuals();
        UpdateStatusText();
    }

    void SnapToCamera()
    {
        if (!followCamera) return;
        if (cam == null) cam = Camera.main != null ? Camera.main.transform : null;
        if (cam == null) return;
        Vector3 targetPos = cam.position + cam.forward * distanceFromCamera
                            + cam.right * offset.x + cam.up * offset.y;
        transform.position = targetPos;
        transform.rotation = Quaternion.LookRotation(targetPos - cam.position);
    }

    void SyncButtonVisuals()
    {
        foreach (var btn in buttons)
        {
            var r = btn.bgQuad.GetComponent<Renderer>();
            if (r == null) continue;

            Color baseColor;
            if (btn.isReset) baseColor = buttonResetColor;
            else baseColor = (btn.isActive != null && btn.isActive()) ? buttonOnColor : buttonOffColor;

            r.material.SetColor("_BaseColor", btn.isHovered ? highlightColor : baseColor);
        }
    }

    void UpdateStatusText()
    {
        var t = TaskTracker.Instance;
        if (statusText != null)
        {
            if (t == null) statusText.text = "(no TaskTracker)";
            else statusText.text = $"{t.mode}    ✓{t.TotalCorrect}    ✗{t.TotalWrong}    [{t.State}]";
        }
        if (targetValueText != null)
            targetValueText.text = t != null ? t.targetCount.ToString() : "—";
    }

    public void Toggle() => SetVisible(!gameObject.activeSelf);

    public void SetVisible(bool visible)
    {
        gameObject.SetActive(visible);
        if (visible) SnapToCamera();
        Debug.Log($"[TaskConfigPanel] {(visible ? "shown" : "hidden")}");
    }
}
