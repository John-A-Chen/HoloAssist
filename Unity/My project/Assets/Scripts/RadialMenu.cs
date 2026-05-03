using UnityEngine;
using UnityEngine.InputSystem;
using TMPro;
using System;
using System.Collections.Generic;

/// <summary>
/// Radial menu that hovers above the left XR controller.
/// Toggle with Y button. Styled to match RobotDataPanel.
/// Point with right controller and pull trigger to select.
/// </summary>
public class RadialMenu : MonoBehaviour
{
    [Header("Input")]
    [Tooltip("Button to toggle menu open/close")]
    public InputAction toggleAction;

    [Header("Layout")]
    public float menuRadius = 0.1f;
    public float buttonRadius = 0.032f;
    public float hoverHeight = 0.12f;
    public float bgRadius = 0.17f;

    [Header("Style — matches RobotDataPanel")]
    public Color panelColor = new Color(0.05f, 0.05f, 0.12f, 0.9f);
    public Color headerColor = new Color(0.1f, 0.15f, 0.3f, 0.95f);
    public Color titleColor = new Color(0.6f, 0.8f, 1.0f, 1f);
    public Color accentColor = new Color(0.2f, 0.6f, 1.0f, 1f);
    public Color labelColor = Color.white;
    public Color buttonOffColor = new Color(0.15f, 0.15f, 0.25f, 0.9f);
    public Color buttonOnColor = new Color(0.1f, 0.45f, 0.2f, 0.9f);
    public Color highlightColor = new Color(0.25f, 0.35f, 0.55f, 0.95f);
    public float fontSize = 0.08f;
    public float titleFontSize = 0.12f;

    [Header("References")]
    public JointTFVisualizer tfVisualizer;
    public RobotDataPanel dataPanel;
    public BinStatusPanel binStatusPanel;
    public CoachingPanel coachingPanel;
    public PassthroughToggle passthroughToggle;
    public RobotController robotController;
    public RobotHUD robotHUD;
    public OperatingModeController modeController;

    [Header("Controllers (auto-detected if left blank)")]
    [Tooltip("Drag the left controller transform here for reliable anchoring")]
    public Transform leftControllerOverride;
    [Tooltip("Drag the right controller transform here for reliable anchoring")]
    public Transform rightControllerOverride;

    // Menu state
    private bool isOpen = false;
    private GameObject menuRoot;
    private Transform leftController;
    private Transform rightController;
    private InputAction selectAction;
    private List<RadialButton> buttons = new List<RadialButton>();
    private int currentPage = 0;
    private int totalPages = 2;
    private TextMeshPro pageLabel;
    private GameObject pageButton;

    // Z layers — more negative = closer to camera (menu's Z points away from camera)
    private const float Z_BG = 0f;
    private const float Z_RING = -0.001f;   // behind button
    private const float Z_BUTTON = -0.003f; // in front of ring
    private const float Z_TEXT = -0.005f;   // in front of button
    private const float Z_STATUS = -0.007f; // in front of label

    private class RadialButton
    {
        public string label;
        public int page;
        public GameObject bgQuad;
        public GameObject ringQuad;
        public TextMeshPro labelText;
        public TextMeshPro statusText;
        public bool isOn;
        public Action onToggle;
    }

    void Start()
    {
        // Always create our own action with multiple bindings (Y on quest, K on keyboard for testing)
        toggleAction = new InputAction("ToggleRadialMenu", InputActionType.Button);
        toggleAction.AddBinding("<XRController>{LeftHand}/secondaryButton");
        toggleAction.AddBinding("<OculusTouchController>{LeftHand}/secondaryButton");
        toggleAction.AddBinding("<MetaQuestTouchPlusController>{LeftHand}/secondaryButton");
        toggleAction.AddBinding("<Keyboard>/k"); // Editor fallback
        toggleAction.Enable();
        Debug.Log("[RadialMenu] Toggle action enabled with bindings: Y button + K key");

        selectAction = new InputAction("SelectRadial", InputActionType.Button,
            "<XRController>{RightHand}/triggerPressed");
        selectAction.Enable();

        FindControllers();
        BuildMenu();
        menuRoot.SetActive(false);
    }

    void FindControllers()
    {
        // Manual overrides take priority
        if (leftControllerOverride != null) leftController = leftControllerOverride;
        if (rightControllerOverride != null) rightController = rightControllerOverride;

        if (leftController != null && rightController != null) return;

        // Try by input action reference
        var controllers = FindObjectsByType<UnityEngine.XR.Interaction.Toolkit.XRBaseController>(FindObjectsSortMode.None);
        foreach (var c in controllers)
        {
            string n = c.gameObject.name.ToLower();
            if (leftController == null && n.Contains("left")) leftController = c.transform;
            if (rightController == null && n.Contains("right")) rightController = c.transform;
        }

        // Try common names
        if (leftController == null)
        {
            foreach (var name in new[] { "Left Controller", "LeftHand Controller", "Left Hand",
                                          "XR Left Controller", "LeftHand" })
            {
                var go = GameObject.Find(name);
                if (go != null) { leftController = go.transform; break; }
            }
        }
        if (rightController == null)
        {
            foreach (var name in new[] { "Right Controller", "RightHand Controller", "Right Hand",
                                          "XR Right Controller", "RightHand" })
            {
                var go = GameObject.Find(name);
                if (go != null) { rightController = go.transform; break; }
            }
        }

        if (leftController == null)
        {
            Debug.LogWarning("[RadialMenu] Could not find left controller. Drag it into leftControllerOverride in the Inspector.");
            leftController = Camera.main?.transform;
        }
        else
        {
            Debug.Log($"[RadialMenu] Left controller: {leftController.name}");
        }
    }

    void BuildMenu()
    {
        menuRoot = new GameObject("RadialMenuRoot");
        menuRoot.transform.SetParent(transform, false);

        // Background circle (large quad) — lowest render queue
        CreateQuad(menuRoot.transform, "Background", new Vector3(0, 0, Z_BG),
            bgRadius * 2f, bgRadius * 2f, panelColor, 3000);

        // Header label at top
        var titleObj = new GameObject("Title");
        titleObj.transform.SetParent(menuRoot.transform, false);
        titleObj.transform.localPosition = new Vector3(0f, bgRadius - 0.025f, Z_TEXT);
        var title = titleObj.AddComponent<TextMeshPro>();
        title.text = "OPTIONS";
        title.fontSize = titleFontSize;
        title.color = titleColor;
        title.alignment = TextAlignmentOptions.Center;
        title.enableWordWrapping = false;
        title.fontStyle = FontStyles.Bold;
        title.overflowMode = TextOverflowModes.Overflow;
        title.rectTransform.sizeDelta = new Vector2(bgRadius * 2f, 0.03f);
        if (title.fontSharedMaterial != null)
        {
            var titleMat = new Material(title.fontSharedMaterial);
            titleMat.renderQueue = 3500;
            title.fontMaterial = titleMat;
        }

        // Accent line under header
        CreateQuad(menuRoot.transform, "AccentLine",
            new Vector3(0f, bgRadius - 0.04f, Z_RING),
            bgRadius * 1.4f, 0.002f, accentColor, 3020);

        // Center dot
        CreateQuad(menuRoot.transform, "CenterDot", new Vector3(0, 0, Z_RING),
            0.012f, 0.012f, accentColor, 3020);

        // Add buttons
        AddButton("TF Axes", false, () =>
        {
            if (tfVisualizer != null)
                tfVisualizer.Toggle();
        });

        AddButton("Data\nPanel", true, () =>
        {
            if (dataPanel != null)
                dataPanel.gameObject.SetActive(!dataPanel.gameObject.activeSelf);
        });

        AddButton("Bin\nStatus", true, () =>
        {
            if (binStatusPanel != null)
                binStatusPanel.gameObject.SetActive(!binStatusPanel.gameObject.activeSelf);
        });

        AddButton("Coach", false, () =>
        {
            if (coachingPanel != null)
                coachingPanel.gameObject.SetActive(!coachingPanel.gameObject.activeSelf);
        });

        AddButton("Pass\nthru", true, () =>
        {
            if (passthroughToggle != null)
                passthroughToggle.Toggle();
        });

        AddButton("Robot\nHUD", true, () =>
        {
            if (robotHUD != null)
                robotHUD.gameObject.SetActive(!robotHUD.gameObject.activeSelf);
        });

        // === PAGE 1: Robot Controls (Nic's features) ===
        AddButton("RMRC\nMode", false, () =>
        {
            if (robotController != null)
                robotController.ToggleRMRCSubMode();
        }, 1);

        AddButton("EE\nLock", false, () =>
        {
            if (robotController != null)
                robotController.ToggleEELockDown();
        }, 1);

        AddButton("Joint\nMode", false, () =>
        {
            if (robotController != null)
            {
                robotController.mode = RobotController.ControlMode.DirectJoint;
                Debug.Log("[RadialMenu] Switched to DirectJoint mode");
            }
        }, 1);

        AddButton("RMRC\nCart", false, () =>
        {
            if (robotController != null)
            {
                robotController.mode = RobotController.ControlMode.RMRC;
                Debug.Log("[RadialMenu] Switched to RMRC mode");
            }
        }, 1);

        AddButton("Hand\nGuide", false, () =>
        {
            if (robotController != null)
            {
                robotController.mode = RobotController.ControlMode.HandGuide;
                Debug.Log("[RadialMenu] Switched to HandGuide mode");
            }
        }, 1);

        AddButton("Auto\nMode", false, () =>
        {
            if (modeController != null)
            {
                modeController.ToggleMode();
                Debug.Log($"[RadialMenu] Toggled operating mode");
            }
            else
            {
                Debug.LogWarning("[RadialMenu] OperatingModeController not assigned");
            }
        }, 1);

        // Page nav button — at center, swaps between page 0 and page 1
        CreatePageButton();

        LayoutButtons();
        UpdatePageVisibility();
    }

    void AddButton(string label, bool initialState, Action onToggle, int page = 0)
    {
        var btn = new RadialButton
        {
            label = label,
            page = page,
            isOn = initialState,
            onToggle = onToggle
        };

        // Button background quad — render queue 3030 (on top of rings)
        btn.bgQuad = CreateQuad(menuRoot.transform, $"Btn_{label}",
            Vector3.zero, buttonRadius * 2f, buttonRadius * 2f,
            initialState ? buttonOnColor : buttonOffColor, 3030);

        // Accent ring around button — render queue 3010 (behind button)
        float ringSize = buttonRadius * 2f + 0.006f;
        btn.ringQuad = CreateQuad(menuRoot.transform, $"Ring_{label}",
            Vector3.zero, ringSize, ringSize,
            accentColor * 0.5f, 3010);
        // Ring behind button
        btn.ringQuad.transform.localPosition = new Vector3(0, 0, Z_RING + 0.001f);

        // Label — parented to menuRoot (not button) so scale is correct
        var labelObj = new GameObject($"Label_{label}");
        labelObj.transform.SetParent(menuRoot.transform, false);
        labelObj.transform.localPosition = new Vector3(0f, 0.005f, Z_TEXT);
        btn.labelText = labelObj.AddComponent<TextMeshPro>();
        btn.labelText.text = label;
        btn.labelText.fontSize = fontSize;
        btn.labelText.color = labelColor;
        btn.labelText.alignment = TextAlignmentOptions.Center;
        btn.labelText.enableWordWrapping = true;
        btn.labelText.fontStyle = FontStyles.Bold;
        btn.labelText.overflowMode = TextOverflowModes.Overflow;
        btn.labelText.rectTransform.sizeDelta = new Vector2(buttonRadius * 2.5f, buttonRadius * 1.5f);
        // Force text to render on top of all quads
        if (btn.labelText.fontSharedMaterial != null)
        {
            var labelMat = new Material(btn.labelText.fontSharedMaterial);
            labelMat.renderQueue = 3500;
            btn.labelText.fontMaterial = labelMat;
        }

        // Status text (ON/OFF) — parented to menuRoot
        var statusObj = new GameObject($"Status_{label}");
        statusObj.transform.SetParent(menuRoot.transform, false);
        statusObj.transform.localPosition = new Vector3(0f, -0.015f, Z_STATUS);
        btn.statusText = statusObj.AddComponent<TextMeshPro>();
        btn.statusText.text = initialState ? "ON" : "OFF";
        btn.statusText.fontSize = fontSize * 0.7f;
        btn.statusText.color = initialState ? new Color(0.2f, 1f, 0.4f) : new Color(1f, 0.4f, 0.4f);
        btn.statusText.alignment = TextAlignmentOptions.Center;
        btn.statusText.enableWordWrapping = false;
        btn.statusText.fontStyle = FontStyles.Bold;
        btn.statusText.overflowMode = TextOverflowModes.Overflow;
        btn.statusText.rectTransform.sizeDelta = new Vector2(buttonRadius * 2f, 0.025f);
        // Force text to render on top of all quads
        if (btn.statusText.fontSharedMaterial != null)
        {
            var statusMat = new Material(btn.statusText.fontSharedMaterial);
            statusMat.renderQueue = 3500;
            btn.statusText.fontMaterial = statusMat;
        }

        buttons.Add(btn);
    }

    public void RegisterButton(string label, bool initialState, Action onToggle)
    {
        AddButton(label, initialState, onToggle);
        LayoutButtons();
    }

    void CreatePageButton()
    {
        // Small button at center of menu that cycles pages
        pageButton = CreateQuad(menuRoot.transform, "PageBtn", new Vector3(0, -0.015f, Z_BUTTON),
            0.024f, 0.024f, accentColor * 0.8f, 3030);

        var labelObj = new GameObject("PageLabel");
        labelObj.transform.SetParent(menuRoot.transform, false);
        labelObj.transform.localPosition = new Vector3(0f, -0.015f, Z_TEXT);
        pageLabel = labelObj.AddComponent<TextMeshPro>();
        pageLabel.text = "1/2";
        pageLabel.fontSize = fontSize * 0.7f;
        pageLabel.color = Color.white;
        pageLabel.alignment = TextAlignmentOptions.Center;
        pageLabel.enableWordWrapping = false;
        pageLabel.fontStyle = FontStyles.Bold;
        pageLabel.rectTransform.sizeDelta = new Vector2(0.04f, 0.025f);
        if (pageLabel.fontSharedMaterial != null)
        {
            var mat = new Material(pageLabel.fontSharedMaterial);
            mat.renderQueue = 3500;
            pageLabel.fontMaterial = mat;
        }
    }

    void UpdatePageVisibility()
    {
        // Show/hide buttons based on current page
        foreach (var btn in buttons)
        {
            bool show = btn.page == currentPage;
            if (btn.bgQuad != null) btn.bgQuad.SetActive(show);
            if (btn.ringQuad != null) btn.ringQuad.SetActive(show);
            if (btn.labelText != null) btn.labelText.gameObject.SetActive(show);
            if (btn.statusText != null) btn.statusText.gameObject.SetActive(show);
        }

        if (pageLabel != null)
            pageLabel.text = $"{currentPage + 1}/{totalPages}";
    }

    void NextPage()
    {
        currentPage = (currentPage + 1) % totalPages;
        UpdatePageVisibility();
        Debug.Log($"[RadialMenu] Switched to page {currentPage + 1}");
    }

    void LayoutButtons()
    {
        // Layout each page independently — buttons spread evenly within their page
        for (int p = 0; p < totalPages; p++)
        {
            var pageButtons = new List<RadialButton>();
            foreach (var b in buttons) if (b.page == p) pageButtons.Add(b);

            int count = pageButtons.Count;
            if (count == 0) continue;

            float angleStep = 360f / count;
            float startAngle = 90f;

            for (int i = 0; i < count; i++)
            {
                float angle = (startAngle - i * angleStep) * Mathf.Deg2Rad;
                float x = Mathf.Cos(angle) * menuRadius;
                float y = Mathf.Sin(angle) * menuRadius;

                // Shift down slightly to account for header
                y -= 0.015f;

                Vector3 pos = new Vector3(x, y, Z_BUTTON);
                pageButtons[i].bgQuad.transform.localPosition = pos;

                Vector3 ringPos = new Vector3(x, y, Z_RING);
                pageButtons[i].ringQuad.transform.localPosition = ringPos;

                if (pageButtons[i].labelText != null)
                    pageButtons[i].labelText.transform.localPosition = new Vector3(x, y + 0.005f, Z_TEXT);
                if (pageButtons[i].statusText != null)
                    pageButtons[i].statusText.transform.localPosition = new Vector3(x, y - 0.015f, Z_STATUS);
            }
        }
    }

    GameObject CreateQuad(Transform parent, string name, Vector3 localPos,
        float width, float height, Color color, int renderQueue = 3000)
    {
        var quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quad.name = name;
        quad.transform.SetParent(parent, false);
        quad.transform.localPosition = localPos;
        quad.transform.localScale = new Vector3(width, height, 1f);

        Destroy(quad.GetComponent<Collider>());

        var mat = new Material(Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default"));
        mat.SetFloat("_Surface", 1);
        mat.SetFloat("_Blend", 0);
        mat.SetColor("_BaseColor", color);
        mat.SetFloat("_ZWrite", 0);
        mat.SetOverrideTag("RenderType", "Transparent");
        mat.renderQueue = renderQueue;
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        quad.GetComponent<Renderer>().material = mat;

        return quad;
    }

    void Update()
    {
        if (toggleAction != null && toggleAction.WasPressedThisFrame())
        {
            isOpen = !isOpen;
            menuRoot.SetActive(isOpen);
            Debug.Log($"[RadialMenu] Toggled: {(isOpen ? "OPEN" : "CLOSED")}");
        }

        if (!isOpen) return;

        // Follow left controller
        if (leftController != null)
        {
            Vector3 pos = leftController.position + leftController.up * hoverHeight;
            menuRoot.transform.position = pos;

            if (Camera.main != null)
            {
                Vector3 lookDir = Camera.main.transform.position - menuRoot.transform.position;
                menuRoot.transform.rotation = Quaternion.LookRotation(-lookDir);
            }
        }

        UpdateHighlight();

        if (selectAction != null && selectAction.WasPressedThisFrame())
        {
            // Check page button first (center)
            if (IsHoveringPageButton())
            {
                NextPage();
            }
            else
            {
                int nearest = GetNearestButton();
                if (nearest >= 0)
                    SelectButton(nearest);
            }
        }

        // Sync TF visualizer state (in case toggled via X button)
        SyncButtonStates();
    }

    void SyncButtonStates()
    {
        // Sync TF Axes (button 0)
        if (tfVisualizer != null && buttons.Count > 0)
            SyncButton(0, tfVisualizer.showAxes);

        // Sync Data Panel (button 1)
        if (dataPanel != null && buttons.Count > 1)
            SyncButton(1, dataPanel.gameObject.activeSelf);

        // Sync Bin Status (button 2)
        if (binStatusPanel != null && buttons.Count > 2)
            SyncButton(2, binStatusPanel.gameObject.activeSelf);

        // Sync Coaching (button 3)
        if (coachingPanel != null && buttons.Count > 3)
            SyncButton(3, coachingPanel.gameObject.activeSelf);

        // Sync Passthrough (button 4)
        if (passthroughToggle != null && buttons.Count > 4)
            SyncButton(4, passthroughToggle.PassthroughEnabled);

        // Sync RMRC Mode (button 5) — green when in Rotate mode
        if (robotController != null && buttons.Count > 5)
            SyncButton(5, robotController.CurrentRMRCSubMode == RobotController.RMRCSubMode.Rotate);

        // Sync EE Lock (button 6)
        if (robotController != null && buttons.Count > 6)
            SyncButton(6, robotController.IsEELockedDown);

        // Sync Robot HUD (button 7)
        if (robotHUD != null && buttons.Count > 7)
            SyncButton(7, robotHUD.gameObject.activeSelf);
    }

    void SyncButton(int index, bool state)
    {
        if (index >= buttons.Count) return;
        var btn = buttons[index];
        if (btn.isOn != state)
        {
            btn.isOn = state;
            UpdateButtonVisual(btn);
        }
    }

    void UpdateButtonVisual(RadialButton btn)
    {
        var renderer = btn.bgQuad.GetComponent<Renderer>();
        if (renderer != null)
            renderer.material.SetColor("_BaseColor", btn.isOn ? buttonOnColor : buttonOffColor);

        if (btn.statusText != null)
        {
            btn.statusText.text = btn.isOn ? "ON" : "OFF";
            btn.statusText.color = btn.isOn
                ? new Color(0.2f, 1f, 0.4f)
                : new Color(1f, 0.4f, 0.4f);
        }

        var ringRenderer = btn.ringQuad.GetComponent<Renderer>();
        if (ringRenderer != null)
            ringRenderer.material.SetColor("_BaseColor",
                btn.isOn ? accentColor : accentColor * 0.5f);
    }

    void UpdateHighlight()
    {
        int nearest = GetNearestButton();
        for (int i = 0; i < buttons.Count; i++)
        {
            var btn = buttons[i];
            var renderer = btn.bgQuad.GetComponent<Renderer>();
            if (renderer == null) continue;

            if (i == nearest)
            {
                renderer.material.SetColor("_BaseColor", highlightColor);
            }
            else
            {
                renderer.material.SetColor("_BaseColor",
                    btn.isOn ? buttonOnColor : buttonOffColor);
            }
        }
    }

    int GetNearestButton()
    {
        Transform pointer = rightController != null ? rightController : Camera.main?.transform;
        if (pointer == null || buttons.Count == 0) return -1;

        Ray ray = new Ray(pointer.position, pointer.forward);
        Plane menuPlane = new Plane(menuRoot.transform.forward, menuRoot.transform.position);

        if (!menuPlane.Raycast(ray, out float dist)) return -1;

        Vector3 hitWorld = ray.GetPoint(dist);
        Vector3 hitLocal = menuRoot.transform.InverseTransformPoint(hitWorld);

        int nearest = -1;
        float nearestDist = float.MaxValue;
        for (int i = 0; i < buttons.Count; i++)
        {
            // Only consider buttons on the current page
            if (buttons[i].page != currentPage) continue;

            Vector3 btnLocal = buttons[i].bgQuad.transform.localPosition;
            float d = Vector2.Distance(
                new Vector2(hitLocal.x, hitLocal.y),
                new Vector2(btnLocal.x, btnLocal.y));
            if (d < buttonRadius * 1.2f && d < nearestDist)
            {
                nearest = i;
                nearestDist = d;
            }
        }
        return nearest;
    }

    bool IsHoveringPageButton()
    {
        Transform pointer = rightController != null ? rightController : Camera.main?.transform;
        if (pointer == null || pageButton == null) return false;

        Ray ray = new Ray(pointer.position, pointer.forward);
        Plane menuPlane = new Plane(menuRoot.transform.forward, menuRoot.transform.position);
        if (!menuPlane.Raycast(ray, out float dist)) return false;

        Vector3 hitWorld = ray.GetPoint(dist);
        Vector3 hitLocal = menuRoot.transform.InverseTransformPoint(hitWorld);
        Vector3 pageBtnLocal = pageButton.transform.localPosition;
        float d = Vector2.Distance(
            new Vector2(hitLocal.x, hitLocal.y),
            new Vector2(pageBtnLocal.x, pageBtnLocal.y));
        return d < 0.018f;
    }

    public void SelectButton(int index)
    {
        if (index < 0 || index >= buttons.Count) return;

        var btn = buttons[index];
        btn.isOn = !btn.isOn;
        btn.onToggle?.Invoke();
        UpdateButtonVisual(btn);
    }

    public void ToggleButtonByName(string label)
    {
        for (int i = 0; i < buttons.Count; i++)
        {
            if (buttons[i].label.Contains(label))
            {
                SelectButton(i);
                return;
            }
        }
    }

    void OnDestroy()
    {
        if (toggleAction != null)
        {
            toggleAction.Disable();
            toggleAction.Dispose();
        }
        if (selectAction != null)
        {
            selectAction.Disable();
            selectAction.Dispose();
        }
    }
}
