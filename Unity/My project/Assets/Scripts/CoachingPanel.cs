using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;
using TMPro;
using System.Collections.Generic;

/// <summary>
/// Coaching/tutorial panel that teaches the user how to use the XR sim and controls.
/// Styled to match RobotDataPanel. Shows step-by-step instructions with pages.
/// Can be toggled via radial menu.
/// </summary>
[ExecuteInEditMode]
public class CoachingPanel : MonoBehaviour
{
    [Header("Layout")]
    public float distanceFromCamera = 1.6f;
    public Vector3 offset = new Vector3(0f, 0.2f, 0f); // centered, slightly above
    public float followSpeed = 2.5f;

    [Header("Panel Dimensions")]
    public float panelWidth = 0.7f;
    public float headerHeight = 0.06f;
    public float rowHeight = 0.04f;
    public float fontSize = 0.2f;
    public float headerFontSize = 0.28f;
    public float padding = 0.015f;

    [Header("Colors — matches RobotDataPanel")]
    public Color panelColor = new Color(0.05f, 0.05f, 0.12f, 0.92f);
    public Color headerColor = new Color(0.1f, 0.15f, 0.3f, 0.95f);
    public Color titleColor = new Color(0.6f, 0.8f, 1.0f, 1f);
    public Color labelColor = new Color(0.8f, 0.82f, 0.85f, 1f);
    public Color accentColor = new Color(0.2f, 0.6f, 1.0f, 1f);
    public Color highlightColor = new Color(0.9f, 0.7f, 0.2f, 1f);
    public Color dimColor = new Color(0.5f, 0.5f, 0.55f, 1f);

    [Header("State")]
    public int currentPage = 0;

    [Header("Custom UI Reveal")]
    [Tooltip("Custom UI panels (RobotDataPanel, RadialMenu, BinStatusPanel, RobotHUD, etc.) " +
             "hidden at runtime until the user finishes coaching. Stay active in the Editor " +
             "so you can still see + edit them in the Scene view.")]
    public List<GameObject> hiddenUntilCoachingDone = new List<GameObject>();

    [Tooltip("If true, calling NextPage past the last coaching page automatically reveals " +
             "the UI panels above and hides this coaching panel.")]
    public bool revealOnLastPageNext = true;

    private Transform cam;
    private bool built;
    private float panelHeight;
    private List<GameObject> pageObjects = new List<GameObject>();
    private TextMeshPro pageIndicatorText;
    private GameObject prevButton;
    private GameObject nextButton;
    private TextMeshPro prevButtonText;
    private TextMeshPro nextButtonText;
    private InputAction selectAction;
    private Transform rightController;

    [Header("Page Nav")]
    [Tooltip("Time (seconds) to hover on a button to activate it")]
    public float hoverDwellTime = 0.6f;
    private float prevHoverTime = 0f;
    private float nextHoverTime = 0f;

    [Header("Controllers")]
    public Transform rightControllerOverride;

    // Page content — each page is a string array of lines
    private static readonly string[][] pages = new string[][]
    {
        // Page 0: Welcome
        new string[]
        {
            "GETTING STARTED",
            "",
            "Welcome to HoloAssist!",
            "This sim teleoperates a UR3e robot arm",
            "with an OnRobot RG2 gripper.",
            "",
            "Three control modes: RMRC (Cartesian),",
            "Direct Joint, and Hand Guide.",
            "",
            "Press Next on the right to flip pages.",
            "Press Y at any time for the radial menu.",
        },
        // Page 1: Mode switching
        new string[]
        {
            "MODE SWITCHING",
            "",
            "Menu Button .... Cycle modes",
            "  RMRC -> Direct Joint -> Hand Guide",
            "",
            "Or open the radial menu (Y), flip to",
            "Page 2 of 2, and pick:",
            "  RMRC Cart  /  Joint Mode  /  Hand Guide",
            "",
            "Current mode is shown on the Robot HUD.",
        },
        // Page 2: RMRC (Cartesian)
        new string[]
        {
            "RMRC - CARTESIAN MODE",
            "",
            "Default sub-mode is TRANSLATE:",
            "  Right Stick   .. EE forward/back + L/R",
            "  Left Stick Y  .. EE up/down (Z)",
            "  Left Stick X  .. Yaw",
            "",
            "Switch to ROTATE via radial -> RMRC Mode:",
            "  Right Stick   .. Wrist 1 + Wrist 2",
            "  Left Stick X  .. Wrist 3",
        },
        // Page 3: Direct Joint
        new string[]
        {
            "DIRECT JOINT MODE",
            "",
            "Right Stick Y .. Jog selected joint",
            "A Button ...... Next joint",
            "B Button ...... Previous joint",
            "",
            "Or use radial Page 2: Prev/Next Joint",
            "buttons (status shows the joint name).",
            "",
            "Selected joint is shown on the Robot HUD.",
        },
        // Page 4: Hand Guide
        new string[]
        {
            "HAND GUIDE MODE",
            "",
            "Hold the Right Grip to track your hand.",
            "The end-effector follows your controller",
            "while held; release grip to stop.",
            "",
            "Best for fine positioning. Movement is",
            "speed-limited for safety.",
        },
        // Page 5: Gripper + EE Lock
        new string[]
        {
            "GRIPPER + EE LOCK",
            "",
            "Right Trigger .. Gripper width 0-100%",
            "  Squeeze partially for analog control.",
            "  Works in all modes simultaneously.",
            "",
            "Radial Page 2 -> EE Lock toggles tool",
            "pointing straight down (locks orientation",
            "while you translate freely).",
        },
        // Page 6: Radial Menu (Options)
        new string[]
        {
            "RADIAL MENU (Y BUTTON)",
            "",
            "Hold Y to open. Point with right",
            "controller; pull Right Trigger to select.",
            "",
            "Page 1 - UI panels:",
            "  TF Axes / Data Panel / Bin Status",
            "  Coach / Passthru / Robot HUD",
            "",
            "Page 2 - Robot controls (modes, EE Lock,",
            "RMRC Mode, Prev/Next Joint).",
        },
        // Page 7: Bins + finishing
        new string[]
        {
            "BINS + GETTING STARTED",
            "",
            "Drop objects into the bins around the",
            "workspace - the Bin Status panel updates",
            "occupancy automatically.",
            "",
            "Press Passthru on the radial menu to",
            "switch between MR (real world) and VR",
            "(virtual environment).",
            "",
            "Press NEXT now to start.",
        },
    };

    void Awake()
    {
        // Runtime-only: hide the user-listed UI panels until coaching finishes.
        // In the Editor (edit mode) we leave them active so the Scene view shows them.
        if (Application.isPlaying)
        {
            foreach (var go in hiddenUntilCoachingDone)
            {
                if (go != null) go.SetActive(false);
            }
        }
    }

    void OnEnable()
    {
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            if (Application.isPlaying)
                Destroy(transform.GetChild(i).gameObject);
            else
                DestroyImmediate(transform.GetChild(i).gameObject);
        }
        pageObjects.Clear();
        built = false;
    }

    void Start()
    {
        Rebuild();

        // Right trigger for page button selection
        selectAction = new InputAction("CoachingSelect", InputActionType.Button,
            "<XRController>{RightHand}/triggerPressed");
        selectAction.Enable();

        FindController();
    }

    void FindController()
    {
        if (rightControllerOverride != null) { rightController = rightControllerOverride; return; }

        var controllers = FindObjectsByType<UnityEngine.XR.Interaction.Toolkit.XRBaseController>(FindObjectsSortMode.None);
        foreach (var c in controllers)
        {
            if (c.gameObject.name.ToLower().Contains("right"))
            {
                rightController = c.transform;
                return;
            }
        }
    }

    void Rebuild()
    {
        if (built) return;
        cam = Camera.main != null ? Camera.main.transform : null;
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

        // Calculate height based on max page content
        int maxRows = 0;
        foreach (var page in pages)
            if (page.Length > maxRows) maxRows = page.Length;

        panelHeight = headerHeight + padding * 2
            + maxRows * rowHeight
            + rowHeight // page indicator
            + padding * 2;

        float yPos = panelHeight / 2f;

        // Header
        yPos -= headerHeight / 2f;
        CreateQuad("Header", new Vector3(0f, yPos, zQuad),
            new Vector2(panelWidth, headerHeight), headerColor);
        CreateText("title", "COACHING", new Vector3(0f, yPos, zText),
            headerFontSize, titleColor, TextAlignmentOptions.Center, panelWidth - padding * 2);
        yPos -= headerHeight / 2f + padding;

        // Accent line
        CreateQuad("AccentLine", new Vector3(0f, yPos, zQuad),
            new Vector2(panelWidth - padding * 2, 0.003f), accentColor);
        yPos -= padding;

        // Content area — build all pages, show only current
        float contentStartY = yPos;
        for (int p = 0; p < pages.Length; p++)
        {
            var pageRoot = new GameObject($"Page_{p}");
            pageRoot.transform.SetParent(transform, false);

            float cy = contentStartY;
            for (int r = 0; r < pages[p].Length; r++)
            {
                cy -= rowHeight / 2f;
                string line = pages[p][r];

                if (line == "") { cy -= rowHeight / 2f; continue; }

                Color lineColor = labelColor;
                if (r == 0) lineColor = highlightColor; // section title
                if (line.Contains("...")) lineColor = accentColor; // control hints

                var textObj = new GameObject($"Line_{r}");
                textObj.transform.SetParent(pageRoot.transform, false);
                textObj.transform.localPosition = new Vector3(0f, cy, zText);
                var tmp = textObj.AddComponent<TextMeshPro>();
                tmp.text = line;
                tmp.fontSize = fontSize;
                tmp.color = lineColor;
                tmp.alignment = TextAlignmentOptions.Center;
                tmp.enableWordWrapping = false;
                tmp.overflowMode = TextOverflowModes.Overflow;
                tmp.rectTransform.sizeDelta = new Vector2(panelWidth - padding * 4, rowHeight);

                cy -= rowHeight / 2f;
            }

            pageRoot.SetActive(p == currentPage);
            pageObjects.Add(pageRoot);
        }

        // Page nav row at bottom: [PREV]  o . . . . .  [NEXT]
        float bottomY = -panelHeight / 2f + padding + rowHeight / 2f;
        float btnSize = rowHeight * 1.4f;
        float halfW = panelWidth / 2f;

        // Prev button (left) — push slightly forward of panel for ray priority
        float btnZ = zQuad - 0.05f;
        prevButton = CreateQuad("PrevBtn",
            new Vector3(-halfW + padding + btnSize / 2f, bottomY, btnZ),
            new Vector2(btnSize, btnSize), accentColor * 0.7f);
        AddPageNavInteractable(prevButton, btnSize, () => PrevPage());
        prevButtonText = CreateText("PrevText", "<",
            new Vector3(-halfW + padding + btnSize / 2f, bottomY, zText),
            fontSize, labelColor, TextAlignmentOptions.Center, btnSize);
        prevButtonText.fontStyle = FontStyles.Bold;

        // Next button (right)
        nextButton = CreateQuad("NextBtn",
            new Vector3(halfW - padding - btnSize / 2f, bottomY, btnZ),
            new Vector2(btnSize, btnSize), accentColor * 0.7f);
        AddPageNavInteractable(nextButton, btnSize, () => NextPage());
        nextButtonText = CreateText("NextText", ">",
            new Vector3(halfW - padding - btnSize / 2f, bottomY, zText),
            fontSize, labelColor, TextAlignmentOptions.Center, btnSize);
        nextButtonText.fontStyle = FontStyles.Bold;

        // Page indicator dots (between buttons)
        pageIndicatorText = CreateText("pageIndicator", GetPageIndicator(),
            new Vector3(0f, bottomY, zText),
            fontSize * 0.8f, dimColor, TextAlignmentOptions.Center, panelWidth - btnSize * 2 - padding * 4);

        // Background
        CreateQuad("Background", Vector3.zero,
            new Vector2(panelWidth, panelHeight), panelColor);
    }

    string GetPageIndicator()
    {
        string dots = "";
        for (int i = 0; i < pages.Length; i++)
            dots += (i == currentPage) ? " O " : " . ";
        return dots.Trim();
    }

    public void NextPage()
    {
        if (revealOnLastPageNext && currentPage >= pages.Length - 1)
        {
            FinishCoaching();
            return;
        }
        SetPage(currentPage + 1);
    }

    public void PrevPage()
    {
        SetPage(currentPage - 1);
    }

    /// <summary>
    /// Reveal all panels listed in <see cref="hiddenUntilCoachingDone"/> and hide
    /// this coaching panel. Safe to call from anywhere — e.g. a "Start" button on
    /// the last page, or from the radial menu.
    /// </summary>
    public void FinishCoaching()
    {
        foreach (var go in hiddenUntilCoachingDone)
        {
            if (go != null) go.SetActive(true);
        }
        gameObject.SetActive(false);
    }

    public void SetPage(int page)
    {
        currentPage = Mathf.Clamp(page, 0, pages.Length - 1);
        for (int i = 0; i < pageObjects.Count; i++)
        {
            if (pageObjects[i] != null)
                pageObjects[i].SetActive(i == currentPage);
        }

        if (pageIndicatorText != null)
            pageIndicatorText.text = GetPageIndicator();
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

    void AddPageNavInteractable(GameObject btn, float btnSize, System.Action onClick)
    {
        // Add a small box collider so XR ray can hit the button
        var col = btn.AddComponent<BoxCollider>();
        col.size = new Vector3(1f, 1f, 0.1f); // local space (quad scale already btnSize)
        col.isTrigger = false;

        // Add XRSimpleInteractable so XR ray + trigger selects it
        var interactable = btn.AddComponent<XRSimpleInteractable>();
        interactable.colliders.Add(col);
        interactable.selectMode = InteractableSelectMode.Single;

        // Hover highlight (gold while hovered)
        interactable.hoverEntered.AddListener(args =>
        {
            var rend = btn.GetComponent<Renderer>();
            if (rend != null)
                rend.material.SetColor("_BaseColor", new Color(1f, 0.85f, 0.2f, 1f));
        });
        interactable.hoverExited.AddListener(args =>
        {
            var rend = btn.GetComponent<Renderer>();
            if (rend != null)
                rend.material.SetColor("_BaseColor", accentColor * 0.7f);
        });

        // Trigger select to fire the click
        interactable.selectEntered.AddListener(args => onClick?.Invoke());
    }

    GameObject CreateQuad(string name, Vector3 localPos, Vector2 size, Color color)
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

        var mat = new Material(Shader.Find("Universal Render Pipeline/Unlit"));
        mat.SetFloat("_Surface", 1);
        mat.SetFloat("_Blend", 0);
        mat.SetColor("_BaseColor", color);
        mat.SetFloat("_ZWrite", 0);
        mat.SetOverrideTag("RenderType", "Transparent");
        mat.renderQueue = name == "Background" ? 2950 : 3000;
        mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        quad.GetComponent<Renderer>().material = mat;
        return quad;
    }

    void LateUpdate()
    {
        if (!built) Rebuild();
        if (!Application.isPlaying) return;
        if (cam == null) return;
        UpdatePosition(false);
        HandlePageNav();
    }

    void HandlePageNav()
    {
        // Page navigation is now handled by XRSimpleInteractable hover/select on the buttons.
        // Hovering shows gold highlight, pulling trigger advances/retreats pages.
    }

    void OnDestroy()
    {
        if (selectAction != null)
        {
            selectAction.Disable();
            selectAction.Dispose();
        }
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
}
