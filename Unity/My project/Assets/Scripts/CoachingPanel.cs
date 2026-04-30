using UnityEngine;
using UnityEngine.InputSystem;
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
            "This sim controls a UR3e robot arm.",
            "",
            "Use Quest controllers to move the robot",
            "and interact with objects in the scene.",
            "",
            "Press Y to open the Options Menu.",
            "Navigate pages with A (next) / B (prev)."
        },
        // Page 1: Robot Control - Direct Joint
        new string[]
        {
            "DIRECT JOINT MODE",
            "",
            "Right Stick Y ... Jog selected joint",
            "A Button ....... Next joint",
            "B Button ....... Previous joint",
            "Menu Button .... Switch to Servo mode",
            "",
            "The selected joint is shown on the HUD.",
            "Move the stick gently for slow motion.",
        },
        // Page 2: Robot Control - Servo
        new string[]
        {
            "SERVO MODE (Cartesian)",
            "",
            "Left Stick Y ... Forward / Back",
            "Left Stick X ... Left / Right",
            "Right Stick Y .. Up / Down",
            "Right Stick X .. Yaw rotation",
            "Menu Button .... Switch to Joint mode",
            "",
            "Requires MoveIt Servo running on ROS.",
        },
        // Page 3: Visualization
        new string[]
        {
            "VISUALIZATION",
            "",
            "X Button ....... Toggle TF Axes",
            "Y Button ....... Open Options Menu",
            "",
            "TF axes show coordinate frames on each",
            "robot joint and moving objects (RGB=XYZ).",
            "",
            "The Data Panel shows live joint angles",
            "and end-effector position.",
        },
        // Page 4: Interaction
        new string[]
        {
            "OBJECT INTERACTION",
            "",
            "Right Trigger .. Grab objects",
            "Grip Button .... Release objects",
            "",
            "Grab the cube or other objects and move",
            "them around the scene. Objects placed in",
            "bins are detected automatically.",
            "",
            "The Bin Status panel shows occupancy.",
        },
        // Page 5: Options Menu
        new string[]
        {
            "OPTIONS MENU",
            "",
            "Y Button ....... Toggle menu",
            "Point + Trigger  Select option",
            "",
            "Available toggles:",
            "  - TF Axes (coordinate frames)",
            "  - Data Panel (joint angles)",
            "  - Bin Status (bin detection)",
            "  - Coaching (this panel)",
        },
    };

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

        // Prev button (left)
        prevButton = CreateQuad("PrevBtn",
            new Vector3(-halfW + padding + btnSize / 2f, bottomY, zQuad),
            new Vector2(btnSize, btnSize), accentColor * 0.7f);
        prevButtonText = CreateText("PrevText", "<",
            new Vector3(-halfW + padding + btnSize / 2f, bottomY, zText),
            fontSize, labelColor, TextAlignmentOptions.Center, btnSize);
        prevButtonText.fontStyle = FontStyles.Bold;

        // Next button (right)
        nextButton = CreateQuad("NextBtn",
            new Vector3(halfW - padding - btnSize / 2f, bottomY, zQuad),
            new Vector2(btnSize, btnSize), accentColor * 0.7f);
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
        SetPage(currentPage + 1);
    }

    public void PrevPage()
    {
        SetPage(currentPage - 1);
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
        if (rightController == null) FindController();
        if (rightController == null || prevButton == null || nextButton == null) return;

        // Raycast from right controller forward onto panel plane
        Ray ray = new Ray(rightController.position, rightController.forward);
        Plane panelPlane = new Plane(-transform.forward, transform.position);
        if (!panelPlane.Raycast(ray, out float dist) || dist > 15f) return;

        Vector3 hitWorld = ray.GetPoint(dist);

        // Distance to each button in world space — generous radius
        float distToPrev = Vector3.Distance(hitWorld, prevButton.transform.position);
        float distToNext = Vector3.Distance(hitWorld, nextButton.transform.position);
        float btnRadius = Mathf.Max(prevButton.transform.lossyScale.x, prevButton.transform.lossyScale.y) * 1.0f;

        bool overPrev = distToPrev < btnRadius;
        bool overNext = distToNext < btnRadius;

        // Highlight hovered button
        if (prevButton != null)
        {
            var rend = prevButton.GetComponent<Renderer>();
            if (rend != null)
                rend.material.SetColor("_BaseColor", overPrev ? accentColor : accentColor * 0.7f);
        }
        if (nextButton != null)
        {
            var rend = nextButton.GetComponent<Renderer>();
            if (rend != null)
                rend.material.SetColor("_BaseColor", overNext ? accentColor : accentColor * 0.7f);
        }

        // Click on press
        if (selectAction != null && selectAction.WasPressedThisFrame())
        {
            if (overPrev) PrevPage();
            else if (overNext) NextPage();
        }
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
