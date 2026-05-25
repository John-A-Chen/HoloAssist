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
    public Color correctFlashColor = new Color(0.2f, 1f, 0.35f, 0.75f);
    public Color wrongFlashColor = new Color(1f, 0.25f, 0.25f, 0.75f);
    [Tooltip("Persistent cylinder colour while task state is Failed.")]
    public Color failedBaseColor = new Color(0.6f, 0.1f, 0.1f, 0.75f);
    [Tooltip("Persistent text tint while task state is Failed.")]
    public Color failedTextColor = new Color(1f, 0.7f, 0.7f, 1f);

    [Header("Flash")]
    [Tooltip("How long a flash takes to fade back to base colour.")]
    public float flashDuration = 1.2f;

    [Header("Progress Ring")]
    [Tooltip("Number of segments around the cylinder top. More = smoother ring.")]
    public int ringSegmentCount = 32;
    [Tooltip("Vertical offset of the ring above the cylinder top.")]
    public float ringYOffset = 0.04f;
    [Tooltip("Ring band height.")]
    public float ringHeight = 0.05f;
    [Tooltip("Outward offset of the ring beyond the cylinder radius.")]
    public float ringRadiusOffset = 0.015f;
    public Color ringEmptyColor = new Color(0.15f, 0.2f, 0.3f, 0.7f);
    public Color ringFilledColor = new Color(0.3f, 0.8f, 1f, 0.95f);
    public Color ringCompleteColor = new Color(0.2f, 1f, 0.4f, 0.95f);

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

    [Header("Timer")]
    [Tooltip("Whether the floating timer display is visible at start.")]
    public bool timerVisibleOnStart = false;
    [Tooltip("Number of timer text instances around the cylinder (mirrors textInstances).")]
    public int timerInstances = 4;
    [Tooltip("Vertical offset of the timer below the cylinder.")]
    public float timerYOffset = -0.18f;
    public float timerFontSize = 0.5f;
    public Color timerColor = new Color(1f, 0.95f, 0.6f);

    public bool IsVisible => gameObject.activeSelf;

    private Renderer cylRenderer;
    private List<TextMeshPro> labels = new List<TextMeshPro>();
    private List<TextMeshPro> timerLabels = new List<TextMeshPro>();
    private GameObject timerRoot;
    private List<Renderer> ringSegments = new List<Renderer>();
    private float flashEndTime = -1f;
    private Color activeFlashColor;
    private float progress01 = 0f;
    private bool taskComplete = false;
    private bool taskFailed = false;
    private float timerElapsed = 0f;
    private bool timerRunning = false;

    public bool IsTimerVisible => timerRoot != null && timerRoot.activeSelf;
    public bool IsTaskFailed => taskFailed;

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

        BuildProgressRing();
        BuildTimer();
    }

    void BuildTimer()
    {
        timerRoot = new GameObject("BannerTimer");
        timerRoot.transform.SetParent(transform, false);
        timerRoot.transform.localPosition = Vector3.zero;
        timerRoot.transform.localRotation = Quaternion.identity;

        int n = Mathf.Max(1, timerInstances);
        for (int i = 0; i < n; i++)
        {
            float angleDeg = (360f / n) * i;
            float angleRad = angleDeg * Mathf.Deg2Rad;

            var tObj = new GameObject($"TimerLbl_{i}");
            tObj.transform.SetParent(timerRoot.transform, false);
            tObj.transform.localPosition = new Vector3(
                Mathf.Sin(angleRad) * (radius + 0.01f),
                timerYOffset,
                Mathf.Cos(angleRad) * (radius + 0.01f));
            tObj.transform.localRotation = Quaternion.Euler(0f, angleDeg, 0f);

            var tmp = tObj.AddComponent<TextMeshPro>();
            tmp.text = "0:00";
            tmp.fontSize = timerFontSize;
            tmp.color = timerColor;
            tmp.alignment = TextAlignmentOptions.Center;
            tmp.fontStyle = FontStyles.Bold;
            tmp.enableWordWrapping = false;
            tmp.rectTransform.sizeDelta = new Vector2(radius * 1.6f, 0.08f);

            if (tmp.fontSharedMaterial != null)
            {
                var mat = new Material(tmp.fontSharedMaterial);
                mat.renderQueue = 3500;
                tmp.fontMaterial = mat;
            }

            timerLabels.Add(tmp);
        }

        timerRoot.SetActive(timerVisibleOnStart);
        timerRunning = timerVisibleOnStart;
    }

    void BuildProgressRing()
    {
        int n = Mathf.Max(8, ringSegmentCount);
        float ringRadius = radius + ringRadiusOffset;
        float ringY = height * 0.5f + ringYOffset;
        // Width of each segment along the ring's tangent — slightly overlap to hide seams
        float segWidth = (2f * Mathf.PI * ringRadius) / n * 1.05f;

        var shader = Shader.Find("Universal Render Pipeline/Unlit")
                     ?? Shader.Find("Unlit/Color")
                     ?? Shader.Find("Sprites/Default");

        for (int i = 0; i < n; i++)
        {
            float angleDeg = (360f / n) * i;
            float angleRad = angleDeg * Mathf.Deg2Rad;

            var seg = GameObject.CreatePrimitive(PrimitiveType.Cube);
            seg.name = $"RingSeg_{i}";
            Destroy(seg.GetComponent<Collider>());
            seg.transform.SetParent(transform, false);
            seg.transform.localPosition = new Vector3(
                Mathf.Sin(angleRad) * ringRadius,
                ringY,
                Mathf.Cos(angleRad) * ringRadius);
            // Face outward (segment's local Z aligned with radial direction)
            seg.transform.localRotation = Quaternion.Euler(0f, angleDeg, 0f);
            seg.transform.localScale = new Vector3(segWidth, ringHeight, 0.005f);

            var mat = new Material(shader);
            mat.SetFloat("_Surface", 1);
            mat.SetFloat("_Blend", 0);
            mat.SetColor("_BaseColor", ringEmptyColor);
            mat.color = ringEmptyColor;
            mat.SetFloat("_ZWrite", 0);
            mat.SetOverrideTag("RenderType", "Transparent");
            mat.renderQueue = 3000;
            mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");

            var rend = seg.GetComponent<Renderer>();
            rend.material = mat;
            rend.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            rend.receiveShadows = false;

            ringSegments.Add(rend);
        }

        RefreshRing();
    }

    void RefreshRing()
    {
        if (ringSegments.Count == 0) return;
        int total = ringSegments.Count;
        int filled = Mathf.Clamp(Mathf.RoundToInt(progress01 * total), 0, total);
        Color fillCol = taskComplete ? ringCompleteColor : ringFilledColor;
        for (int i = 0; i < total; i++)
        {
            var r = ringSegments[i];
            if (r == null) continue;
            r.material.SetColor("_BaseColor", i < filled ? fillCol : ringEmptyColor);
        }
    }

    void LateUpdate()
    {
        // Resting colour depends on task state.
        Color restCylinder = taskFailed ? failedBaseColor : baseColor;
        Color restText = taskFailed ? failedTextColor : textColor;

        if (flashEndTime > 0f)
        {
            float remaining = flashEndTime - Time.time;
            if (remaining <= 0f)
            {
                flashEndTime = -1f;
                ApplyColor(restCylinder, restText);
            }
            else
            {
                float t = Mathf.Clamp01(remaining / flashDuration);
                ApplyColor(Color.Lerp(restCylinder, activeFlashColor, t),
                           Color.Lerp(restText, flashTextColor, t));
            }
        }

        // Timer tick.
        if (timerRunning && timerRoot != null && timerRoot.activeSelf)
        {
            timerElapsed += Time.deltaTime;
            UpdateTimerText();
        }
    }

    void UpdateTimerText()
    {
        if (timerLabels.Count == 0) return;
        int totalSec = Mathf.FloorToInt(timerElapsed);
        int m = totalSec / 60;
        int s = totalSec % 60;
        string text = $"{m}:{s:D2}";
        for (int i = 0; i < timerLabels.Count; i++)
            if (timerLabels[i] != null) timerLabels[i].text = text;
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

    /// <summary>Trigger a pulse in the default flashColor (legacy / generic).</summary>
    public void Flash()
    {
        Flash(flashColor);
    }

    /// <summary>Trigger a pulse in a specific colour.</summary>
    public void Flash(Color color)
    {
        activeFlashColor = color;
        flashEndTime = Time.time + flashDuration;
    }

    public void FlashCorrect() => Flash(correctFlashColor);
    public void FlashWrong() => Flash(wrongFlashColor);

    /// <summary>Update the progress ring (0..1). Called by TaskTracker.</summary>
    public void SetProgress(float fraction)
    {
        progress01 = Mathf.Clamp01(fraction);
        RefreshRing();
    }

    /// <summary>Mark the task as complete — ring switches to the complete colour. Freezes the timer at its current value so the displayed time becomes the final completion time.</summary>
    public void SetTaskComplete(bool complete)
    {
        bool wasComplete = taskComplete;
        taskComplete = complete;
        if (complete) taskFailed = false;
        RefreshRing();

        // Pause the timer on the transition to Complete. Reset Timer / Toggle
        // Timer can restart it manually; we don't auto-resume on un-complete
        // because the displayed time would jump unexpectedly.
        if (complete && !wasComplete && timerRunning)
        {
            timerRunning = false;
            Debug.Log($"[EventBanner] Task complete — timer frozen at {timerLabels[0]?.text ?? "0:00"}");
        }
    }

    /// <summary>Mark the task as failed — cylinder stays red, text uses failed tint.</summary>
    public void SetTaskFailed(bool failed)
    {
        if (taskFailed == failed) return;
        taskFailed = failed;
        if (failed) taskComplete = false;
        // Immediately repaint the resting colour (no flash needed if we're not mid-flash).
        if (flashEndTime <= 0f)
            ApplyColor(taskFailed ? failedBaseColor : baseColor,
                       taskFailed ? failedTextColor : textColor);
        Debug.Log($"[EventBanner] Task failed = {failed}");
    }

    public void SetTimerVisible(bool visible)
    {
        if (timerRoot == null) return;
        timerRoot.SetActive(visible);
        timerRunning = visible;
        Debug.Log($"[EventBanner] Timer {(visible ? "shown (running)" : "hidden (paused)")}");
    }

    public void ToggleTimer() => SetTimerVisible(!IsTimerVisible);

    public void ResetTimer()
    {
        timerElapsed = 0f;
        UpdateTimerText();
        // If the timer is visible, resume counting after reset — most natural after a
        // complete-pause + Reset Timer click. If hidden, leave paused (it'll resume on Show).
        if (timerRoot != null && timerRoot.activeSelf) timerRunning = true;
        Debug.Log("[EventBanner] Timer reset to 0:00");
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
