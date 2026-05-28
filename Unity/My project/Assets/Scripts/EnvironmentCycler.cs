using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// Cycles between visual environment themes (e.g. "Portal Chamber", "Workshop", "Outdoor").
/// Each theme is a root GameObject containing only decorative 3D models for that scene.
/// The robot rig, XR rig, bins, physics colliders, and UI panels should NOT be part of any theme —
/// keep them outside this list so they always stay active.
///
/// Setup:
///   1. Create an empty GameObject in the scene named e.g. "Environments".
///   2. Under it, create one empty child per theme ("Theme_Portal", "Theme_Workshop", ...).
///   3. Put each theme's decorative meshes inside its corresponding child.
///   4. Add this component (to "Environments" or any GameObject).
///   5. Populate the `themes` list in the inspector — assign a display name + root GameObject for each.
///   6. Toggle `Allow None` if you want a "no environment" entry in the cycle.
///
/// Cycle from the radial menu page 3 (Theme button).
/// </summary>
public class EnvironmentCycler : MonoBehaviour
{
    public static EnvironmentCycler Instance { get; private set; }

    [System.Serializable]
    public class EnvironmentTheme
    {
        [Tooltip("Display name shown under the radial Theme button.")]
        public string displayName = "Theme";

        [Tooltip("Root GameObject containing this theme's environment meshes (skybox proxy, ground, props).")]
        public GameObject root;
    }

    [Header("Themes")]
    public List<EnvironmentTheme> themes = new List<EnvironmentTheme>();

    [Tooltip("Index of the theme to show at startup. -1 = none (only valid if Allow None is true).")]
    public int startIndex = 0;

    [Tooltip("Include a 'None' state in the cycle so the user can hide all environments.")]
    public bool allowNone = true;

    [Tooltip("Label shown when no theme is active.")]
    public string noneLabel = "None";

    // -1 means the "none" state; 0..themes.Count-1 means that theme is active.
    private int currentIndex = -1;

    public string CurrentName
    {
        get
        {
            if (currentIndex < 0 || currentIndex >= themes.Count) return noneLabel;
            var t = themes[currentIndex];
            return string.IsNullOrEmpty(t.displayName) ? (t.root != null ? t.root.name : "?") : t.displayName;
        }
    }

    void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Debug.LogWarning($"[EnvironmentCycler] Multiple instances — keeping '{Instance.gameObject.name}', destroying '{gameObject.name}'.");
            Destroy(this);
            return;
        }
        Instance = this;
    }

    void Start()
    {
        // Hide every theme first so we don't end up with two active at once.
        foreach (var t in themes)
            if (t != null && t.root != null) t.root.SetActive(false);

        int initial = Mathf.Clamp(startIndex, allowNone ? -1 : 0, themes.Count - 1);
        Apply(initial);
    }

    void OnDestroy()
    {
        if (Instance == this) Instance = null;
    }

    /// <summary>Advance to the next theme. Wraps around. Includes a None state if allowed.</summary>
    public void CycleNext()
    {
        int max = themes.Count;
        if (max == 0)
        {
            Debug.LogWarning("[EnvironmentCycler] No themes configured.");
            return;
        }

        // Sequence: 0, 1, ..., max-1, [-1 if allowNone], 0, ...
        int next;
        if (allowNone)
            next = currentIndex + 1 > max - 1 ? -1 : currentIndex + 1;
        else
            next = (currentIndex + 1) % max;

        Apply(next);
    }

    void Apply(int newIndex)
    {
        // Disable old
        if (currentIndex >= 0 && currentIndex < themes.Count)
        {
            var prev = themes[currentIndex];
            if (prev != null && prev.root != null) prev.root.SetActive(false);
        }

        currentIndex = newIndex;

        // Themes are VR-only — show the active theme's root only when passthrough is OFF.
        bool showTheme = !IsPassthroughOn();

        if (currentIndex >= 0 && currentIndex < themes.Count)
        {
            var t = themes[currentIndex];
            if (t != null && t.root != null) t.root.SetActive(showTheme);
        }

        Debug.Log($"[EnvironmentCycler] → {CurrentName} (index {currentIndex}, visible={showTheme})");
    }

    static bool IsPassthroughOn()
    {
        return PassthroughToggle.Instance != null && PassthroughToggle.Instance.PassthroughEnabled;
    }

    /// <summary>
    /// Called by PassthroughToggle whenever MR/VR mode flips so the current theme's
    /// visibility matches the new mode. Hides the theme in MR, shows it in VR.
    /// </summary>
    public void OnPassthroughChanged(bool passthroughOn)
    {
        if (currentIndex < 0 || currentIndex >= themes.Count) return;
        var t = themes[currentIndex];
        if (t == null || t.root == null) return;
        t.root.SetActive(!passthroughOn);
        Debug.Log($"[EnvironmentCycler] passthrough {(passthroughOn ? "ON" : "OFF")} → theme '{CurrentName}' {(passthroughOn ? "hidden" : "shown")}.");
    }
}
