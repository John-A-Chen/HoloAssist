using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Toggles between MR passthrough and VR (virtual environment).
///
/// The MR Template's AR Passthrough Feature event handles the skybox-dome
/// fade automatically (via FadeMaterial.FadeSkybox), so this script only
/// needs to flip the camera's clear flags / background and show or hide
/// the virtual environment GameObject.
///
/// Attach to any GameObject. Wire up to RadialMenu via RegisterButton.
/// </summary>
public class PassthroughToggle : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Camera to modify (auto-finds Main Camera if blank)")]
    public Camera xrCamera;

    [Tooltip("Virtual environment GameObject — shown when passthrough is OFF (e.g. skybox, room mesh)")]
    public GameObject virtualEnvironment;

    [Tooltip("If true, only hides renderers in MR mode but keeps colliders active (so gravity works)")]
    public bool keepCollidersInPassthrough = true;

    [Header("Settings")]
    [Tooltip("Background color to use when in VR mode (no passthrough)")]
    public Color vrBackgroundColor = new Color(0.05f, 0.07f, 0.1f, 1f);

    [Tooltip("Skybox material to use in VR mode (optional)")]
    public Material vrSkyboxMaterial;

    [Tooltip("Start with passthrough enabled")]
    public bool startWithPassthrough = true;

    private bool passthroughEnabled = true;
    private CameraClearFlags origClearFlags;
    private Color origBackgroundColor;
    private Material origSkybox;

    public bool PassthroughEnabled => passthroughEnabled;

    void Start()
    {
        if (xrCamera == null)
            xrCamera = Camera.main;

        // Fallback — pick the first enabled camera in the scene.
        if (xrCamera == null)
        {
            foreach (var c in FindObjectsOfType<Camera>())
            {
                if (c.enabled && c.gameObject.activeInHierarchy) { xrCamera = c; break; }
            }
        }

        if (xrCamera == null)
        {
            Debug.LogError("[PassthroughToggle] No camera found");
            enabled = false;
            return;
        }

        Debug.Log($"[PassthroughToggle] Bound to camera '{xrCamera.name}' (instance {xrCamera.GetInstanceID()}). Initial clearFlags={xrCamera.clearFlags}, bg={xrCamera.backgroundColor}");

        // Save original camera settings
        origClearFlags = xrCamera.clearFlags;
        origBackgroundColor = xrCamera.backgroundColor;
        origSkybox = RenderSettings.skybox;

        SetPassthrough(startWithPassthrough);
    }

    public void Toggle()
    {
        Debug.Log($"[PassthroughToggle] Toggle() called. passthroughEnabled was {passthroughEnabled}.");
        SetPassthrough(!passthroughEnabled);
    }

    public void SetPassthrough(bool enabled)
    {
        passthroughEnabled = enabled;

        if (enabled)
        {
            // MR mode — passthrough on, transparent background.
            xrCamera.clearFlags = CameraClearFlags.SolidColor;
            xrCamera.backgroundColor = new Color(0, 0, 0, 0); // alpha 0 reveals passthrough
            if (vrSkyboxMaterial != null && origSkybox != vrSkyboxMaterial)
                RenderSettings.skybox = origSkybox;

            SetEnvironmentVisible(false);

            Debug.Log("[PassthroughToggle] Passthrough ON (MR mode)");
        }
        else
        {
            if (vrSkyboxMaterial != null)
            {
                xrCamera.clearFlags = CameraClearFlags.Skybox;
                RenderSettings.skybox = vrSkyboxMaterial;
            }
            else
            {
                xrCamera.clearFlags = CameraClearFlags.SolidColor;
                xrCamera.backgroundColor = vrBackgroundColor;
            }

            SetEnvironmentVisible(true);

            Debug.Log("[PassthroughToggle] Passthrough OFF (VR mode)");
        }
    }

    void SetEnvironmentVisible(bool visible)
    {
        if (virtualEnvironment == null) return;

        if (keepCollidersInPassthrough)
        {
            // Keep GameObject active so colliders work, but toggle renderers only
            virtualEnvironment.SetActive(true);
            foreach (var rend in virtualEnvironment.GetComponentsInChildren<Renderer>(true))
                rend.enabled = visible;
        }
        else
        {
            virtualEnvironment.SetActive(visible);
        }
    }
}
