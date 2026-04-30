using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Toggles between MR passthrough and VR (virtual environment).
/// Switches the camera's clear flags + background, and optionally shows/hides
/// a virtual environment GameObject (e.g. skybox or room).
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

        if (xrCamera == null)
        {
            Debug.LogError("[PassthroughToggle] No camera found");
            enabled = false;
            return;
        }

        // Save original camera settings
        origClearFlags = xrCamera.clearFlags;
        origBackgroundColor = xrCamera.backgroundColor;
        origSkybox = RenderSettings.skybox;

        SetPassthrough(startWithPassthrough);
    }

    public void Toggle()
    {
        SetPassthrough(!passthroughEnabled);
    }

    public void SetPassthrough(bool enabled)
    {
        passthroughEnabled = enabled;

        if (enabled)
        {
            // MR mode — passthrough on, transparent background
            xrCamera.clearFlags = CameraClearFlags.SolidColor;
            xrCamera.backgroundColor = new Color(0, 0, 0, 0); // alpha 0 reveals passthrough
            if (vrSkyboxMaterial != null && origSkybox != vrSkyboxMaterial)
                RenderSettings.skybox = origSkybox;

            if (virtualEnvironment != null)
                virtualEnvironment.SetActive(false);

            Debug.Log("[PassthroughToggle] Passthrough ON (MR mode)");
        }
        else
        {
            // VR mode — solid color or skybox, passthrough off
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

            if (virtualEnvironment != null)
                virtualEnvironment.SetActive(true);

            Debug.Log("[PassthroughToggle] Passthrough OFF (VR mode)");
        }
    }
}
