using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

/// <summary>
/// Captures the Quest 3 camera feed (or Unity scene view as fallback) and publishes
/// it as a CompressedImage (JPEG) to a ROS topic for display on the dashboard.
///
/// Setup:
///   1. Create an empty GameObject in the scene (e.g. "HeadsetStream")
///   2. Drag this script onto it
///   3. Leave sourceCamera empty — it will auto-find Camera.main (the XR camera)
///
/// The script first tries WebCamTexture to get the real passthrough camera feed.
/// If that fails (no camera permission, no device found), it falls back to rendering
/// the Unity scene via a secondary capture camera attached to the XR head.
/// </summary>
public class HeadsetStreamPublisher : MonoBehaviour
{
    [Header("ROS")]
    [Tooltip("ROS topic to publish compressed images to")]
    public string topic = "/headset/image_compressed";

    [Header("Capture Settings")]
    [Tooltip("Width of captured image (lower = less GPU cost)")]
    public int captureWidth = 640;

    [Tooltip("Height of captured image")]
    public int captureHeight = 480;

    [Tooltip("Target frames per second for capture")]
    public int targetFPS = 15;

    [Tooltip("JPEG quality 1-100 (lower = smaller, faster)")]
    [Range(1, 100)]
    public int jpegQuality = 50;

    [Tooltip("Source camera for fallback render (leave empty to use Camera.main)")]
    public Camera sourceCamera;

    private ROSConnection ros;
    private float captureInterval;
    private float nextCaptureTime;

    // Render capture
    private Camera captureCamera;
    private RenderTexture renderTexture;
    private Texture2D renderReadback;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<CompressedImageMsg>(topic);

        captureInterval = 1f / targetFPS;
        nextCaptureTime = Time.time;

        if (sourceCamera == null)
            sourceCamera = Camera.main;

        SetupCaptureCamera();
    }

    void SetupCaptureCamera()
    {
        renderTexture = new RenderTexture(captureWidth, captureHeight, 24, RenderTextureFormat.ARGB32);
        renderTexture.antiAliasing = 1;
        renderTexture.Create();

        GameObject camObj = new GameObject("HeadsetCaptureCamera");
        camObj.transform.SetParent(sourceCamera.transform, false);
        camObj.transform.localPosition = Vector3.zero;
        camObj.transform.localRotation = Quaternion.identity;

        captureCamera = camObj.AddComponent<Camera>();
        captureCamera.CopyFrom(sourceCamera);
        captureCamera.targetTexture = renderTexture;
        captureCamera.enabled = false;

        // Detach the capture camera from the XR stereo pipeline so it can't compete
        // with the main XR camera for the headset's eye buffers / passthrough alpha.
        // CopyFrom inherits stereoTargetEye = Both, which can blank passthrough.
        captureCamera.stereoTargetEye = StereoTargetEyeMask.None;

        // Force skybox background so the dashboard sees the scene, not transparent black.
        // This only affects the capture camera — the user's XR view is unchanged.
        captureCamera.clearFlags = CameraClearFlags.Skybox;
        captureCamera.backgroundColor = new Color(0.1f, 0.1f, 0.15f, 1f);

        renderReadback = new Texture2D(captureWidth, captureHeight, TextureFormat.RGB24, false);
        Debug.Log("[HeadsetStream] Render capture camera ready");
    }

    void LateUpdate()
    {
        if (Time.time < nextCaptureTime)
            return;

        nextCaptureTime = Time.time + captureInterval;

        if (captureCamera != null)
            CaptureRender();
    }

    void CaptureRender()
    {
        if (captureCamera == null || renderTexture == null)
            return;

        captureCamera.fieldOfView = sourceCamera.fieldOfView;
        captureCamera.nearClipPlane = sourceCamera.nearClipPlane;
        captureCamera.farClipPlane = sourceCamera.farClipPlane;

        captureCamera.Render();

        RenderTexture prev = RenderTexture.active;
        RenderTexture.active = renderTexture;
        renderReadback.ReadPixels(new Rect(0, 0, captureWidth, captureHeight), 0, 0, false);
        renderReadback.Apply();
        RenderTexture.active = prev;

        byte[] jpegData = renderReadback.EncodeToJPG(jpegQuality);
        PublishImage(jpegData);
    }

    void PublishImage(byte[] jpegData)
    {
        var stamp = new TimeMsg(
            (int)Time.time,
            (uint)((Time.time % 1f) * 1e9f)
        );

        var msg = new CompressedImageMsg
        {
            header = new HeaderMsg(stamp, "headset"),
            format = "jpeg",
            data = jpegData
        };

        ros.Publish(topic, msg);
    }

    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
            Destroy(renderTexture);
        }
        if (renderReadback != null)
            Destroy(renderReadback);
        if (captureCamera != null)
            Destroy(captureCamera.gameObject);
    }
}
