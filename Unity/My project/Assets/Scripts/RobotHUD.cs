using UnityEngine;
using TMPro;

public class RobotHUD : MonoBehaviour
{
    [Header("References")]
    public RobotController controller;

    [Header("Layout")]
    public float distanceFromCamera = 1.5f;
    public Vector3 offset = new Vector3(0.3f, -0.2f, 0f); // bottom-right of view
    public float followSpeed = 3f;
    public float panelWidth = 2.5f;
    public float panelHeight = 0.08f;

    private TextMeshPro label;
    private Transform cam;

    void Start()
    {
        cam = Camera.main.transform;

        // Background quad
        var bg = GameObject.CreatePrimitive(PrimitiveType.Quad);
        bg.transform.SetParent(transform, false);
        bg.transform.localScale = new Vector3(panelWidth + 0.04f, panelHeight + 0.02f, 1f);
        var bgMat = new Material(Shader.Find("Universal Render Pipeline/Unlit"));
        bgMat.SetFloat("_Surface", 1); // transparent
        bgMat.SetFloat("_Blend", 0);   // alpha blend
        bgMat.SetColor("_BaseColor", new Color(0.05f, 0.05f, 0.1f, 0.85f));
        bgMat.SetFloat("_ZWrite", 0);
        bgMat.SetOverrideTag("RenderType", "Transparent");
        bgMat.renderQueue = 3000;
        bgMat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
        bg.GetComponent<Renderer>().material = bgMat;
        Destroy(bg.GetComponent<Collider>());

        // Text
        var textObj = new GameObject("HUDText");
        textObj.transform.SetParent(transform, false);
        textObj.transform.localPosition = new Vector3(0f, 0f, -0.001f);
        label = textObj.AddComponent<TextMeshPro>();
        label.fontSize = 0.3f;
        label.alignment = TextAlignmentOptions.Center;
        label.enableWordWrapping = false;
        label.overflowMode = TextOverflowModes.Overflow;
        label.color = Color.white;
        label.rectTransform.sizeDelta = new Vector2(panelWidth, panelHeight);

        // Initial position
        UpdatePosition(true);
    }

    void LateUpdate()
    {
        if (controller == null || cam == null) return;

        UpdatePosition(false);
        UpdateText();
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

    void UpdateText()
    {
        string modeStr = controller.CurrentMode == RobotController.ControlMode.DirectJoint
            ? "JOINT" : "SERVO";

        if (controller.CurrentMode == RobotController.ControlMode.DirectJoint)
        {
            label.text = $"{modeStr}  |  {controller.SelectedJointName}";
        }
        else
        {
            label.text = $"{modeStr}  |  Cartesian";
        }
    }
}
