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
    public float panelWidth = 0.25f;
    public float panelHeight = 0.1f;

    private TextMeshPro label;
    private Transform cam;

    void Start()
    {
        cam = Camera.main.transform;

        // Background quad
        var bg = GameObject.CreatePrimitive(PrimitiveType.Quad);
        bg.transform.SetParent(transform, false);
        bg.transform.localScale = new Vector3(panelWidth, panelHeight, 1f);
        var bgMat = bg.GetComponent<Renderer>().material;
        bgMat.color = new Color(0f, 0f, 0f, 0.6f);
        bgMat.SetFloat("_Mode", 3); // transparent
        bgMat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        bgMat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        bgMat.SetInt("_ZWrite", 0);
        bgMat.DisableKeyword("_ALPHATEST_ON");
        bgMat.EnableKeyword("_ALPHABLEND_ON");
        bgMat.DisableKeyword("_ALPHAPREMULTIPLY_ON");
        bgMat.renderQueue = 3000;
        Destroy(bg.GetComponent<Collider>());

        // Text
        var textObj = new GameObject("HUDText");
        textObj.transform.SetParent(transform, false);
        textObj.transform.localPosition = new Vector3(0f, 0f, -0.001f);
        label = textObj.AddComponent<TextMeshPro>();
        label.fontSize = 1.5f;
        label.alignment = TextAlignmentOptions.Center;
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
