using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Attach to an EMPTY parent GameObject that contains the robot (and optionally the trolley).
/// Hold grip on a controller near the rig to grab and drag the whole rig.
/// This moves the parent transform only — the robot chain inside is untouched.
///
/// Setup:
///   1. Create an empty GameObject called e.g. "RobotRig"
///   2. Drag the "ur" robot (and trolley) under it as children
///   3. Attach this script to "RobotRig"
/// </summary>
public class RobotBasePlacer : MonoBehaviour
{
    [Header("Controller Settings")]
    [Tooltip("Which hand to use for grabbing")]
    public bool useRightHand = true;

    [Header("Grab Settings")]
    [Tooltip("How close the controller must be to the rig origin to start a grab (metres)")]
    public float grabRadius = 1.5f;

    [Header("Placement")]
    public bool lockY = false;
    public float lockedYValue = 0f;
    public float smoothSpeed = 20f;

    [Header("Rotation")]
    [Tooltip("Allow rotating the rig while grabbed (controller rotation maps to rig Y-rotation)")]
    public bool allowRotation = true;

    private InputAction gripAction;
    private InputAction positionAction;
    private InputAction rotationAction;

    private bool isGrabbing = false;
    private Vector3 grabOffset;
    private Quaternion grabControllerRot;
    private Quaternion grabRigRot;

    void Start()
    {
        string hand = useRightHand ? "RightHand" : "LeftHand";

        gripAction = new InputAction("PlacerGrip", InputActionType.Value,
            $"<XRController>{{{hand}}}/grip");
        positionAction = new InputAction("PlacerPos", InputActionType.Value,
            $"<XRController>{{{hand}}}/devicePosition");
        rotationAction = new InputAction("PlacerRot", InputActionType.Value,
            $"<XRController>{{{hand}}}/deviceRotation");

        gripAction.Enable();
        positionAction.Enable();
        rotationAction.Enable();

        Debug.Log($"[RobotBasePlacer] Using {hand} controller. Grip near rig to grab.");
    }

    void Update()
    {
        float grip = gripAction.ReadValue<float>();
        Vector3 controllerPos = positionAction.ReadValue<Vector3>();

        // Controller position is in tracking space — convert to world via XR Origin
        Transform trackingSpace = Camera.main != null ? Camera.main.transform.parent : null;
        Vector3 worldPos = trackingSpace != null
            ? trackingSpace.TransformPoint(controllerPos)
            : controllerPos;

        bool gripHeld = grip > 0.5f;

        if (!isGrabbing)
        {
            if (gripHeld)
            {
                float dist = Vector3.Distance(worldPos, transform.position);
                if (dist <= grabRadius)
                {
                    isGrabbing = true;
                    grabOffset = transform.position - worldPos;

                    if (allowRotation)
                    {
                        Quaternion controllerRot = rotationAction.ReadValue<Quaternion>();
                        grabControllerRot = trackingSpace != null
                            ? trackingSpace.rotation * controllerRot
                            : controllerRot;
                        grabRigRot = transform.rotation;
                    }

                    Debug.Log("[RobotBasePlacer] GRABBED rig");
                }
            }
        }
        else
        {
            if (!gripHeld)
            {
                isGrabbing = false;
                Debug.Log($"[RobotBasePlacer] PLACED rig at {transform.position}");
                return;
            }

            // Position
            Vector3 targetPos = worldPos + grabOffset;
            if (lockY)
                targetPos.y = lockedYValue;
            transform.position = Vector3.Lerp(transform.position, targetPos, smoothSpeed * Time.deltaTime);

            // Rotation (Y-axis only to keep robot upright)
            if (allowRotation)
            {
                Quaternion controllerRot = rotationAction.ReadValue<Quaternion>();
                Quaternion currentWorldRot = trackingSpace != null
                    ? trackingSpace.rotation * controllerRot
                    : controllerRot;

                Quaternion deltaRot = currentWorldRot * Quaternion.Inverse(grabControllerRot);

                // Extract only Y-axis rotation to keep the robot upright
                Vector3 euler = deltaRot.eulerAngles;
                Quaternion yOnly = Quaternion.Euler(0f, euler.y, 0f);

                Quaternion targetRot = yOnly * grabRigRot;
                transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, smoothSpeed * Time.deltaTime);
            }
        }
    }

    void OnDestroy()
    {
        gripAction?.Disable();
        gripAction?.Dispose();
        positionAction?.Disable();
        positionAction?.Dispose();
        rotationAction?.Disable();
        rotationAction?.Dispose();
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = isGrabbing ? Color.green : Color.yellow;
        Gizmos.DrawWireSphere(transform.position, grabRadius);
    }
}
