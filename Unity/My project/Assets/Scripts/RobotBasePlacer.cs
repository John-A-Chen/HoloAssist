using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Hands;
using Unity.XR.CoreUtils;

/// <summary>
/// Attach to an EMPTY parent GameObject that contains the robot (and optionally the trolley).
/// Pinch near it to grab and drag the whole rig. This moves the parent transform only —
/// the ArticulationBody chain inside is never touched, so JointStateSubscriber keeps working.
///
/// Setup:
///   1. Create an empty GameObject called e.g. "RobotRig"
///   2. Drag the "ur" robot (and trolley) under it as children
///   3. Attach this script to "RobotRig"
/// </summary>
public class RobotBasePlacer : MonoBehaviour
{
    [Header("Hand Tracking")]
    public bool useRightHand = true;

    [Header("Grab Settings")]
    [Tooltip("Max finger-tip distance to count as a pinch (metres)")]
    public float pinchDistance = 0.05f;
    [Tooltip("How close your pinch must be to the rig origin to start a grab (metres)")]
    public float grabRadius = 1.5f;

    [Header("Placement")]
    public bool lockY = false;
    public float lockedYValue = 0f;
    public float smoothSpeed = 20f;

    [Header("Debug")]
    public bool showDebugLogs = true;

    private XRHandSubsystem handSubsystem;
    private bool isGrabbing = false;
    private Vector3 grabOffset;
    private Transform xrOrigin;
    private float debugTimer = 0f;

    void Start()
    {
        var origin = FindObjectOfType<XROrigin>();
        if (origin != null)
        {
            xrOrigin = origin.transform;
            Debug.Log($"[RobotBasePlacer] Found XR Origin at {xrOrigin.position}");
        }
        else
        {
            xrOrigin = Camera.main != null ? Camera.main.transform.parent : null;
            Debug.LogWarning("[RobotBasePlacer] No XROrigin found, using camera parent as fallback");
        }

        Debug.Log("[RobotBasePlacer] Script started on wrapper GameObject. Waiting for hand subsystem...");
    }

    void Update()
    {
        if (handSubsystem == null || !handSubsystem.running)
        {
            TryGetHandSubsystem();
            return;
        }

        XRHand hand = useRightHand ? handSubsystem.rightHand : handSubsystem.leftHand;

        if (!hand.isTracked)
        {
            debugTimer += Time.deltaTime;
            if (showDebugLogs && debugTimer > 3f)
            {
                Debug.Log($"[RobotBasePlacer] Hand NOT tracked. Show your {(useRightHand ? "right" : "left")} hand.");
                debugTimer = 0f;
            }
            return;
        }

        if (!TryGetJointWorldPos(hand, XRHandJointID.ThumbTip, out Vector3 thumbTip)) return;
        if (!TryGetJointWorldPos(hand, XRHandJointID.IndexTip, out Vector3 indexTip)) return;

        float tipDistance = Vector3.Distance(thumbTip, indexTip);
        bool isPinching = tipDistance < pinchDistance;
        Vector3 pinchPos = (thumbTip + indexTip) / 2f;
        float distToRig = Vector3.Distance(pinchPos, transform.position);

        debugTimer += Time.deltaTime;
        if (showDebugLogs && debugTimer > 2f)
        {
            Debug.Log($"[RobotBasePlacer] tipDist={tipDistance:F3}m pinchThresh={pinchDistance:F3}m " +
                      $"distToRig={distToRig:F2}m grabRadius={grabRadius:F2}m " +
                      $"isPinching={isPinching} isGrabbing={isGrabbing} pos={transform.position}");
            debugTimer = 0f;
        }

        if (!isGrabbing)
        {
            if (isPinching && distToRig <= grabRadius)
            {
                isGrabbing = true;
                grabOffset = transform.position - pinchPos;
                Debug.Log("[RobotBasePlacer] GRABBED rig");
            }
        }
        else
        {
            if (tipDistance > pinchDistance * 2.5f)
            {
                isGrabbing = false;
                Debug.Log($"[RobotBasePlacer] PLACED rig at {transform.position}");
                return;
            }

            Vector3 targetPos = pinchPos + grabOffset;

            if (lockY)
                targetPos.y = lockedYValue;

            transform.position = Vector3.Lerp(transform.position, targetPos, smoothSpeed * Time.deltaTime);
        }
    }

    bool TryGetJointWorldPos(XRHand hand, XRHandJointID jointId, out Vector3 worldPos)
    {
        XRHandJoint joint = hand.GetJoint(jointId);
        if (joint.TryGetPose(out Pose pose))
        {
            if (xrOrigin != null)
                worldPos = xrOrigin.TransformPoint(pose.position);
            else
                worldPos = pose.position;
            return true;
        }
        worldPos = Vector3.zero;
        return false;
    }

    void TryGetHandSubsystem()
    {
        var subsystems = new List<XRHandSubsystem>();
        SubsystemManager.GetSubsystems(subsystems);
        if (subsystems.Count > 0)
        {
            handSubsystem = subsystems[0];
            Debug.Log($"[RobotBasePlacer] Found hand subsystem (running={handSubsystem.running})");
        }
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = isGrabbing ? Color.green : Color.yellow;
        Gizmos.DrawWireSphere(transform.position, grabRadius);
    }
}
