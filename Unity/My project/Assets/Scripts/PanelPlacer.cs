using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Interactables;

/// <summary>
/// Makes any UI panel grabbable using Unity XR Interaction Toolkit's XRGrabInteractable.
/// Works with the same XR ray + grip controls as the built-in XR template UI panels.
///
/// On Start: adds a BoxCollider sized to the panel and an XRGrabInteractable.
/// While grabbed: disables the panel's follow-camera behavior so it stays where placed.
/// </summary>
[RequireComponent(typeof(MonoBehaviour))]
public class PanelPlacer : MonoBehaviour
{
    [Header("Collider Sizing")]
    [Tooltip("Width of the grab collider (matches panel width)")]
    public float colliderWidth = 0.8f;

    [Tooltip("Height of the grab collider (matches panel height)")]
    public float colliderHeight = 0.6f;

    [Tooltip("Depth/thickness of the grab collider")]
    public float colliderDepth = 0.05f;

    [Header("Behavior")]
    [Tooltip("Re-enable camera follow when reset is requested")]
    public bool returnToFollowOnReset = false;

    private XRGrabInteractable grabInteractable;
    private BoxCollider grabCollider;
    private Rigidbody panelRigidbody;
    private MonoBehaviour panelBehavior;
    private TaskConfigPanel taskConfigPanel;   // special-cased: needs Update to keep running for button hover feedback
    private RobotHUD robotHud;                 // also flip-don't-disable so subscriptions + text keep updating after grab
    private RobotInfoPanel robotInfoPanel;     // same pattern as robotHud
    private bool wasFollowing;

    void Start()
    {
        // Three categories of panel-grab handling:
        //
        // 1. TaskConfigPanel — keep script enabled (interactive buttons need its Update),
        //    just flip its `followCamera` field on grab.
        // 2. RobotHUD / RobotInfoPanel — keep script enabled (live ROS data must keep
        //    flowing into the text), just flip their `followCamera` field on grab.
        //    DISABLING THESE BREAKS DATA UPDATES PERMANENTLY since LateUpdate stops.
        // 3. Other display-only panels — disable the whole MonoBehaviour on grab.
        taskConfigPanel = GetComponent<TaskConfigPanel>();
        robotHud = GetComponent<RobotHUD>();
        robotInfoPanel = GetComponent<RobotInfoPanel>();

        if (taskConfigPanel == null && robotHud == null && robotInfoPanel == null)
        {
            panelBehavior = GetComponent<RobotDataPanel>() as MonoBehaviour;
            if (panelBehavior == null) panelBehavior = GetComponent<BinStatusPanel>() as MonoBehaviour;
            if (panelBehavior == null) panelBehavior = GetComponent<CoachingPanel>() as MonoBehaviour;
        }

        // Add box collider sized to panel
        grabCollider = GetComponent<BoxCollider>();
        if (grabCollider == null)
            grabCollider = gameObject.AddComponent<BoxCollider>();
        grabCollider.size = new Vector3(colliderWidth, colliderHeight, colliderDepth);
        grabCollider.center = Vector3.zero;
        grabCollider.isTrigger = false;

        // Need a Rigidbody for XRGrabInteractable
        panelRigidbody = GetComponent<Rigidbody>();
        if (panelRigidbody == null)
            panelRigidbody = gameObject.AddComponent<Rigidbody>();
        panelRigidbody.useGravity = false;
        panelRigidbody.isKinematic = true; // we don't want physics simulation
        panelRigidbody.constraints = RigidbodyConstraints.FreezeAll; // hold position when not grabbed

        // Add XR Grab Interactable
        grabInteractable = GetComponent<XRGrabInteractable>();
        if (grabInteractable == null)
            grabInteractable = gameObject.AddComponent<XRGrabInteractable>();

        // Configure: keep world position on grab, no throw on release
        grabInteractable.movementType = XRBaseInteractable.MovementType.Instantaneous;
        grabInteractable.throwOnDetach = false;
        grabInteractable.trackPosition = true;
        grabInteractable.trackRotation = true;

        // Listen for grab/release events
        grabInteractable.selectEntered.AddListener(OnGrabbed);
        grabInteractable.selectExited.AddListener(OnReleased);
    }

    void OnGrabbed(SelectEnterEventArgs args)
    {
        // Flip the followCamera flag for panels that need to keep updating after grab.
        if (taskConfigPanel != null && taskConfigPanel.followCamera)
        {
            wasFollowing = true;
            taskConfigPanel.followCamera = false;
        }
        else if (robotHud != null && robotHud.followCamera)
        {
            wasFollowing = true;
            robotHud.followCamera = false;
        }
        else if (robotInfoPanel != null && robotInfoPanel.followCamera)
        {
            wasFollowing = true;
            robotInfoPanel.followCamera = false;
        }
        // Other panels (display-only): disable the entire script.
        else if (panelBehavior != null && panelBehavior.enabled)
        {
            wasFollowing = true;
            panelBehavior.enabled = false;
        }

        // Allow movement during grab
        if (panelRigidbody != null)
            panelRigidbody.constraints = RigidbodyConstraints.None;

        Debug.Log($"[PanelPlacer] Grabbed {gameObject.name}");
    }

    void OnReleased(SelectExitEventArgs args)
    {
        // Lock position so panel stays put
        if (panelRigidbody != null)
            panelRigidbody.constraints = RigidbodyConstraints.FreezeAll;

        // Optionally re-enable follow on release (default off — panel stays placed).
        if (returnToFollowOnReset && wasFollowing)
        {
            if (taskConfigPanel != null) taskConfigPanel.followCamera = true;
            else if (robotHud != null) robotHud.followCamera = true;
            else if (robotInfoPanel != null) robotInfoPanel.followCamera = true;
            else if (panelBehavior != null) panelBehavior.enabled = true;
        }

        Debug.Log($"[PanelPlacer] Released {gameObject.name} at {transform.position}");
    }

    /// <summary>
    /// Call this to re-enable camera-follow behavior on the panel.
    /// </summary>
    public void ResetToFollow()
    {
        if (taskConfigPanel != null) taskConfigPanel.followCamera = true;
        if (robotHud != null) robotHud.followCamera = true;
        if (robotInfoPanel != null) robotInfoPanel.followCamera = true;
        if (panelBehavior != null) panelBehavior.enabled = true;
        Debug.Log($"[PanelPlacer] Reset {gameObject.name} to follow camera");
    }

    void OnDestroy()
    {
        if (grabInteractable != null)
        {
            grabInteractable.selectEntered.RemoveListener(OnGrabbed);
            grabInteractable.selectExited.RemoveListener(OnReleased);
        }
    }

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.DrawWireCube(Vector3.zero, new Vector3(colliderWidth, colliderHeight, colliderDepth));
    }
}
