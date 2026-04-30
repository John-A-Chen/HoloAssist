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
    private bool wasFollowing;

    void Start()
    {
        // Find which panel script is on this object so we can disable it during grab
        panelBehavior = GetComponent<RobotDataPanel>() as MonoBehaviour;
        if (panelBehavior == null) panelBehavior = GetComponent<BinStatusPanel>() as MonoBehaviour;
        if (panelBehavior == null) panelBehavior = GetComponent<CoachingPanel>() as MonoBehaviour;

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
        // Disable follow-camera while grabbed
        if (panelBehavior != null && panelBehavior.enabled)
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

        // If user wants to re-enable follow when released, do so
        // Otherwise leave it disabled — panel stays placed
        if (returnToFollowOnReset && panelBehavior != null && wasFollowing)
        {
            panelBehavior.enabled = true;
        }

        Debug.Log($"[PanelPlacer] Released {gameObject.name} at {transform.position}");
    }

    /// <summary>
    /// Call this to re-enable camera-follow behavior on the panel.
    /// </summary>
    public void ResetToFollow()
    {
        if (panelBehavior != null)
            panelBehavior.enabled = true;
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
