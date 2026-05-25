using UnityEngine;

/// <summary>
/// Attach to any GameObject you want JointTFVisualizer to always show TF axes on,
/// even if the object would otherwise be skipped by the auto-discovery filters
/// (e.g. it's a child of a mesh, under MR Interaction Setup, or its parent name
/// contains "Controller"). Pure marker — no fields, no Update.
///
/// Example: attach to the moveable interactable sphere in the scene.
/// </summary>
public class TFAxisMarker : MonoBehaviour
{
}
