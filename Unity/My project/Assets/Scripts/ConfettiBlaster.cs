using UnityEngine;

/// <summary>
/// Confetti blaster — fires a one-shot particle burst upward when a cube
/// enters a bin. Place this GameObject inside the trolley pointing up.
///
/// Setup:
///   1. Create empty GameObject as a child of the trolley.
///   2. Add this component.
///   3. Either:
///        a) Assign Confetti.prefab (Assets/MRTemplateAssets/Prefabs/Blaster/Confetti.prefab)
///           to confettiPrefab — it will be instantiated as a child at startup.
///        b) Or drop a ParticleSystem GameObject as a child and assign it to particleSystemRef.
///   4. Rotate so its forward (or particle Z+) points UP.
///
/// Toggled on/off from RadialMenu page 3.
/// </summary>
public class ConfettiBlaster : MonoBehaviour
{
    public static ConfettiBlaster Instance { get; private set; }

    [Header("References")]
    [Tooltip("Optional: confetti prefab to instantiate as a child on Awake. If null, particleSystemRef is used directly.")]
    public GameObject confettiPrefab;

    [Tooltip("Optional: existing ParticleSystem to drive. Auto-found in children if blank.")]
    public ParticleSystem particleSystemRef;

    [Tooltip("Optional: AudioSource for the launch sound. Auto-found in children if blank (Confetti.prefab ships with one).")]
    public AudioSource audioSourceRef;

    [Range(0f, 1f)]
    public float audioVolume = 1f;

    [Header("Settings")]
    [Tooltip("Whether the blaster fires when a cube enters a bin. Toggle from radial menu.")]
    public bool enabledOnStart = true;

    [Tooltip("Minimum seconds between bursts (debounce so a stack of cubes doesn't spam).")]
    public float minFireInterval = 0.3f;

    public bool Enabled { get; private set; }

    private float lastFireTime = -999f;

    void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Debug.LogWarning($"[ConfettiBlaster] Multiple instances in scene — keeping '{Instance.gameObject.name}', destroying '{gameObject.name}'.");
            Destroy(this);
            return;
        }
        Instance = this;
        Enabled = enabledOnStart;
    }

    void Start()
    {
        if (particleSystemRef == null && confettiPrefab != null)
        {
            var spawned = Instantiate(confettiPrefab, transform);
            spawned.transform.localPosition = Vector3.zero;
            spawned.transform.localRotation = Quaternion.identity;
            spawned.name = "Confetti_Instance";
            particleSystemRef = spawned.GetComponentInChildren<ParticleSystem>();
            if (audioSourceRef == null)
                audioSourceRef = spawned.GetComponentInChildren<AudioSource>();
        }

        if (particleSystemRef == null)
            particleSystemRef = GetComponentInChildren<ParticleSystem>();
        if (audioSourceRef == null)
            audioSourceRef = GetComponentInChildren<AudioSource>();

        if (particleSystemRef == null)
        {
            Debug.LogError("[ConfettiBlaster] No ParticleSystem found — assign confettiPrefab or particleSystemRef.");
            return;
        }

        // Confetti.prefab has playOnAwake=true on both the ParticleSystem and AudioSource;
        // suppress the initial burst + sound.
        particleSystemRef.Stop(true, ParticleSystemStopBehavior.StopEmittingAndClear);
        if (audioSourceRef != null)
        {
            audioSourceRef.playOnAwake = false;
            audioSourceRef.Stop();
        }
    }

    void OnDestroy()
    {
        if (Instance == this) Instance = null;
    }

    /// <summary>Fire one burst, respecting the Enabled flag and debounce.</summary>
    public void Fire()
    {
        if (!Enabled) return;
        if (particleSystemRef == null) return;
        if (Time.time - lastFireTime < minFireInterval) return;

        lastFireTime = Time.time;
        particleSystemRef.Stop(true, ParticleSystemStopBehavior.StopEmittingAndClear);
        particleSystemRef.Play(true);
        if (audioSourceRef != null && audioSourceRef.clip != null)
        {
            audioSourceRef.volume = audioVolume;
            audioSourceRef.PlayOneShot(audioSourceRef.clip, audioVolume);
        }
    }

    public void SetEnabled(bool value)
    {
        Enabled = value;
        if (!value && particleSystemRef != null)
            particleSystemRef.Stop(true, ParticleSystemStopBehavior.StopEmittingAndClear);
        Debug.Log($"[ConfettiBlaster] {(value ? "ON" : "OFF")}");
    }

    public void Toggle() => SetEnabled(!Enabled);

    /// <summary>Static convenience for callers that don't hold a reference.</summary>
    public static void FireIfEnabled()
    {
        if (Instance != null) Instance.Fire();
    }
}
