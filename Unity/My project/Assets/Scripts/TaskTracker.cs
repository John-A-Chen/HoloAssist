using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Aggregates correct/wrong sorts across all BinDetectors and drives the
/// EventBanner's progress ring + text. Completion mode is configurable.
///
/// Receives events from BinDetector.HandleEntry via OnSort().
/// Drives EventBanner via SetProgress / SetText / SetTaskComplete.
/// Fires ConfettiBlaster on completion (one celebratory burst).
///
/// Setup: drop on any GameObject in the scene. Optional — without it,
/// BinDetector still produces local correct/wrong feedback (banner flash +
/// confetti) but no global progress is tracked.
/// </summary>
public class TaskTracker : MonoBehaviour
{
    public static TaskTracker Instance { get; private set; }

    public enum CompletionMode
    {
        [Tooltip("Task complete when correct-sort count reaches the target.")]
        FixedTarget,
        [Tooltip("Task complete when every Good ObjectClass in the scene has been sorted.")]
        SortAllGood,
        [Tooltip("Task complete when at least one of every known className has been correctly sorted.")]
        OneOfEachClass,
    }

    public enum TaskState { Idle, InProgress, Complete, Failed }

    [Header("Task Configuration")]
    [Tooltip("How task completion is evaluated. Switchable at runtime via TaskConfigPanel.")]
    public CompletionMode mode = CompletionMode.FixedTarget;

    [Tooltip("Required correct sorts for FixedTarget mode.")]
    public int targetCount = 5;

    [Tooltip("Wrong-sort count that flips the task to Failed. -1 disables auto-fail.")]
    public int wrongFailThreshold = -1;

    [Header("State (read-only)")]
    [SerializeField] private int totalCorrect = 0;
    [SerializeField] private int totalWrong = 0;
    [SerializeField] private TaskState state = TaskState.Idle;

    private readonly HashSet<string> classesSeenCorrect = new HashSet<string>();
    // Used by SortAllGood — cached on demand so spawned objects are counted.
    private int cachedGoodCount = -1;
    private float cachedGoodAt = -1f;

    public int TotalCorrect => totalCorrect;
    public int TotalWrong => totalWrong;
    public TaskState State => state;
    public float Progress { get; private set; }

    void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Debug.LogWarning($"[TaskTracker] Multiple instances — keeping '{Instance.gameObject.name}', destroying '{gameObject.name}'.");
            Destroy(this);
            return;
        }
        Instance = this;
    }

    void Start()
    {
        Recompute();
    }

    void OnDestroy()
    {
        if (Instance == this) Instance = null;
    }

    /// <summary>Called by BinDetector after each classified entry.</summary>
    public void OnSort(BinDetector bin, ObjectClass obj, bool isGood)
    {
        if (obj == null) return;

        if (isGood)
        {
            totalCorrect++;
            if (!string.IsNullOrEmpty(obj.className))
                classesSeenCorrect.Add(obj.className);
        }
        else
        {
            totalWrong++;
        }

        Recompute();
    }

    /// <summary>Zero all counters and return to Idle. Optionally resets per-bin scores too.</summary>
    public void ResetTask(bool resetBins = true)
    {
        totalCorrect = 0;
        totalWrong = 0;
        classesSeenCorrect.Clear();
        cachedGoodCount = -1;
        state = TaskState.Idle;

        if (resetBins)
        {
            foreach (var bd in FindObjectsByType<BinDetector>(FindObjectsSortMode.None))
                bd.ResetScore();
        }

        Recompute();
        Debug.Log("[TaskTracker] Task reset.");
    }

    public void SetMode(CompletionMode newMode)
    {
        if (mode == newMode) return;
        mode = newMode;
        Debug.Log($"[TaskTracker] Completion mode → {mode}");
        Recompute();
    }

    public void SetTarget(int n)
    {
        targetCount = Mathf.Max(1, n);
        Recompute();
    }

    void Recompute()
    {
        // 1. Update derived progress based on current mode.
        switch (mode)
        {
            case CompletionMode.FixedTarget:
                Progress = targetCount > 0 ? Mathf.Clamp01((float)totalCorrect / targetCount) : 0f;
                break;

            case CompletionMode.SortAllGood:
                int total = GetGoodObjectCountInScene();
                Progress = total > 0 ? Mathf.Clamp01((float)totalCorrect / total) : 0f;
                break;

            case CompletionMode.OneOfEachClass:
                int distinct = CountDistinctGoodClassesInScene();
                Progress = distinct > 0 ? Mathf.Clamp01((float)classesSeenCorrect.Count / distinct) : 0f;
                break;
        }

        // 2. Update state.
        TaskState newState = state;
        if (state != TaskState.Failed && state != TaskState.Complete)
        {
            if (totalCorrect == 0 && totalWrong == 0) newState = TaskState.Idle;
            else newState = TaskState.InProgress;
        }
        if (Progress >= 1f && state != TaskState.Failed) newState = TaskState.Complete;
        if (wrongFailThreshold >= 0 && totalWrong >= wrongFailThreshold) newState = TaskState.Failed;

        bool stateChanged = state != newState;
        state = newState;

        // 3. Drive EventBanner.
        var banner = EventBanner.Instance;
        if (banner != null)
        {
            banner.SetProgress(Progress);
            banner.SetTaskComplete(state == TaskState.Complete);
            banner.SetTaskFailed(state == TaskState.Failed);
            banner.SetText(BuildStatusText());

            if (stateChanged && state == TaskState.Complete)
            {
                banner.Flash(banner.correctFlashColor);
                ConfettiBlaster.FireIfEnabled();
            }
            else if (stateChanged && state == TaskState.Failed)
            {
                banner.Flash(banner.wrongFlashColor);
            }
        }
    }

    string BuildStatusText()
    {
        switch (state)
        {
            case TaskState.Complete: return "COMPLETE!";
            case TaskState.Failed: return $"FAILED ✗{totalWrong}";
        }
        switch (mode)
        {
            case CompletionMode.FixedTarget:
                return $"{totalCorrect} / {targetCount}";
            case CompletionMode.SortAllGood:
                int t = GetGoodObjectCountInScene();
                return t > 0 ? $"{totalCorrect} / {t}" : $"{totalCorrect}";
            case CompletionMode.OneOfEachClass:
                int d = CountDistinctGoodClassesInScene();
                return d > 0 ? $"{classesSeenCorrect.Count} / {d}" : $"{classesSeenCorrect.Count}";
        }
        return totalCorrect.ToString();
    }

    int GetGoodObjectCountInScene()
    {
        // Cache for ~1s to avoid scanning every Recompute (spawn-friendly).
        if (cachedGoodCount >= 0 && Time.time - cachedGoodAt < 1f) return cachedGoodCount;
        int n = 0;
        foreach (var oc in FindObjectsByType<ObjectClass>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            if (oc.isGood) n++;
        cachedGoodCount = n;
        cachedGoodAt = Time.time;
        return n;
    }

    int CountDistinctGoodClassesInScene()
    {
        var s = new HashSet<string>();
        foreach (var oc in FindObjectsByType<ObjectClass>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            if (oc.isGood && !string.IsNullOrEmpty(oc.className))
                s.Add(oc.className);
        return s.Count;
    }
}
