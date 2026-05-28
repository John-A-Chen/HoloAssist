using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class SessionLogger : MonoBehaviour
{
    [Header("References")]
    public RobotController robotController;
    public MeshCollisionGuard collisionGuard;

    [Header("Settings")]
    [Tooltip("How often to publish session status to ROS (Hz).")]
    public float publishRate = 2f;

    // Session tracking
    private float sessionStartTime;
    private float publishTimer;
    private int modeSwitches;

    // Mode duration tracking
    private RobotController.ControlMode lastMode;
    private RobotController.RMRCSubMode lastSubMode;
    private Dictionary<string, float> modeDurations = new Dictionary<string, float>();
    private float lastModeChangeTime;

    // Collision tracking
    private bool wasBlocked;
    private float blockStartTime;
    private int collisionBlockCount;
    private float totalBlockedTime;

    // Gripper tracking
    private bool wasGripping;
    private int gripCount;
    private float gripStartTime;
    private float totalGripTime;

    // EE lock tracking
    private bool wasEELocked;
    private int eeLockCount;

    // E-stop tracking (count from mode)
    private int estopCount;

    // Event log
    private List<SessionEvent> events = new List<SessionEvent>();

    // ROS
    private ROSConnection ros;
    private const string STATUS_TOPIC = "/session/status";
    private const string EVENT_TOPIC = "/session/events";

    private struct SessionEvent
    {
        public float time;
        public string type;
        public string detail;
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(STATUS_TOPIC);
        ros.RegisterPublisher<StringMsg>(EVENT_TOPIC);

        sessionStartTime = Time.time;
        lastModeChangeTime = Time.time;

        if (robotController == null)
            robotController = FindObjectOfType<RobotController>();
        if (collisionGuard == null)
            collisionGuard = FindObjectOfType<MeshCollisionGuard>();

        if (robotController != null)
        {
            lastMode = robotController.CurrentMode;
            lastSubMode = robotController.CurrentRMRCSubMode;
        }

        modeDurations["RMRC_Translate"] = 0f;
        modeDurations["RMRC_Rotate"] = 0f;
        modeDurations["DirectJoint"] = 0f;
        modeDurations["HandGuide"] = 0f;

        LogEvent("session_start", "Session started");
    }

    void Update()
    {
        if (robotController == null) return;

        TrackModeChanges();
        TrackCollision();
        TrackGripper();
        TrackEELock();

        publishTimer += Time.deltaTime;
        if (publishTimer >= 1f / publishRate)
        {
            publishTimer = 0f;
            PublishStatus();
        }
    }

    void TrackModeChanges()
    {
        var currentMode = robotController.CurrentMode;
        var currentSubMode = robotController.CurrentRMRCSubMode;

        bool modeChanged = currentMode != lastMode;
        bool subModeChanged = currentMode == RobotController.ControlMode.RMRC && currentSubMode != lastSubMode;

        if (modeChanged || subModeChanged)
        {
            string prevKey = GetModeKey(lastMode, lastSubMode);
            modeDurations[prevKey] += Time.time - lastModeChangeTime;
            lastModeChangeTime = Time.time;

            modeSwitches++;
            string newKey = GetModeKey(currentMode, currentSubMode);
            LogEvent("mode_switch", prevKey + " -> " + newKey);

            lastMode = currentMode;
            lastSubMode = currentSubMode;
        }
    }

    void TrackCollision()
    {
        if (collisionGuard == null) return;

        bool isBlocked = collisionGuard.IsBlocked;
        float scale = collisionGuard.VelocityScale;

        if (isBlocked && !wasBlocked)
        {
            collisionBlockCount++;
            blockStartTime = Time.time;
            LogEvent("collision_blocked", "velocity zeroed");
        }
        else if (!isBlocked && wasBlocked)
        {
            float dur = Time.time - blockStartTime;
            totalBlockedTime += dur;
            LogEvent("collision_cleared", $"blocked {dur:F2}s");
        }
        wasBlocked = isBlocked;
    }

    void TrackGripper()
    {
        float grip = robotController.GripperValue;
        bool gripping = grip > 0.3f;

        if (gripping && !wasGripping)
        {
            gripCount++;
            gripStartTime = Time.time;
            LogEvent("gripper_close", $"{grip:P0}");
        }
        else if (!gripping && wasGripping)
        {
            float dur = Time.time - gripStartTime;
            totalGripTime += dur;
            LogEvent("gripper_open", $"held {dur:F1}s");
        }
        wasGripping = gripping;
    }

    void TrackEELock()
    {
        bool locked = robotController.IsEELockedDown;
        if (locked && !wasEELocked)
        {
            eeLockCount++;
            LogEvent("ee_lock_on", "EE pointing down");
        }
        else if (!locked && wasEELocked)
        {
            LogEvent("ee_lock_off", "EE free");
        }
        wasEELocked = locked;
    }

    string GetModeKey(RobotController.ControlMode mode, RobotController.RMRCSubMode subMode)
    {
        if (mode == RobotController.ControlMode.RMRC)
            return subMode == RobotController.RMRCSubMode.Translate ? "RMRC_Translate" : "RMRC_Rotate";
        return mode.ToString();
    }

    float GetModeDuration(string key)
    {
        float dur = 0f;
        if (modeDurations.ContainsKey(key))
            dur = modeDurations[key];
        if (robotController != null)
        {
            string currentKey = GetModeKey(robotController.CurrentMode, robotController.CurrentRMRCSubMode);
            if (key == currentKey)
                dur += Time.time - lastModeChangeTime;
        }
        return dur;
    }

    void PublishStatus()
    {
        if (robotController == null) return;

        float sessionTime = Time.time - sessionStartTime;
        float collisionScale = collisionGuard != null ? collisionGuard.VelocityScale : 1f;
        bool collisionBlocked = collisionGuard != null && collisionGuard.IsBlocked;

        var sb = new StringBuilder(512);
        sb.Append("{");
        sb.Append("\"mode\":\"").Append(robotController.CurrentMode).Append("\",");
        sb.Append("\"sub_mode\":\"").Append(robotController.CurrentRMRCSubMode).Append("\",");
        sb.Append("\"session_s\":").Append(sessionTime.ToString("F1")).Append(",");
        sb.Append("\"mode_switches\":").Append(modeSwitches).Append(",");
        sb.Append("\"hand_guide_active\":").Append(robotController.IsHandGuideActive ? "true" : "false").Append(",");
        sb.Append("\"ee_locked\":").Append(robotController.IsEELockedDown ? "true" : "false").Append(",");
        sb.Append("\"gripper_pct\":").Append((int)(robotController.GripperValue * 100)).Append(",");
        sb.Append("\"collision_scale\":").Append(collisionScale.ToString("F2")).Append(",");
        sb.Append("\"collision_blocked\":").Append(collisionBlocked ? "true" : "false").Append(",");
        sb.Append("\"collision_events\":").Append(collisionBlockCount).Append(",");
        sb.Append("\"gripper_grips\":").Append(gripCount).Append(",");
        sb.Append("\"ee_lock_count\":").Append(eeLockCount).Append(",");
        sb.Append("\"mode_durations\":{");
        sb.Append("\"RMRC_Translate\":").Append(GetModeDuration("RMRC_Translate").ToString("F1")).Append(",");
        sb.Append("\"RMRC_Rotate\":").Append(GetModeDuration("RMRC_Rotate").ToString("F1")).Append(",");
        sb.Append("\"DirectJoint\":").Append(GetModeDuration("DirectJoint").ToString("F1")).Append(",");
        sb.Append("\"HandGuide\":").Append(GetModeDuration("HandGuide").ToString("F1"));
        sb.Append("}}");

        ros.Publish(STATUS_TOPIC, new StringMsg(sb.ToString()));
    }

    void LogEvent(string type, string detail)
    {
        float elapsed = Time.time - sessionStartTime;
        events.Add(new SessionEvent { time = elapsed, type = type, detail = detail });

        string json = "{\"t\":" + elapsed.ToString("F2") +
            ",\"type\":\"" + type +
            "\",\"detail\":\"" + detail.Replace("\"", "'") + "\"}";
        ros.Publish(EVENT_TOPIC, new StringMsg(json));

        Debug.Log("[SessionLogger] " + type + ": " + detail);
    }

    void OnApplicationQuit()
    {
        if (robotController == null) return;

        string currentKey = GetModeKey(robotController.CurrentMode, robotController.CurrentRMRCSubMode);
        if (modeDurations.ContainsKey(currentKey))
            modeDurations[currentKey] += Time.time - lastModeChangeTime;

        if (wasGripping)
            totalGripTime += Time.time - gripStartTime;
        if (wasBlocked)
            totalBlockedTime += Time.time - blockStartTime;

        float sessionTime = Time.time - sessionStartTime;
        LogEvent("session_end", $"Duration: {sessionTime:F1}s");

        SaveSessionJSON();
        SaveSessionCSV();
    }

    void SaveSessionJSON()
    {
        string dir = Path.Combine(Application.persistentDataPath, "SessionLogs");
        Directory.CreateDirectory(dir);
        string filename = "session_" + DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss") + ".json";
        string path = Path.Combine(dir, filename);

        float sessionTime = Time.time - sessionStartTime;

        var sb = new StringBuilder(2048);
        sb.AppendLine("{");
        sb.AppendLine($"  \"timestamp\": \"{DateTime.Now:O}\",");
        sb.AppendLine($"  \"session_duration_s\": {sessionTime:F1},");
        sb.AppendLine($"  \"mode_switches\": {modeSwitches},");
        sb.AppendLine($"  \"collision_blocks\": {collisionBlockCount},");
        sb.AppendLine($"  \"collision_blocked_time_s\": {totalBlockedTime:F2},");
        sb.AppendLine($"  \"gripper_grips\": {gripCount},");
        sb.AppendLine($"  \"gripper_grip_time_s\": {totalGripTime:F1},");
        sb.AppendLine($"  \"ee_lock_toggles\": {eeLockCount},");
        sb.AppendLine("  \"mode_durations\": {");
        sb.AppendLine($"    \"RMRC_Translate\": {GetDuration("RMRC_Translate")},");
        sb.AppendLine($"    \"RMRC_Rotate\": {GetDuration("RMRC_Rotate")},");
        sb.AppendLine($"    \"DirectJoint\": {GetDuration("DirectJoint")},");
        sb.AppendLine($"    \"HandGuide\": {GetDuration("HandGuide")}");
        sb.AppendLine("  },");
        sb.Append("  \"events\": [");
        for (int i = 0; i < events.Count; i++)
        {
            if (i > 0) sb.Append(",");
            sb.Append($"\n    {{\"t\":{events[i].time:F2},\"type\":\"{events[i].type}\",\"detail\":\"{events[i].detail.Replace("\"", "'")}\"}}");
        }
        sb.AppendLine("\n  ]");
        sb.AppendLine("}");

        File.WriteAllText(path, sb.ToString());
        Debug.Log("[SessionLogger] JSON saved: " + path);
    }

    void SaveSessionCSV()
    {
        string dir = Path.Combine(Application.persistentDataPath, "SessionLogs");
        Directory.CreateDirectory(dir);
        string filename = "session_" + DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss") + ".csv";
        string path = Path.Combine(dir, filename);

        var sb = new StringBuilder(1024);
        sb.AppendLine("time_s,event_type,detail");
        for (int i = 0; i < events.Count; i++)
            sb.AppendLine($"{events[i].time:F2},{events[i].type},\"{events[i].detail}\"");

        File.WriteAllText(path, sb.ToString());
        Debug.Log("[SessionLogger] CSV saved: " + path);
    }

    string GetDuration(string key)
    {
        return modeDurations.ContainsKey(key) ? modeDurations[key].ToString("F1") : "0.0";
    }
}
