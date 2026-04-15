using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// Tracks session metrics and publishes them to ROS for the dashboard.
/// Attach to any GameObject; assign RobotController reference in Inspector.
/// Saves a JSON session log to Application.persistentDataPath/SessionLogs/ on quit.
/// </summary>
public class SessionLogger : MonoBehaviour
{
    [Header("References")]
    public RobotController robotController;

    [Header("Settings")]
    [Tooltip("How often to publish session status to ROS (Hz).")]
    public float publishRate = 2f;

    // ── Session tracking ──
    private float sessionStartTime;
    private float publishTimer;
    private int modeSwitches;
    private int cmdPublishCount;

    // Mode duration tracking
    private RobotController.ControlMode lastMode;
    private RobotController.RMRCSubMode lastSubMode;
    private Dictionary<string, float> modeDurations = new Dictionary<string, float>();
    private float lastModeChangeTime;

    // Event log
    private List<string> eventTimestamps = new List<string>();
    private List<string> eventTypes = new List<string>();
    private List<string> eventDetails = new List<string>();

    // ROS
    private ROSConnection ros;
    private const string STATUS_TOPIC = "/session/status";
    private const string EVENT_TOPIC = "/session/events";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(STATUS_TOPIC);
        ros.RegisterPublisher<StringMsg>(EVENT_TOPIC);

        sessionStartTime = Time.time;
        lastModeChangeTime = Time.time;

        if (robotController == null)
            robotController = FindObjectOfType<RobotController>();

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

        // Detect mode changes
        var currentMode = robotController.CurrentMode;
        var currentSubMode = robotController.CurrentRMRCSubMode;

        bool modeChanged = currentMode != lastMode;
        bool subModeChanged = currentMode == RobotController.ControlMode.RMRC && currentSubMode != lastSubMode;

        if (modeChanged || subModeChanged)
        {
            // Accumulate time for previous mode
            string prevKey = GetModeKey(lastMode, lastSubMode);
            modeDurations[prevKey] += Time.time - lastModeChangeTime;
            lastModeChangeTime = Time.time;

            modeSwitches++;
            string newKey = GetModeKey(currentMode, currentSubMode);
            LogEvent("mode_switch", prevKey + " -> " + newKey);

            lastMode = currentMode;
            lastSubMode = currentSubMode;
        }

        // Publish status at configured rate
        publishTimer += Time.deltaTime;
        if (publishTimer >= 1f / publishRate)
        {
            publishTimer = 0f;
            PublishStatus();
        }
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

        // Add ongoing time if this is the currently active mode
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

        string json = "{" +
            "\"mode\":\"" + robotController.CurrentMode + "\"," +
            "\"sub_mode\":\"" + robotController.CurrentRMRCSubMode + "\"," +
            "\"session_s\":" + sessionTime.ToString("F1") + "," +
            "\"mode_switches\":" + modeSwitches + "," +
            "\"hand_guide_active\":" + (robotController.IsHandGuideActive ? "true" : "false") + "," +
            "\"mode_durations\":{" +
                "\"RMRC_Translate\":" + GetModeDuration("RMRC_Translate").ToString("F1") + "," +
                "\"RMRC_Rotate\":" + GetModeDuration("RMRC_Rotate").ToString("F1") + "," +
                "\"DirectJoint\":" + GetModeDuration("DirectJoint").ToString("F1") + "," +
                "\"HandGuide\":" + GetModeDuration("HandGuide").ToString("F1") +
            "}" +
        "}";

        ros.Publish(STATUS_TOPIC, new StringMsg(json));
    }

    void LogEvent(string type, string detail)
    {
        float elapsed = Time.time - sessionStartTime;
        eventTimestamps.Add(elapsed.ToString("F2"));
        eventTypes.Add(type);
        eventDetails.Add(detail);

        // Publish to ROS
        string json = "{\"t\":" + elapsed.ToString("F2") +
            ",\"type\":\"" + type +
            "\",\"detail\":\"" + detail + "\"}";
        ros.Publish(EVENT_TOPIC, new StringMsg(json));

        Debug.Log("[SessionLogger] " + type + ": " + detail);
    }

    void OnApplicationQuit()
    {
        if (robotController == null) return;

        // Final mode duration update
        string currentKey = GetModeKey(robotController.CurrentMode, robotController.CurrentRMRCSubMode);
        if (modeDurations.ContainsKey(currentKey))
            modeDurations[currentKey] += Time.time - lastModeChangeTime;

        float sessionTime = Time.time - sessionStartTime;
        LogEvent("session_end", "Duration: " + sessionTime.ToString("F1") + "s");
        SaveSessionLog();
    }

    void SaveSessionLog()
    {
        string dir = Path.Combine(Application.persistentDataPath, "SessionLogs");
        Directory.CreateDirectory(dir);

        string filename = "session_" + DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss") + ".json";
        string path = Path.Combine(dir, filename);

        float sessionTime = Time.time - sessionStartTime;

        // Build events JSON array
        string eventsJson = "[";
        for (int i = 0; i < eventTimestamps.Count; i++)
        {
            if (i > 0) eventsJson += ",";
            eventsJson += "{\"t\":" + eventTimestamps[i] +
                ",\"type\":\"" + eventTypes[i] +
                "\",\"detail\":\"" + eventDetails[i] + "\"}";
        }
        eventsJson += "]";

        string json = "{\n" +
            "  \"session_duration_s\": " + sessionTime.ToString("F1") + ",\n" +
            "  \"mode_switches\": " + modeSwitches + ",\n" +
            "  \"mode_durations\": {\n" +
            "    \"RMRC_Translate\": " + GetDuration("RMRC_Translate") + ",\n" +
            "    \"RMRC_Rotate\": " + GetDuration("RMRC_Rotate") + ",\n" +
            "    \"DirectJoint\": " + GetDuration("DirectJoint") + ",\n" +
            "    \"HandGuide\": " + GetDuration("HandGuide") + "\n" +
            "  },\n" +
            "  \"events\": " + eventsJson + "\n" +
            "}";

        File.WriteAllText(path, json);
        Debug.Log("[SessionLogger] Session log saved to " + path);
    }

    string GetDuration(string key)
    {
        return modeDurations.ContainsKey(key) ? modeDurations[key].ToString("F1") : "0.0";
    }
}
