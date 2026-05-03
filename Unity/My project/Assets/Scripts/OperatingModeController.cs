using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// Publishes operating mode commands to the dashboard and subscribes to mode status.
/// Allows the Quest 3 operator to switch between TELEOP and MOVEIT modes.
/// Attach to any GameObject. RadialMenu calls SetMode() on toggle.
/// </summary>
public class OperatingModeController : MonoBehaviour
{
    [Header("ROS Topics")]
    public string commandTopic = "/holoassist/mode_command";
    public string statusTopic = "/holoassist/mode_status";

    [Header("Status")]
    [Tooltip("Current confirmed operating mode (read-only, updated from dashboard)")]
    public string currentMode = "TELEOP";

    public bool IsTeleop => currentMode == "TELEOP";
    public bool IsMoveit => currentMode == "MOVEIT";

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(commandTopic);
        ros.Subscribe<StringMsg>(statusTopic, OnModeStatus);
        Debug.Log("[OperatingModeController] Ready — publishing to " + commandTopic);
    }

    void OnModeStatus(StringMsg msg)
    {
        if (!string.IsNullOrEmpty(msg.data))
        {
            currentMode = msg.data.ToUpper();
        }
    }

    public void SetMode(string mode)
    {
        mode = mode.ToUpper();
        if (mode != "TELEOP" && mode != "MOVEIT") return;

        var msg = new StringMsg { data = mode };
        ros.Publish(commandTopic, msg);
        Debug.Log($"[OperatingModeController] Requested mode: {mode}");
    }

    public void ToggleMode()
    {
        SetMode(IsTeleop ? "MOVEIT" : "TELEOP");
    }
}
