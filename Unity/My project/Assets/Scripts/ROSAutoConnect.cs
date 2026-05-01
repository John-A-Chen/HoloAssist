using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

[DefaultExecutionOrder(-50)]
public class ROSAutoConnect : MonoBehaviour
{
    [SerializeField] int rosPort = 10000;
    [SerializeField] int connectTimeoutMs = 300;

    volatile string foundIP;
    Thread scanThread;
    bool connected;

    // Scan 192.168.0.101–109, skip .100 (robot Ethernet)
    static readonly int RangeStart = 101;
    static readonly int RangeEnd = 109;

    void Awake()
    {
        var ros = ROSConnection.GetOrCreateInstance();
        ros.ConnectOnStart = false;

        scanThread = new Thread(ScanSubnet) { IsBackground = true };
        scanThread.Start();
    }

    void Update()
    {
        if (connected) return;

        string ip = foundIP;
        if (ip != null)
        {
            connected = true;
            Debug.Log($"[ROSAutoConnect] Found ROS endpoint at {ip}:{rosPort}");
            ROSConnection.GetOrCreateInstance().Connect(ip, rosPort);
            enabled = false;
        }
    }

    void ScanSubnet()
    {
        Debug.Log($"[ROSAutoConnect] Scanning 192.168.0.{RangeStart}-{RangeEnd}:{rosPort}...");

        for (int round = 0; round < 20 && foundIP == null; round++)
        {
            for (int i = RangeStart; i <= RangeEnd && foundIP == null; i++)
            {
                string ip = $"192.168.0.{i}";
                if (TryConnect(ip))
                {
                    foundIP = ip;
                    return;
                }
            }
            Thread.Sleep(500);
        }

        Debug.LogWarning("[ROSAutoConnect] No endpoint found — falling back to ROS Settings IP");
        foundIP = ROSConnection.GetOrCreateInstance().RosIPAddress;
    }

    bool TryConnect(string ip)
    {
        try
        {
            using (var tcp = new TcpClient())
            {
                var result = tcp.BeginConnect(ip, rosPort, null, null);
                bool success = result.AsyncWaitHandle.WaitOne(connectTimeoutMs);
                if (success && tcp.Connected)
                {
                    tcp.EndConnect(result);
                    return true;
                }
            }
        }
        catch { }
        return false;
    }
}
