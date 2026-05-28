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

    void Awake()
    {
        var ros = ROSConnection.GetOrCreateInstance();
        ros.ConnectOnStart = false;

        scanThread = new Thread(ScanForEndpoint) { IsBackground = true };
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

    void ScanForEndpoint()
    {
        Debug.Log($"[ROSAutoConnect] Scanning for ROS endpoint on port {rosPort}...");

        for (int round = 0; round < 30 && foundIP == null; round++)
        {
            // Try localhost first (Unity Editor on same machine)
            if (TryConnect("127.0.0.1")) { foundIP = "127.0.0.1"; return; }

            // Scan robot subnet 192.168.0.101-110 (skip .100 = robot Ethernet)
            for (int i = 101; i <= 110 && foundIP == null; i++)
            {
                string ip = $"192.168.0.{i}";
                if (TryConnect(ip)) { foundIP = ip; return; }
            }

            // Scan local machine IPs (catches any subnet)
            try
            {
                var host = Dns.GetHostEntry(Dns.GetHostName());
                foreach (var addr in host.AddressList)
                {
                    if (addr.AddressFamily != AddressFamily.InterNetwork) continue;
                    string ip = addr.ToString();
                    if (ip == "127.0.0.1") continue;
                    if (TryConnect(ip)) { foundIP = ip; return; }
                }
            }
            catch { }

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
