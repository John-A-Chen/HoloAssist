#!/usr/bin/env python3
"""UDP beacon — multicasts this machine's IP so the Quest can auto-discover the ROS endpoint."""

import socket
import struct
import time
import argparse
import fcntl

MULTICAST_GROUP = "239.255.42.99"


def get_ip_for_subnet(prefix="192.168.0."):
    """Find a local IP on the given subnet by scanning interfaces."""
    import netifaces
    for iface in netifaces.interfaces():
        addrs = netifaces.ifaddresses(iface).get(netifaces.AF_INET, [])
        for a in addrs:
            if a["addr"].startswith(prefix):
                return a["addr"]
    return None


def get_local_ip(preferred_prefix="192.168.0."):
    """Return the best local IP — prefer the 192.168.0.x subnet, fall back to default route."""
    try:
        import netifaces
        ip = get_ip_for_subnet(preferred_prefix)
        if ip:
            return ip
    except ImportError:
        pass
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


def main():
    parser = argparse.ArgumentParser(description="HoloAssist ROS IP beacon")
    parser.add_argument("--ip", default=None, help="Override IP to broadcast (default: auto-detect 192.168.0.x)")
    parser.add_argument("--port", type=int, default=10000, help="ROS TCP port (default: 10000)")
    parser.add_argument("--beacon-port", type=int, default=9090, help="UDP beacon port (default: 9090)")
    args = parser.parse_args()

    ip = args.ip or get_local_ip()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    msg = f"HOLOASSIST:{ip}:{args.port}".encode()
    print(f"Beacon: multicasting {ip}:{args.port} to {MULTICAST_GROUP}:{args.beacon_port}")

    while True:
        sock.sendto(msg, (MULTICAST_GROUP, args.beacon_port))
        # subnet broadcast as fallback
        subnet_broadcast = ip.rsplit(".", 1)[0] + ".255"
        sock.sendto(msg, (subnet_broadcast, args.beacon_port))
        time.sleep(1)


if __name__ == "__main__":
    main()
