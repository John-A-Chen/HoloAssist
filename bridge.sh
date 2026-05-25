#!/bin/bash
# Run the WebSocket bridge server on the laptop (sources ROS automatically).
# The Steam Deck connects to this over USB-C Ethernet.
#
# Usage:
#   ./bridge.sh                    # default port 9090
#   ./bridge.sh --port 9091        # custom port
#   ./bridge.sh --no-ros           # UI testing without ROS
source /opt/ros/humble/setup.bash
source /home/nic/git/RS2-HoloAssist/nic/ros2_ws/install/setup.bash
python3 /home/nic/git/RS2-HoloAssist/nic/dashboard/bridge_server.py "$@"
