#!/bin/bash
# Run the WebSocket bridge server on the laptop (sources ROS automatically).
# The Steam Deck connects to this over USB-C Ethernet or WiFi.
#
# Usage:
#   ./bridge.sh                    # default port 9090
#   ./bridge.sh --port 9091        # custom port
#   ./bridge.sh --no-ros           # UI testing without ROS
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
fuser -k 9090/tcp 2>/dev/null; sleep 0.3
python3 "$SCRIPT_DIR/dashboard/bridge_server.py" "$@"
