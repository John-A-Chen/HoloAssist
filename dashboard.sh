#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/ros2_ws/install/setup.bash"

if [[ "$*" == *"--bridge"* ]]; then
    # Network client mode (Steam Deck) — no bridge server needed
    python3 "$SCRIPT_DIR/dashboard/main.py" "$@"
else
    # Normal mode — start bridge server in background so Nic can connect
    python3 "$SCRIPT_DIR/dashboard/bridge_server.py" &
    BRIDGE_PID=$!
    trap "kill $BRIDGE_PID 2>/dev/null" EXIT
    python3 "$SCRIPT_DIR/dashboard/main.py" "$@"
fi
