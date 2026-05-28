#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/ros2_ws/install/setup.bash"

if [[ "$*" == *"--bridge"* ]]; then
    # Network client mode (Steam Deck) — no bridge server needed
    exec python3 "$REPO_ROOT/dashboard/main.py" "$@"
else
    # Normal mode — start bridge server in background so Nic can connect
    python3 "$REPO_ROOT/dashboard/bridge_server.py" &
    BRIDGE_PID=$!
    trap "kill $BRIDGE_PID 2>/dev/null" EXIT
    python3 "$REPO_ROOT/dashboard/main.py" "$@"
fi
