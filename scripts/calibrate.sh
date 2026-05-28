#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/ros2_ws/install/setup.bash"

exec python3 "$SCRIPT_DIR/calibrate.py" "$@"
