#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
exec python3 "$SCRIPT_DIR/launch.py" "$@"
