#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/ros2_ws/install/setup.bash"
python3 "${SCRIPT_DIR}/launch.py" "$@"
