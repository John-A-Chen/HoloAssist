#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/nic/git/RS2-HoloAssist/nic/ros2_ws/install/setup.bash
[ -f /home/nic/git/RS2-HoloAssist/main/ros2_ws/install/setup.bash ] && source /home/nic/git/RS2-HoloAssist/main/ros2_ws/install/setup.bash
python3 /home/nic/git/RS2-HoloAssist/nic/calibrate.py "$@"
