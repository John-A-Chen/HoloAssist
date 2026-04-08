#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/nic/git/RS2-HoloAssist/nic/ros2_ws/install/setup.bash
python3 /home/nic/git/RS2-HoloAssist/nic/launch.py "$@"
