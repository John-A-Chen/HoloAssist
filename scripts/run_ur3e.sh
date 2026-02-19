#!/usr/bin/env bash
set -e

# 1) Start URSim (host Docker)
docker rm -f ursim >/dev/null 2>&1 || true
docker run -d \
  -p 6080:6080 -p 29999:29999 -p 30001:30001 -p 30002:30002 -p 30003:30003 -p 30004:30004 \
  --name ursim \
  universalrobots/ursim_e-series

echo "URSim starting... open: http://localhost:6080/vnc.html"

# 2) Start ROS driver + RViz (your ROS container)
docker exec -it rs2-ur bash -lc '
  source /opt/ros/humble/setup.bash
  source /ws/install/setup.bash
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=127.0.0.1 launch_rviz:=true
'
