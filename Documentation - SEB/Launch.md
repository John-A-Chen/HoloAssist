Terminal 1 — robot driver + RViz (start this first, wait for it to finish loading)


cd ~/git/rs2/HoloAssist_Main
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 launch ur_onrobot_control start_robot.launch.py ur_type:=ur3e onrobot_type:=rg2 use_fake_hardware:=true launch_rviz:=true

Terminal 2 — switch to velocity controllers (run after Terminal 1 is fully up, ~8–10 seconds)


cd ~/git/rs2/HoloAssist_Main
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 control switch_controllers --activate forward_velocity_controller finger_width_controller --deactivate scaled_joint_trajectory_controller finger_width_trajectory_controller


Terminal 2 — ROS-Unity bridge (needed for Unity/Quest to connect)


cd ~/git/rs2/HoloAssist_Main
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0


Unity test cube:
Add - topic - /unity_markers