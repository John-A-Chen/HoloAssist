cmake_minimum_required(VERSION 3.8)
project(holo_assist_depth_tracker_sim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CubePerceptionStatus.msg"
  "srv/SetAprilCubePose.srv"
  "srv/SetCameraPose.srv"
  "srv/PickCubeToBin.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
