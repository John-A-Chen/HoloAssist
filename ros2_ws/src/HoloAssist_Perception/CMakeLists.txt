cmake_minimum_required(VERSION 3.8)
project(holoassist_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CubePerceptionStatus.msg"
  "srv/PickCubeToBin.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_python_install_package(holoassist_perception_nodes)

install(DIRECTORY
  config
  launch
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS holoassist_perception_nodes/aprilcube_tracker_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aprilcube_tracker_node
)
install(PROGRAMS holoassist_perception_nodes/webcam_image_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME holoassist_webcam_image_publisher
)
install(PROGRAMS holoassist_perception_nodes/trolley_scene_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME holoassist_trolley_scene_publisher
)
install(PROGRAMS holoassist_perception_nodes/pick_place_service_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_place_service_node
)
install(PROGRAMS holoassist_perception_nodes/selected_cube_to_moveit_target_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME selected_cube_to_moveit_target_node
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
