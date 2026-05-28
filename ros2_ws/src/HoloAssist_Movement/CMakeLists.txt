cmake_minimum_required(VERSION 3.8)
project(holoassist_movement)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetRPY.msg"
)

ament_python_install_package(holoassist_movement_nodes)

install(DIRECTORY
  config
  docs
  launch
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS holoassist_movement_nodes/holoassist_movement.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME holoassist_movement
)
install(PROGRAMS holoassist_movement_nodes/holoassist_movement.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME coordinate_listener
)
install(PROGRAMS holoassist_movement_nodes/workspace_scene_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME workspace_scene_manager
)
install(PROGRAMS holoassist_movement_nodes/pick_place_sequencer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_place_sequencer
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
