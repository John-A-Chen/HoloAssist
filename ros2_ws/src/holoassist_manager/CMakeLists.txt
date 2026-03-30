cmake_minimum_required(VERSION 3.8)
project(holoassist_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── Manager node executable ───────────────────────────────────────────────────
add_executable(manager_node src/manager_node.cpp)
target_include_directories(manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(manager_node PUBLIC cxx_std_17)
ament_target_dependencies(manager_node
  rclcpp std_msgs std_srvs diagnostic_msgs)

# ── TF Marker Bridge node ────────────────────────────────────────────────────
add_executable(tf_marker_bridge src/tf_marker_bridge.cpp)
target_compile_features(tf_marker_bridge PUBLIC cxx_std_17)
ament_target_dependencies(tf_marker_bridge
  rclcpp tf2_ros visualization_msgs geometry_msgs)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS manager_node tf_marker_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
