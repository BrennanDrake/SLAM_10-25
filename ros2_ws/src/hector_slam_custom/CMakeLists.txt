cmake_minimum_required(VERSION 3.8)
project(hector_slam_custom)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# Include directories
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

if(EXISTS "/opt/ros/jazzy/include")
  include_directories("/opt/ros/jazzy/include")
  add_definitions(-DROS_JAZZY_TF2_WORKAROUND)
endif()

# Create library (only with implemented files)
add_library(${PROJECT_NAME}_lib
  src/scan_matcher.cpp
  src/map_manager.cpp
  src/hector_slam_processor.cpp
  src/ekf_filter.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

# Create executable for Hector SLAM node
add_executable(${PROJECT_NAME}_node
  src/hector_slam_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)

# Create static scan publisher for testing
add_executable(static_scan_publisher
  src/static_scan_publisher.cpp
)

ament_target_dependencies(static_scan_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
)

# Create odometry simulator for testing EKF
add_executable(odom_simulator
  src/odom_simulator.cpp
)

ament_target_dependencies(odom_simulator
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# Create component library for composition (commented out until we implement HectorSlamProcessor)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install executables
install(TARGETS
  ${PROJECT_NAME}_node
  static_scan_publisher
  odom_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/monitor_slam.py
  scripts/test_slam.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Linting
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests
  ament_add_gtest(test_scan_matcher test/test_scan_matcher.cpp)
  if(TARGET test_scan_matcher)
    target_link_libraries(test_scan_matcher ${PROJECT_NAME}_lib)
    ament_target_dependencies(test_scan_matcher rclcpp sensor_msgs)
  endif()
endif()

ament_package()
