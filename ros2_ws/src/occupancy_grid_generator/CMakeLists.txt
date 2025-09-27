cmake_minimum_required(VERSION 3.8)
project(occupancy_grid_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Print TF2 include directories for debugging
message(STATUS "tf2_ros_INCLUDE_DIRS: ${tf2_ros_INCLUDE_DIRS}")
message(STATUS "tf2_INCLUDE_DIRS: ${tf2_INCLUDE_DIRS}")
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Create the occupancy grid generator node
add_executable(occupancy_grid_generator_node
  src/occupancy_grid_generator_node.cpp
  src/occupancy_grid_generator.cpp
)

# Create the fake scan publisher for testing
add_executable(fake_scan_publisher
  src/fake_scan_publisher.cpp
)

# Create the simplified occupancy grid generator (without TF2)
add_executable(occupancy_grid_generator_simple_node
  src/occupancy_grid_generator_simple_node.cpp
  src/occupancy_grid_generator_simple.cpp
)

target_include_directories(occupancy_grid_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/ros/jazzy/include
)

ament_target_dependencies(occupancy_grid_generator_node
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  builtin_interfaces
)

target_include_directories(fake_scan_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/ros/jazzy/include
)

ament_target_dependencies(fake_scan_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)

target_include_directories(occupancy_grid_generator_simple_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(occupancy_grid_generator_simple_node
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  builtin_interfaces
)

# Create the composable component
add_library(occupancy_grid_generator_component SHARED
  src/occupancy_grid_generator.cpp
)

target_include_directories(occupancy_grid_generator_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/ros/jazzy/include
)

ament_target_dependencies(occupancy_grid_generator_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  builtin_interfaces
)

rclcpp_components_register_nodes(occupancy_grid_generator_component "occupancy_grid_generator::OccupancyGridGenerator")

# Install targets
install(TARGETS occupancy_grid_generator_node fake_scan_publisher occupancy_grid_generator_simple_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS occupancy_grid_generator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_occupancy_grid_generator.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    /opt/ros/jazzy/include
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )
endif()

ament_package()
