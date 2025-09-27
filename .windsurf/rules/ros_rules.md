---
trigger: always_on
---

# ============================================
# ROS 2 DEVELOPMENT RULES (template-friendly)
# Assumes a ros2_ws/ directory exists inside
# the project folder when you duplicate it.
# ============================================
rules:

  # ===========================================
  # ROS 2: Python (rclpy nodes and launch files)
  # ===========================================
  python:
    files: [
      "ros2_ws/src/**/*.py",   # project-local ROS 2 Python nodes
      "src/**/*.py",          # optional: other src Python under project
      "ros2_ws/**/*.launch.py",
      "**/*.launch.py"        # allow launch files anywhere in project
    ]
    rules:
      # Ensure ROS 2 lifecycle calls are present in main-entry scripts
      - match: "(?m)^if __name__ == ['\\"]__main__['\\"]:"
        append: |
          # TODO: Ensure rclpy.init() is called before creating nodes and rclpy.shutdown() is called on exit.
        explanation: "Ensures proper ROS 2 lifecycle."

      # Encourage ROS-native logging over print()
      - match: "(?m)^import rclpy"
        append: |
          # TODO: Prefer node.get_logger().info()/warn()/error() over print().
        explanation: "Encourages ROS-native logging."

      # Ensure executable scripts have a Python 3 shebang
      - match: "(?m)^(?!#\\!/usr/bin/env python3).*$"
        prepend: "#!/usr/bin/env python3\n"
        explanation: "Ensure executable scripts start with a Python 3 shebang."

      # Launch file consistency reminders
      - match: "(?m)^from launch import"
        append: |
          # TODO: Ensure node names, namespaces, remappings, and parameters follow project conventions.
          # TODO: Prefer composable nodes where appropriate to reduce process overhead.
        explanation: "Promotes consistent launch configuration."

  # ===========================================
  # ROS 2: C++ Nodes (rclcpp)
  # ===========================================
  cpp:
    files: [
      "ros2_ws/src/**/*.cpp",
      "ros2_ws/**/*.hpp"
    ]
    rules:
      # Encourage RCLCPP logging macros
      - match: "(?m)#include <rclcpp/rclcpp.hpp>"
        append: |
          // TODO: Use RCLCPP_INFO/WARN/ERROR for logging instead of std::cout.
        explanation: "Encourages ROS-native logging."

      # Ensure init/shutdown in main()
      - match: "(?m)^\s*int\s+main\s*\("
        append: |
          // TODO: Ensure rclcpp::init(argc, argv); and rclcpp::shutdown(); are present.
        explanation: "Ensures proper ROS 2 lifecycle."

  # ===========================================
  # ROS 2: CMakeLists.txt (ament dependencies)
  # ===========================================
  cmake:
    files: [
      "ros2_ws/**/CMakeLists.txt"
    ]
    rules:
      - match: "(?m)^\s*ament_target_dependencies\("
        append: |
          # TODO: Verify all required dependencies are listed and match package.xml.
        explanation: "Keep CMake and package.xml in sync."

  # ===========================================
  # ROS 2: package.xml (metadata hygiene)
  # ===========================================
  xml:
    files: [
      "ros2_ws/**/package.xml",
      "**/package.xml"
    ]
    rules:
      - match: "(?s)<package.*?>"
        append: |
          <!-- TODO: Ensure <license>, <maintainer>, <author>, and <url type=\"repository\"> are present and correct. -->
        explanation: "Promotes proper package metadata."

  # ===========================================
  # ROS 2: Parameters (YAML)
  # ===========================================
  yaml:
    files: [
      "ros2_ws/**/*.yaml",
      "**/*.yaml"
    ]
    rules:
      - match: "(?m)^[A-Za-z_].*:"
        append: |
          # TODO: Document parameter purpose/defaults where unclear. Keep types consistent across nodes.
        explanation: "Encourages parameter clarity."
