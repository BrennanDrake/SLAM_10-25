/**
 * @file occupancy_grid_generator_node.cpp
 * @brief Main entry point for occupancy grid generator node
 * @author Brennan Drake
 */

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "occupancy_grid_generator/occupancy_grid_generator.hpp"

/**
 * @brief Main entry point for the occupancy grid generator node
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<occupancy_grid_generator::OccupancyGridGenerator>();
  
  RCLCPP_INFO(node->get_logger(), "Starting occupancy grid generator node");
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in occupancy grid generator: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
