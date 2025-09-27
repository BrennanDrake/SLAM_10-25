/**
 * @file occupancy_grid_generator_simple.hpp
 * @brief Simplified occupancy grid generator without TF2 dependencies for initial testing
 * @author Brennan Drake
 */

#ifndef OCCUPANCY_GRID_GENERATOR__OCCUPANCY_GRID_GENERATOR_SIMPLE_HPP_
#define OCCUPANCY_GRID_GENERATOR__OCCUPANCY_GRID_GENERATOR_SIMPLE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace occupancy_grid_generator
{

/**
 * @brief Simplified node that generates occupancy grids from LiDAR scan data
 * 
 * This version assumes the robot is at the origin and doesn't use TF2.
 * Subscribes to /scan topic and publishes occupancy grid maps to /map topic.
 */
class OccupancyGridGeneratorSimple : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for OccupancyGridGeneratorSimple
   * @param options Node options for configuration
   */
  explicit OccupancyGridGeneratorSimple(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Callback for processing incoming laser scan messages
   * @param msg Incoming LaserScan message
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief Initialize the occupancy grid with default values
   */
  void initialize_grid();

  /**
   * @brief Update occupancy grid with new scan data
   * @param scan LaserScan data to integrate
   */
  void update_grid(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  /**
   * @brief Convert world coordinates to grid indices
   * @param x World x coordinate
   * @param y World y coordinate
   * @param grid_x Output grid x index
   * @param grid_y Output grid y index
   * @return true if coordinates are within grid bounds
   */
  bool world_to_grid(double x, double y, int & grid_x, int & grid_y) const;

  /**
   * @brief Apply Bresenham's line algorithm for ray tracing
   * @param x0 Start x coordinate
   * @param y0 Start y coordinate  
   * @param x1 End x coordinate
   * @param y1 End y coordinate
   * @param free_cells Output vector of free cell indices
   */
  void bresenham_line(int x0, int y0, int x1, int y1, std::vector<std::pair<int, int>> & free_cells);

  // ROS 2 communication
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  // Occupancy grid data
  nav_msgs::msg::OccupancyGrid occupancy_grid_;
  
  // Configuration parameters
  double resolution_;           // meters per pixel
  int grid_width_;             // grid width in cells
  int grid_height_;            // grid height in cells
  double origin_x_;            // grid origin x in world coordinates
  double origin_y_;            // grid origin y in world coordinates
  std::string map_frame_;      // frame for the map
  
  // Occupancy grid update parameters
  double prob_hit_;            // probability of cell being occupied when hit
  double prob_miss_;           // probability of cell being free when missed
  double prob_prior_;          // prior probability of occupancy
  int8_t occupied_threshold_;  // threshold for marking cell as occupied
  int8_t free_threshold_;      // threshold for marking cell as free
  
  // Internal state
  bool grid_initialized_;
  std::vector<double> log_odds_; // log odds representation for updates
  
  // Robot pose (simplified - assumes robot at origin)
  double robot_x_;
  double robot_y_;
  double robot_theta_;
};

}  // namespace occupancy_grid_generator

#endif  // OCCUPANCY_GRID_GENERATOR__OCCUPANCY_GRID_GENERATOR_SIMPLE_HPP_
