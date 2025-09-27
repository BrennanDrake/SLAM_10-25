/**
 * @file occupancy_grid_generator.hpp
 * @brief Occupancy grid generator for SLAM applications
 * @author Brennan Drake
 */

#ifndef OCCUPANCY_GRID_GENERATOR__OCCUPANCY_GRID_GENERATOR_HPP_
#define OCCUPANCY_GRID_GENERATOR__OCCUPANCY_GRID_GENERATOR_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/tf2_ros/transform_listener.hpp"
#include "tf2_ros/tf2_ros/buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace occupancy_grid_generator
{

/**
 * @brief Node that generates occupancy grids from LiDAR scan data
 * 
 * Subscribes to /scan topic and publishes occupancy grid maps to /map topic.
 * Supports configurable grid resolution, size, and update parameters.
 */
class OccupancyGridGenerator : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for OccupancyGridGenerator
   * @param options Node options for configuration
   */
  explicit OccupancyGridGenerator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
   * @param robot_pose Current robot pose in map frame
   */
  void update_grid(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                   const geometry_msgs::msg::Pose & robot_pose);

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
   * @brief Get current robot pose in map frame
   * @param pose Output pose
   * @return true if transform was successful
   */
  bool get_robot_pose(geometry_msgs::msg::Pose & pose) const;

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
  
  // TF2 for coordinate transformations
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Occupancy grid data
  nav_msgs::msg::OccupancyGrid occupancy_grid_;
  
  // Configuration parameters
  double resolution_;           // meters per pixel
  int grid_width_;             // grid width in cells
  int grid_height_;            // grid height in cells
  double origin_x_;            // grid origin x in world coordinates
  double origin_y_;            // grid origin y in world coordinates
  std::string map_frame_;      // frame for the map
  std::string base_frame_;     // robot base frame
  
  // Occupancy grid update parameters
  double prob_hit_;            // probability of cell being occupied when hit
  double prob_miss_;           // probability of cell being free when missed
  double prob_prior_;          // prior probability of occupancy
  int8_t occupied_threshold_;  // threshold for marking cell as occupied
  int8_t free_threshold_;      // threshold for marking cell as free
  
  // Internal state
  bool grid_initialized_;
  std::vector<double> log_odds_; // log odds representation for updates
};

}  // namespace occupancy_grid_generator

#endif  // OCCUPANCY_GRID_GENERATOR__OCCUPANCY_GRID_GENERATOR_HPP_
