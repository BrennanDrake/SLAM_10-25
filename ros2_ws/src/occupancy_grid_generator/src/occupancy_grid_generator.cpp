/**
 * @file occupancy_grid_generator.cpp
 * @brief Implementation of occupancy grid generator for SLAM applications
 * @author Brennan Drake
 */

#include "occupancy_grid_generator/occupancy_grid_generator.hpp"

#include <cmath>
#include <algorithm>
#include "tf2/exceptions.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace occupancy_grid_generator
{

OccupancyGridGenerator::OccupancyGridGenerator(const rclcpp::NodeOptions & options)
: Node("occupancy_grid_generator", options),
  grid_initialized_(false)
{
  // Declare and get parameters
  this->declare_parameter("resolution", 0.05);  // 5cm per pixel
  this->declare_parameter("grid_width", 2000);   // 100m at 5cm resolution
  this->declare_parameter("grid_height", 2000);  // 100m at 5cm resolution
  this->declare_parameter("origin_x", -50.0);    // center the grid
  this->declare_parameter("origin_y", -50.0);    // center the grid
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("prob_hit", 0.7);
  this->declare_parameter("prob_miss", 0.4);
  this->declare_parameter("prob_prior", 0.5);
  this->declare_parameter("occupied_threshold", 65);  // 0-100 scale
  this->declare_parameter("free_threshold", 25);      // 0-100 scale

  resolution_ = this->get_parameter("resolution").as_double();
  grid_width_ = this->get_parameter("grid_width").as_int();
  grid_height_ = this->get_parameter("grid_height").as_int();
  origin_x_ = this->get_parameter("origin_x").as_double();
  origin_y_ = this->get_parameter("origin_y").as_double();
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  prob_hit_ = this->get_parameter("prob_hit").as_double();
  prob_miss_ = this->get_parameter("prob_miss").as_double();
  prob_prior_ = this->get_parameter("prob_prior").as_double();
  occupied_threshold_ = this->get_parameter("occupied_threshold").as_int();
  free_threshold_ = this->get_parameter("free_threshold").as_int();

  // Initialize TF2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create publishers and subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&OccupancyGridGenerator::scan_callback, this, std::placeholders::_1));
  
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);

  // Initialize the occupancy grid
  initialize_grid();

  RCLCPP_INFO(this->get_logger(), "OccupancyGridGenerator initialized");
  RCLCPP_INFO(this->get_logger(), "Grid size: %dx%d, Resolution: %.3fm", 
              grid_width_, grid_height_, resolution_);
}

void OccupancyGridGenerator::initialize_grid()
{
  occupancy_grid_.header.frame_id = map_frame_;
  occupancy_grid_.info.resolution = resolution_;
  occupancy_grid_.info.width = grid_width_;
  occupancy_grid_.info.height = grid_height_;
  occupancy_grid_.info.origin.position.x = origin_x_;
  occupancy_grid_.info.origin.position.y = origin_y_;
  occupancy_grid_.info.origin.position.z = 0.0;
  occupancy_grid_.info.origin.orientation.w = 1.0;

  // Initialize grid data with unknown values (-1)
  occupancy_grid_.data.resize(grid_width_ * grid_height_, -1);
  
  // Initialize log odds array
  log_odds_.resize(grid_width_ * grid_height_, std::log(prob_prior_ / (1.0 - prob_prior_)));
  
  grid_initialized_ = true;
  
  RCLCPP_INFO(this->get_logger(), "Occupancy grid initialized");
}

void OccupancyGridGenerator::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!grid_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Grid not initialized, skipping scan");
    return;
  }

  // Get robot pose in map frame
  geometry_msgs::msg::Pose robot_pose;
  if (!get_robot_pose(robot_pose)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Could not get robot pose, skipping scan update");
    return;
  }

  // Update grid with scan data
  update_grid(msg, robot_pose);

  // Publish updated map
  occupancy_grid_.header.stamp = msg->header.stamp;
  map_pub_->publish(occupancy_grid_);
}

void OccupancyGridGenerator::update_grid(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                                        const geometry_msgs::msg::Pose & robot_pose)
{
  // Convert robot position to grid coordinates
  int robot_grid_x, robot_grid_y;
  if (!world_to_grid(robot_pose.position.x, robot_pose.position.y, robot_grid_x, robot_grid_y)) {
    RCLCPP_WARN(this->get_logger(), "Robot position outside grid bounds");
    return;
  }

  // Get robot orientation
  double robot_yaw = 2.0 * std::atan2(robot_pose.orientation.z, robot_pose.orientation.w);

  // Process each laser ray
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    
    // Skip invalid readings
    if (range < scan->range_min || range > scan->range_max || std::isnan(range) || std::isinf(range)) {
      continue;
    }

    // Calculate ray angle
    double ray_angle = robot_yaw + scan->angle_min + i * scan->angle_increment;
    
    // Calculate end point of ray
    double end_x = robot_pose.position.x + range * std::cos(ray_angle);
    double end_y = robot_pose.position.y + range * std::sin(ray_angle);
    
    int end_grid_x, end_grid_y;
    if (!world_to_grid(end_x, end_y, end_grid_x, end_grid_y)) {
      // End point outside grid, trace to grid boundary
      double max_range = std::min(range, scan->range_max * 0.95);
      end_x = robot_pose.position.x + max_range * std::cos(ray_angle);
      end_y = robot_pose.position.y + max_range * std::sin(ray_angle);
      if (!world_to_grid(end_x, end_y, end_grid_x, end_grid_y)) {
        continue; // Still outside, skip this ray
      }
    }

    // Trace ray and mark free cells
    std::vector<std::pair<int, int>> free_cells;
    bresenham_line(robot_grid_x, robot_grid_y, end_grid_x, end_grid_y, free_cells);
    
    // Update free cells (all cells along ray except the last one)
    for (size_t j = 0; j < free_cells.size() - 1; ++j) {
      int idx = free_cells[j].second * grid_width_ + free_cells[j].first;
      if (idx >= 0 && idx < static_cast<int>(log_odds_.size())) {
        log_odds_[idx] += std::log(prob_miss_ / (1.0 - prob_miss_));
        
        // Convert log odds to probability and then to occupancy grid value
        double prob = 1.0 / (1.0 + std::exp(-log_odds_[idx]));
        if (prob < free_threshold_ / 100.0) {
          occupancy_grid_.data[idx] = 0;  // Free
        } else if (prob > occupied_threshold_ / 100.0) {
          occupancy_grid_.data[idx] = 100;  // Occupied
        } else {
          occupancy_grid_.data[idx] = -1;  // Unknown
        }
      }
    }

    // Mark end cell as occupied (if within range limits)
    if (range < scan->range_max * 0.95) {  // Don't mark max range readings as obstacles
      int end_idx = end_grid_y * grid_width_ + end_grid_x;
      if (end_idx >= 0 && end_idx < static_cast<int>(log_odds_.size())) {
        log_odds_[end_idx] += std::log(prob_hit_ / (1.0 - prob_hit_));
        
        double prob = 1.0 / (1.0 + std::exp(-log_odds_[end_idx]));
        if (prob < free_threshold_ / 100.0) {
          occupancy_grid_.data[end_idx] = 0;  // Free
        } else if (prob > occupied_threshold_ / 100.0) {
          occupancy_grid_.data[end_idx] = 100;  // Occupied
        } else {
          occupancy_grid_.data[end_idx] = -1;  // Unknown
        }
      }
    }
  }
}

bool OccupancyGridGenerator::world_to_grid(double x, double y, int & grid_x, int & grid_y) const
{
  grid_x = static_cast<int>((x - origin_x_) / resolution_);
  grid_y = static_cast<int>((y - origin_y_) / resolution_);
  
  return (grid_x >= 0 && grid_x < grid_width_ && grid_y >= 0 && grid_y < grid_height_);
}

bool OccupancyGridGenerator::get_robot_pose(geometry_msgs::msg::Pose & pose) const
{
  try {
    auto transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(this->get_logger(), "Could not transform %s to %s: %s",
                base_frame_.c_str(), map_frame_.c_str(), ex.what());
    return false;
  }
}

void OccupancyGridGenerator::bresenham_line(int x0, int y0, int x1, int y1, 
                                           std::vector<std::pair<int, int>> & free_cells)
{
  free_cells.clear();
  
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int x = x0;
  int y = y0;

  while (true) {
    // Check bounds before adding
    if (x >= 0 && x < grid_width_ && y >= 0 && y < grid_height_) {
      free_cells.emplace_back(x, y);
    }

    if (x == x1 && y == y1) break;

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
}

}  // namespace occupancy_grid_generator

RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_grid_generator::OccupancyGridGenerator)
