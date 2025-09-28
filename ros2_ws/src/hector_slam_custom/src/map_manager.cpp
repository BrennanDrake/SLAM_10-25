/**
 * @file map_manager_simple.cpp
 * @brief Simplified multi-resolution occupancy grid manager for Hector SLAM
 * 
 * Minimal implementation to get Phase 2.2 working
 */

#include "hector_slam_custom/map_manager.hpp"
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace hector_slam_custom {

// OccupancyGridLevel - Minimal Implementation
OccupancyGridLevel::OccupancyGridLevel(double resolution, int size_x, int size_y,
                                       double origin_x, double origin_y)
    : resolution_(resolution), size_x_(size_x), size_y_(size_y), 
      origin_x_(origin_x), origin_y_(origin_y) {
    
    // Initialize log-odds grid
    int total_cells = size_x_ * size_y_;
    log_odds_grid_.resize(total_cells, 0.0);
    
    // Set occupancy update parameters (from Phase 1)
    prob_hit_ = 0.7;
    prob_miss_ = 0.4;
    prob_prior_ = 0.5;
    log_odds_hit_ = std::log(prob_hit_ / (1.0 - prob_hit_));
    log_odds_miss_ = std::log(prob_miss_ / (1.0 - prob_miss_));
}

void OccupancyGridLevel::updateWithScan(const sensor_msgs::msg::LaserScan& scan,
                                       const geometry_msgs::msg::Pose& robot_pose) {
    // Extract yaw from quaternion
    double yaw = tf2::getYaw(robot_pose.orientation);
    
    // Convert robot pose to grid coordinates
    int robot_grid_x, robot_grid_y;
    if (!worldToGrid(robot_pose.position.x, robot_pose.position.y, 
                    robot_grid_x, robot_grid_y)) {
        return;  // Robot outside grid bounds
    }
    
    // Process each laser ray
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float range = scan.ranges[i];
        
        // Skip invalid measurements
        if (std::isnan(range) || std::isinf(range) ||
            range < scan.range_min || range > scan.range_max) {
            continue;
        }
        
        // Calculate ray angle in world frame
        double angle = yaw + scan.angle_min + i * scan.angle_increment;
        
        // Calculate endpoint in world coordinates
        double end_x = robot_pose.position.x + range * std::cos(angle);
        double end_y = robot_pose.position.y + range * std::sin(angle);
        
        // Convert endpoint to grid coordinates
        int end_grid_x, end_grid_y;
        if (!worldToGrid(end_x, end_y, end_grid_x, end_grid_y)) {
            continue;  // Endpoint outside grid
        }
        
        // Trace ray using Bresenham's algorithm
        bool hit_max_range = (range >= scan.range_max * 0.99);
        traceRay(robot_grid_x, robot_grid_y, end_grid_x, end_grid_y, !hit_max_range);
    }
}

nav_msgs::msg::OccupancyGrid OccupancyGridLevel::getOccupancyGridMsg(const std::string& frame_id) const {
    nav_msgs::msg::OccupancyGrid msg;
    
    // Set header
    msg.header.frame_id = frame_id;
    msg.header.stamp = rclcpp::Clock().now();
    
    // Set metadata
    msg.info.resolution = resolution_;
    msg.info.width = size_x_;
    msg.info.height = size_y_;
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;  // No rotation
    
    // Convert log-odds to occupancy grid values
    msg.data.resize(size_x_ * size_y_);
    for (int i = 0; i < size_x_ * size_y_; ++i) {
        double probability = logOddsToProbability(log_odds_grid_[i]);
        msg.data[i] = probabilityToOccupancyValue(probability);
    }
    
    return msg;
}

double OccupancyGridLevel::getLogOdds(int x, int y) const {
    if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
        return 0.0;
    }
    return log_odds_grid_[y * size_x_ + x];
}

double OccupancyGridLevel::getOccupancyProbability(double world_x, double world_y) const {
    int grid_x, grid_y;
    if (!worldToGrid(world_x, world_y, grid_x, grid_y)) {
        return 0.5;
    }
    
    int index = grid_y * size_x_ + grid_x;
    return logOddsToProbability(log_odds_grid_[index]);
}

// isWithinBounds is now handled by worldToGrid returning false for out-of-bounds

bool OccupancyGridLevel::worldToGrid(double world_x, double world_y, 
                                    int& grid_x, int& grid_y) const {
    grid_x = static_cast<int>((world_x - origin_x_) / resolution_);
    grid_y = static_cast<int>((world_y - origin_y_) / resolution_);
    
    return (grid_x >= 0 && grid_x < size_x_ && grid_y >= 0 && grid_y < size_y_);
}

void OccupancyGridLevel::gridToWorld(int grid_x, int grid_y,
                                    double& world_x, double& world_y) const {
    world_x = origin_x_ + (grid_x + 0.5) * resolution_;
    world_y = origin_y_ + (grid_y + 0.5) * resolution_;
}

// Private methods
void OccupancyGridLevel::updateCell(int grid_x, int grid_y, double log_odds_update) {
    if (grid_x < 0 || grid_x >= size_x_ || grid_y < 0 || grid_y >= size_y_) {
        return;
    }
    
    int index = grid_y * size_x_ + grid_x;
    log_odds_grid_[index] += log_odds_update;
    
    // Clamp log-odds
    double max_log_odds = 5.0;
    double min_log_odds = -5.0;
    log_odds_grid_[index] = std::clamp(log_odds_grid_[index], min_log_odds, max_log_odds);
}

void OccupancyGridLevel::traceRay(int start_x, int start_y, int end_x, int end_y, bool max_range_hit) {
    // Bresenham's line algorithm
    int dx = std::abs(end_x - start_x);
    int dy = std::abs(end_y - start_y);
    int sx = (start_x < end_x) ? 1 : -1;
    int sy = (start_y < end_y) ? 1 : -1;
    int err = dx - dy;
    
    int x = start_x;
    int y = start_y;
    
    while (true) {
        if (x == end_x && y == end_y) {
            // Hit point
            if (max_range_hit) {
                updateCell(x, y, log_odds_hit_);
            }
            break;
        } else {
            // Free space along ray
            updateCell(x, y, log_odds_miss_);
        }
        
        // Bresenham step
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
        
        // Safety check
        if (std::abs(x - start_x) > 1000 || std::abs(y - start_y) > 1000) {
            break;
        }
    }
}

double OccupancyGridLevel::logOddsToProbability(double log_odds) const {
    return 1.0 / (1.0 + std::exp(-log_odds));
}

int8_t OccupancyGridLevel::probabilityToOccupancyValue(double probability) const {
    if (probability < 0.25) {
        return 0;  // Free
    } else if (probability > 0.65) {
        return 100;  // Occupied
    } else {
        return -1;  // Unknown
    }
}

// MapManager Implementation
MapManager::MapManager(double base_resolution, double map_size, int num_levels)
    : base_resolution_(base_resolution), map_size_(map_size), 
      num_levels_(num_levels), initialized_(false) {
    
    // Create multi-resolution grid levels
    for (int i = 0; i < num_levels_; ++i) {
        double level_resolution = base_resolution_ * std::pow(2.0, i);
        int grid_size = static_cast<int>(map_size_ / level_resolution);
        double origin = -map_size_ / 2.0;
        
        grid_levels_.emplace_back(
            std::make_unique<OccupancyGridLevel>(
                level_resolution, grid_size, grid_size, origin, origin
            )
        );
        
        RCLCPP_INFO(rclcpp::get_logger("MapManager"),
                   "Created grid level %d: resolution=%.3fm, size=%dx%d",
                   i, level_resolution, grid_size, grid_size);
    }
    
    initialized_ = true;
}

void MapManager::updateMaps(const sensor_msgs::msg::LaserScan& scan,
                           const geometry_msgs::msg::Pose& robot_pose) {
    for (auto& grid_level : grid_levels_) {
        grid_level->updateWithScan(scan, robot_pose);
    }
}

nav_msgs::msg::OccupancyGrid MapManager::getOccupancyGrid(int level, 
                                                         const std::string& frame_id) const {
    if (level < 0 || level >= static_cast<int>(grid_levels_.size())) {
        throw std::out_of_range("Invalid grid level requested");
    }
    
    return grid_levels_[level]->getOccupancyGridMsg(frame_id);
}

nav_msgs::msg::OccupancyGrid MapManager::getFinestOccupancyGrid(const std::string& frame_id) const {
    return getOccupancyGrid(0, frame_id);
}

const OccupancyGridLevel* MapManager::getGridLevel(int level) const {
    if (level < 0 || level >= static_cast<int>(grid_levels_.size())) {
        return nullptr;
    }
    return grid_levels_[level].get();
}

void MapManager::reset() {
    // Recreate all grid levels to reset them
    grid_levels_.clear();
    
    for (int i = 0; i < num_levels_; ++i) {
        double level_resolution = base_resolution_ * std::pow(2.0, i);
        int grid_size = static_cast<int>(map_size_ / level_resolution);
        double origin = -map_size_ / 2.0;
        
        grid_levels_.emplace_back(
            std::make_unique<OccupancyGridLevel>(
                level_resolution, grid_size, grid_size, origin, origin
            )
        );
    }
}

} // namespace hector_slam_custom
