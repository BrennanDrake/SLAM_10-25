/**
 * @file map_manager.cpp
 * @brief Multi-resolution occupancy grid manager for Hector SLAM
 * 
 * This implementation manages multiple resolution levels of occupancy grids,
 * building on Phase 1's probabilistic mapping with log-odds representation.
 */

#include "hector_slam_custom/map_manager.hpp"
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include "tf2/utils.h"

namespace hector_slam_custom {

// OccupancyGridLevel Implementation
OccupancyGridLevel::OccupancyGridLevel(double resolution, int size_x, int size_y,
                                       double origin_x, double origin_y)
    : resolution_(resolution), size_x_(size_x), size_y_(size_y), 
      origin_x_(origin_x), origin_y_(origin_y) {
    
    // Initialize log-odds grid
    int total_cells = size_x_ * size_y_;
    log_odds_grid_.resize(total_cells, 0.0);  // prob_prior = 0.5 → log_odds = 0
    
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
        
        // Trace ray using Bresenham's algorithm (from Phase 1)
        traceLine(robot_grid_x, robot_grid_y, end_grid_x, end_grid_y);
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
    
    // Copy grid data
    msg.data = grid_data_;
    
    return msg;
}

double OccupancyGridLevel::getLogOdds(int x, int y) const {
    if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
        return 0.0;  // Unknown = log-odds of 0
    }
    return log_odds_grid_[y * size_x_ + x];
}

double OccupancyGridLevel::getOccupancyProbability(double world_x, double world_y) const {
    int grid_x, grid_y;
    if (!worldToGrid(world_x, world_y, grid_x, grid_y)) {
        return 0.5;  // Unknown probability
    }
    
    int index = grid_y * size_x_ + grid_x;
    return logOddsToProbability(log_odds_grid_[index]);
}

bool OccupancyGridLevel::isWithinBounds(double world_x, double world_y) const {
    int grid_x, grid_y;
    return worldToGrid(world_x, world_y, grid_x, grid_y);
}

void OccupancyGridLevel::reset() {
    std::fill(grid_data_.begin(), grid_data_.end(), -1);
    std::fill(log_odds_.begin(), log_odds_.end(), 0.0);
}

double OccupancyGridLevel::getResolution() const {
    return resolution_;
}

int OccupancyGridLevel::getSizeX() const {
    return size_x_;
}

int OccupancyGridLevel::getSizeY() const {
    return size_y_;
}

// Private helper methods
bool OccupancyGridLevel::worldToGrid(double world_x, double world_y, 
                                    int& grid_x, int& grid_y) const {
    grid_x = static_cast<int>((world_x - origin_x_) / resolution_);
    grid_y = static_cast<int>((world_y - origin_y_) / resolution_);
    
    return isValidCell(grid_x, grid_y);
}

bool OccupancyGridLevel::isValidCell(int x, int y) const {
    return x >= 0 && x < size_x_ && y >= 0 && y < size_y_;
}

void OccupancyGridLevel::updateCell(int x, int y, bool hit) {
    if (!isValidCell(x, y)) {
        return;
    }
    
    int index = y * size_x_ + x;
    
    // Update log-odds (Phase 1 algorithm)
    if (hit) {
        log_odds_[index] += log_odds_hit_;
    } else {
        log_odds_[index] += log_odds_miss_;
    }
    
    // Clamp log-odds to prevent overconfidence
    log_odds_[index] = std::clamp(log_odds_[index], log_odds_min_, log_odds_max_);
    
    // Convert log-odds to occupancy probability
    double probability = 1.0 / (1.0 + std::exp(-log_odds_[index]));
    
    // Convert to grid value (0-100 scale, -1 for unknown)
    if (probability < 0.25) {
        grid_data_[index] = 0;  // Free
    } else if (probability > 0.65) {
        grid_data_[index] = 100;  // Occupied
    } else {
        grid_data_[index] = -1;  // Unknown
    }
}

void OccupancyGridLevel::traceLine(int x0, int y0, int x1, int y1) {
    // Bresenham's line algorithm (adapted from Phase 1)
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0;
    int y = y0;
    
    while (true) {
        // Mark cells along the ray as free (except the last one)
        if (x == x1 && y == y1) {
            // Hit point - mark as occupied
            updateCell(x, y, true);
            break;
        } else {
            // Free space along ray
            updateCell(x, y, false);
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
        
        // Safety check to prevent infinite loops
        if (std::abs(x - x0) > 1000 || std::abs(y - y0) > 1000) {
            break;
        }
    }
}

// MapManager Implementation
MapManager::MapManager(double base_resolution, double map_size, int num_levels)
    : base_resolution_(base_resolution), map_size_(map_size), num_levels_(num_levels) {
    
    // Create multi-resolution grid levels
    // Each level has double the resolution (half the cell size) of the previous
    for (int i = 0; i < num_levels_; ++i) {
        double level_resolution = base_resolution_ * std::pow(2.0, i);
        
        // Calculate grid size for this resolution
        int grid_size = static_cast<int>(map_size_ / level_resolution);
        
        // Create grid level centered at origin
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
}

void MapManager::updateMap(const sensor_msgs::msg::LaserScan& scan,
                          const geometry_msgs::msg::Pose& robot_pose) {
    // Update all resolution levels
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

double MapManager::getOccupancyProbability(double world_x, double world_y, int level) const {
    if (level < 0 || level >= static_cast<int>(grid_levels_.size())) {
        return 0.5;  // Unknown
    }
    
    return grid_levels_[level]->getOccupancyProbability(world_x, world_y);
}

void MapManager::reset() {
    for (auto& grid_level : grid_levels_) {
        grid_level->reset();
    }
}

int MapManager::getNumLevels() const {
    return num_levels_;
}

double MapManager::getResolution(int level) const {
    if (level < 0 || level >= static_cast<int>(grid_levels_.size())) {
        return -1.0;
    }
    return grid_levels_[level]->getResolution();
}

} // namespace hector_slam_custom
