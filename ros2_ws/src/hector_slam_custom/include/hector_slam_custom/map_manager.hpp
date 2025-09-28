/**
 * @file map_manager.hpp
 * @brief Multi-resolution occupancy grid management for Hector SLAM
 * @author Brennan Drake
 * 
 * This manages the occupancy grid maps used in Hector SLAM, including
 * multi-resolution representation and efficient updates.
 */

#ifndef HECTOR_SLAM_CUSTOM__MAP_MANAGER_HPP_
#define HECTOR_SLAM_CUSTOM__MAP_MANAGER_HPP_

#include <vector>
#include <memory>
#include <cmath>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace hector_slam_custom {

/**
 * @brief Single occupancy grid level with log-odds representation
 */
class OccupancyGridLevel {
public:
    /**
     * @brief Constructor
     * @param resolution Grid resolution in meters per cell
     * @param size_x Grid width in cells
     * @param size_y Grid height in cells
     * @param origin_x Grid origin x coordinate in world frame
     * @param origin_y Grid origin y coordinate in world frame
     */
    OccupancyGridLevel(double resolution, int size_x, int size_y, 
                      double origin_x, double origin_y);

    /**
     * @brief Update grid with laser scan data
     * @param scan Laser scan data
     * @param robot_pose Robot pose when scan was taken
     */
    void updateWithScan(const sensor_msgs::msg::LaserScan& scan,
                       const geometry_msgs::msg::Pose& robot_pose);

    /**
     * @brief Get occupancy grid message for publishing
     * @param frame_id Frame ID for the grid
     * @return Occupancy grid message
     */
    nav_msgs::msg::OccupancyGrid getOccupancyGridMsg(const std::string& frame_id) const;

    /**
     * @brief Get log-odds value at grid coordinates
     * @param x Grid x coordinate
     * @param y Grid y coordinate
     * @return Log-odds value
     */
    double getLogOdds(int x, int y) const;

    /**
     * @brief Get occupancy probability at world coordinates
     * @param world_x World x coordinate
     * @param world_y World y coordinate
     * @return Occupancy probability [0.0, 1.0]
     */
    double getOccupancyProbability(double world_x, double world_y) const;

    /**
     * @brief Check if world coordinates are within grid bounds
     * @param world_x World x coordinate
     * @param world_y World y coordinate
     * @return True if within bounds
     */
    bool isInBounds(double world_x, double world_y) const;

    /**
     * @brief Convert world coordinates to grid coordinates
     * @param world_x World x coordinate
     * @param world_y World y coordinate
     * @param grid_x Output grid x coordinate
     * @param grid_y Output grid y coordinate
     * @return True if conversion successful and within bounds
     */
    bool worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const;

    /**
     * @brief Convert grid coordinates to world coordinates
     * @param grid_x Grid x coordinate
     * @param grid_y Grid y coordinate
     * @param world_x Output world x coordinate
     * @param world_y Output world y coordinate
     */
    void gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y) const;

    // Getters
    double getResolution() const { return resolution_; }
    int getSizeX() const { return size_x_; }
    int getSizeY() const { return size_y_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }

private:
    // Grid parameters
    double resolution_;
    int size_x_;
    int size_y_;
    double origin_x_;
    double origin_y_;

    // Grid data (log-odds representation)
    std::vector<double> log_odds_grid_;

    // Occupancy update parameters
    double prob_hit_;
    double prob_miss_;
    double prob_prior_;
    double log_odds_hit_;
    double log_odds_miss_;

    /**
     * @brief Update single grid cell with log-odds
     * @param grid_x Grid x coordinate
     * @param grid_y Grid y coordinate
     * @param log_odds_update Log-odds update value
     */
    void updateCell(int grid_x, int grid_y, double log_odds_update);

    /**
     * @brief Trace ray using Bresenham algorithm and update cells
     * @param start_x Ray start x coordinate (grid)
     * @param start_y Ray start y coordinate (grid)
     * @param end_x Ray end x coordinate (grid)
     * @param end_y Ray end y coordinate (grid)
     * @param max_range_hit True if ray hit obstacle within max range
     */
    void traceRay(int start_x, int start_y, int end_x, int end_y, bool max_range_hit);

    /**
     * @brief Convert log-odds to probability
     * @param log_odds Log-odds value
     * @return Probability [0.0, 1.0]
     */
    double logOddsToProbability(double log_odds) const;

    /**
     * @brief Convert probability to occupancy grid value [0, 100, -1]
     * @param probability Probability value [0.0, 1.0]
     * @return Occupancy grid value
     */
    int8_t probabilityToOccupancyValue(double probability) const;
};

/**
 * @brief Multi-resolution map manager for Hector SLAM
 * 
 * Manages multiple occupancy grid levels at different resolutions
 * for efficient scan matching and mapping.
 */
class MapManager {
public:
    /**
     * @brief Constructor
     * @param base_resolution Finest resolution level
     * @param map_size Map size in meters
     * @param num_levels Number of resolution levels
     */
    MapManager(double base_resolution = 0.05, 
              double map_size = 100.0,
              int num_levels = 3);

    /**
     * @brief Destructor
     */
    ~MapManager() = default;

    /**
     * @brief Update all map levels with new scan data
     * @param scan Laser scan data
     * @param robot_pose Robot pose when scan was taken
     */
    void updateMaps(const sensor_msgs::msg::LaserScan& scan,
                   const geometry_msgs::msg::Pose& robot_pose);

    /**
     * @brief Get occupancy grid for specified level
     * @param level Map level (0 = finest resolution)
     * @param frame_id Frame ID for the grid
     * @return Occupancy grid message
     */
    nav_msgs::msg::OccupancyGrid getOccupancyGrid(int level, const std::string& frame_id) const;

    /**
     * @brief Get finest resolution occupancy grid
     * @param frame_id Frame ID for the grid
     * @return Occupancy grid message
     */
    nav_msgs::msg::OccupancyGrid getFinestOccupancyGrid(const std::string& frame_id) const;

    /**
     * @brief Get occupancy grid level for scan matching
     * @param level Map level (0 = finest resolution)
     * @return Pointer to occupancy grid level
     */
    const OccupancyGridLevel* getGridLevel(int level) const;

    /**
     * @brief Get number of map levels
     * @return Number of levels
     */
    int getNumLevels() const { return grid_levels_.size(); }

    /**
     * @brief Check if maps have been initialized
     * @return True if initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Reset all maps to initial state
     */
    void reset();

private:
    // Multi-resolution grid levels
    std::vector<std::unique_ptr<OccupancyGridLevel>> grid_levels_;
    
    // Map parameters
    double base_resolution_;
    double map_size_;
    int num_levels_;
    bool initialized_;

    /**
     * @brief Initialize all grid levels
     */
    void initializeGridLevels();

    /**
     * @brief Calculate grid size in cells for given resolution
     * @param resolution Grid resolution
     * @return Grid size in cells
     */
    int calculateGridSize(double resolution) const;
};

} // namespace hector_slam_custom

#endif // HECTOR_SLAM_CUSTOM__MAP_MANAGER_HPP_
