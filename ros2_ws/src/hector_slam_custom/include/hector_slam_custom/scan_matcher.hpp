/**
 * @file scan_matcher.hpp
 * @brief Scan matching algorithm implementation for Hector SLAM
 * @author Brennan Drake
 * 
 * This implements the core scan matching algorithm that aligns laser scans
 * with the existing occupancy grid to estimate robot pose corrections.
 */

#ifndef HECTOR_SLAM_CUSTOM__SCAN_MATCHER_HPP_
#define HECTOR_SLAM_CUSTOM__SCAN_MATCHER_HPP_

#include <vector>
#include <memory>
#include <Eigen/Dense>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace hector_slam_custom {

/**
 * @brief 2D point structure for scan matching
 */
struct Point2D {
    double x;
    double y;
    
    Point2D() : x(0.0), y(0.0) {}
    Point2D(double x_val, double y_val) : x(x_val), y(y_val) {}
};

/**
 * @brief 2D pose structure (x, y, theta)
 */
struct Pose2D {
    double x;
    double y;
    double theta;
    
    Pose2D() : x(0.0), y(0.0), theta(0.0) {}
    Pose2D(double x_val, double y_val, double theta_val) 
        : x(x_val), y(y_val), theta(theta_val) {}
};

/**
 * @brief Scan matching result with confidence metrics
 */
struct ScanMatchResult {
    Pose2D corrected_pose;
    double match_score;
    int iterations_used;
    bool converged;
    Eigen::Matrix3d pose_covariance;  // NEW: Uncertainty matrix!
    
    ScanMatchResult() : match_score(0.0), iterations_used(0), converged(false) {
        pose_covariance = Eigen::Matrix3d::Identity() * 1000.0;  // High initial uncertainty
    }
};

/**
 * @brief Scan matcher class implementing Gauss-Newton optimization
 * 
 * This class performs scan matching by:
 * 1. Converting laser scan to point cloud
 * 2. Computing gradients of occupancy grid
 * 3. Using Gauss-Newton method to optimize pose
 * 4. Iterating until convergence or max iterations
 */
class ScanMatcher {
public:
    /**
     * @brief Constructor with configuration parameters
     * @param max_iterations Maximum number of optimization iterations
     * @param convergence_threshold Threshold for convergence detection
     * @param match_threshold Minimum match score to accept result
     */
    ScanMatcher(int max_iterations = 10, 
                double convergence_threshold = 0.001,
                double match_threshold = 0.5);

    /**
     * @brief Destructor
     */
    ~ScanMatcher() = default;

    /**
     * @brief Match a laser scan against an occupancy grid
     * @param scan Laser scan to match
     * @param grid Occupancy grid to match against
     * @param initial_pose Initial pose estimate
     * @return Scan match result with corrected pose
     */
    ScanMatchResult matchScan(const sensor_msgs::msg::LaserScan& scan,
                             const nav_msgs::msg::OccupancyGrid& grid,
                             const Pose2D& initial_pose);

    /**
     * @brief Set scan matching parameters
     * @param max_iterations Maximum optimization iterations
     * @param convergence_threshold Convergence threshold
     * @param match_threshold Minimum acceptable match score
     */
    void setParameters(int max_iterations, 
                      double convergence_threshold,
                      double match_threshold);

private:
    // =============================================================================
    // CONFIGURATION PARAMETERS
    // =============================================================================
    
    int max_iterations_;
    double convergence_threshold_;
    double match_threshold_;
    
    // =============================================================================
    // SCAN PROCESSING
    // =============================================================================
    
    /**
     * @brief Convert laser scan to point cloud in robot frame
     * @param scan Laser scan message
     * @return Vector of 2D points
     */
    std::vector<Point2D> scanToPointCloud(const sensor_msgs::msg::LaserScan& scan);
    
    /**
     * @brief Transform point cloud by given pose
     * @param points Input point cloud
     * @param pose Transformation pose
     * @return Transformed point cloud
     */
    std::vector<Point2D> transformPointCloud(const std::vector<Point2D>& points,
                                            const Pose2D& pose);
    
    // =============================================================================
    // GRID OPERATIONS
    // =============================================================================
    
    /**
     * @brief Get occupancy probability at world coordinates
     * @param grid Occupancy grid
     * @param x World x coordinate
     * @param y World y coordinate
     * @return Occupancy probability [0.0, 1.0]
     */
    double getOccupancyProbability(const nav_msgs::msg::OccupancyGrid& grid,
                                  double x, double y);
    
    /**
     * @brief Compute gradient of occupancy grid at world coordinates
     * @param grid Occupancy grid
     * @param x World x coordinate
     * @param y World y coordinate
     * @return Gradient vector (dx, dy)
     */
    Eigen::Vector2d computeOccupancyGradient(const nav_msgs::msg::OccupancyGrid& grid,
                                           double x, double y);
    
    /**
     * @brief Convert world coordinates to grid indices
     * @param grid Occupancy grid
     * @param x World x coordinate
     * @param y World y coordinate
     * @param grid_x Output grid x index
     * @param grid_y Output grid y index
     * @return True if coordinates are within grid bounds
     */
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid& grid,
                    double x, double y,
                    int& grid_x, int& grid_y);
    
    // =============================================================================
    // OPTIMIZATION
    // =============================================================================
    
    /**
     * @brief Perform one iteration of Gauss-Newton optimization
     * @param points Point cloud to match
     * @param grid Occupancy grid
     * @param current_pose Current pose estimate
     * @return Updated pose estimate
     */
    Pose2D optimizePose(const std::vector<Point2D>& points,
                       const nav_msgs::msg::OccupancyGrid& grid,
                       const Pose2D& current_pose);
    
    /**
     * @brief Compute match score for current pose
     * @param points Point cloud
     * @param grid Occupancy grid
     * @param pose Pose to evaluate
     * @return Match score [0.0, 1.0]
     */
    double computeMatchScore(const std::vector<Point2D>& points,
                           const nav_msgs::msg::OccupancyGrid& grid,
                           const Pose2D& pose);
    
    /**
     * @brief Build Hessian matrix and gradient vector for optimization
     * @param points Point cloud
     * @param grid Occupancy grid
     * @param pose Current pose
     * @param hessian Output Hessian matrix (3x3)
     * @param gradient Output gradient vector (3x1)
     */
    void buildOptimizationMatrices(const std::vector<Point2D>& points,
                                  const nav_msgs::msg::OccupancyGrid& grid,
                                  const Pose2D& pose,
                                  Eigen::Matrix3d& hessian,
                                  Eigen::Vector3d& gradient);
    
    // =============================================================================
    // UTILITY FUNCTIONS
    // =============================================================================
    
    /**
     * @brief Normalize angle to [-pi, pi]
     * @param angle Input angle in radians
     * @return Normalized angle
     */
    double normalizeAngle(double angle);
    
    /**
     * @brief Check if pose change is below convergence threshold
     * @param old_pose Previous pose
     * @param new_pose Current pose
     * @return True if converged
     */
    bool hasConverged(const Pose2D& old_pose, const Pose2D& new_pose);
    
    /**
     * @brief Bilinear interpolation for smooth occupancy values
     * @param grid Occupancy grid
     * @param x Continuous x coordinate in grid space
     * @param y Continuous y coordinate in grid space
     * @return Interpolated occupancy value
     */
    double bilinearInterpolation(const nav_msgs::msg::OccupancyGrid& grid,
                               double x, double y);
};

} // namespace hector_slam_custom

#endif // HECTOR_SLAM_CUSTOM__SCAN_MATCHER_HPP_
