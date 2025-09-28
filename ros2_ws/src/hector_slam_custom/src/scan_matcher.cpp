/**
 * @file scan_matcher.cpp
 * @brief Implementation of scan matching algorithm for Hector SLAM
 * @author Brennan Drake
 */

#include "hector_slam_custom/scan_matcher.hpp"
#include <cmath>
#include <algorithm>

namespace hector_slam_custom {

ScanMatcher::ScanMatcher(int max_iterations, 
                        double convergence_threshold,
                        double match_threshold)
    : max_iterations_(max_iterations)
    , convergence_threshold_(convergence_threshold)
    , match_threshold_(match_threshold) {
}

void ScanMatcher::setParameters(int max_iterations, 
                               double convergence_threshold,
                               double match_threshold) {
    max_iterations_ = max_iterations;
    convergence_threshold_ = convergence_threshold;
    match_threshold_ = match_threshold;
}

ScanMatchResult ScanMatcher::matchScan(const sensor_msgs::msg::LaserScan& scan,
                                      const nav_msgs::msg::OccupancyGrid& grid,
                                      const Pose2D& initial_pose) {
    ScanMatchResult result;
    result.corrected_pose = initial_pose;
    
    // Convert scan to point cloud
    std::vector<Point2D> scan_points = scanToPointCloud(scan);
    
    if (scan_points.empty()) {
        result.match_score = 0.0;
        result.converged = false;
        return result;
    }
    
    // Gauss-Newton optimization loop
    Pose2D current_pose = initial_pose;
    
    for (int iteration = 0; iteration < max_iterations_; ++iteration) {
        Pose2D old_pose = current_pose;
        
        // Perform one optimization step
        current_pose = optimizePose(scan_points, grid, current_pose);
        
        result.iterations_used = iteration + 1;
        
        // Check for convergence
        if (hasConverged(old_pose, current_pose)) {
            result.converged = true;
            break;
        }
    }
    
    // Compute final match score and covariance
    result.corrected_pose = current_pose;
    result.match_score = computeMatchScore(scan_points, grid, current_pose);
    
    // Compute pose covariance from final Hessian
    std::vector<Point2D> final_transformed_points = transformPointCloud(scan_points, current_pose);
    Eigen::Matrix3d final_hessian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d final_gradient = Eigen::Vector3d::Zero();
    
    buildOptimizationMatrices(final_transformed_points, grid, current_pose, final_hessian, final_gradient);
    
    // Covariance = inverse of information matrix (Hessian)
    if (final_hessian.determinant() > 1e-6) {
        result.pose_covariance = final_hessian.inverse();
    } else {
        // Singular Hessian → high uncertainty
        result.pose_covariance = Eigen::Matrix3d::Identity() * 1000.0;
    }
    
    return result;
}

std::vector<Point2D> ScanMatcher::scanToPointCloud(const sensor_msgs::msg::LaserScan& scan) {
    std::vector<Point2D> points;
    points.reserve(scan.ranges.size());
    
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float range = scan.ranges[i];
        
        // Skip invalid measurements
        if (range < scan.range_min || range > scan.range_max || !std::isfinite(range)) {
            continue;
        }
        
        // Convert to Cartesian coordinates
        double angle = scan.angle_min + i * scan.angle_increment;
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);
        
        points.emplace_back(x, y);
    }
    
    return points;
}

std::vector<Point2D> ScanMatcher::transformPointCloud(const std::vector<Point2D>& points,
                                                     const Pose2D& pose) {
    std::vector<Point2D> transformed_points;
    transformed_points.reserve(points.size());
    
    double cos_theta = std::cos(pose.theta);
    double sin_theta = std::sin(pose.theta);
    
    for (const auto& point : points) {
        Point2D transformed;
        transformed.x = pose.x + cos_theta * point.x - sin_theta * point.y;
        transformed.y = pose.y + sin_theta * point.x + cos_theta * point.y;
        transformed_points.push_back(transformed);
    }
    
    return transformed_points;
}

double ScanMatcher::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool ScanMatcher::hasConverged(const Pose2D& old_pose, const Pose2D& new_pose) {
    double dx = new_pose.x - old_pose.x;
    double dy = new_pose.y - old_pose.y;
    double dtheta = normalizeAngle(new_pose.theta - old_pose.theta);
    
    double translation_change = std::sqrt(dx * dx + dy * dy);
    double rotation_change = std::abs(dtheta);
    
    return (translation_change < convergence_threshold_ && 
            rotation_change < convergence_threshold_);
}

// Core optimization functions

Pose2D ScanMatcher::optimizePose(const std::vector<Point2D>& points,
                                const nav_msgs::msg::OccupancyGrid& grid,
                                const Pose2D& current_pose) {
    // Step 1: Transform scan points by current pose estimate
    std::vector<Point2D> transformed_points = transformPointCloud(points, current_pose);
    
    // Step 2: Build optimization matrices (Hessian and gradient)
    Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    
    buildOptimizationMatrices(transformed_points, grid, current_pose, hessian, gradient);
    
    // Step 3: Solve the linear system H * delta_pose = gradient
    // Check if Hessian is invertible
    if (hessian.determinant() < 1e-6) {
        // Hessian is singular, return current pose
        return current_pose;
    }
    
    // YOUR TASK: Solve for pose update
    // Gauss-Newton update: delta_pose = -H^(-1) * gradient
    // The negative sign is because we're minimizing the error
    
    Eigen::Vector3d delta_pose = -hessian.inverse() * gradient;
    
    // Step 4: Apply pose update
    Pose2D updated_pose;
    updated_pose.x = current_pose.x + delta_pose(0);
    updated_pose.y = current_pose.y + delta_pose(1);
    updated_pose.theta = normalizeAngle(current_pose.theta + delta_pose(2));
    
    return updated_pose;
}

double ScanMatcher::computeMatchScore(const std::vector<Point2D>& points,
                                    const nav_msgs::msg::OccupancyGrid& grid,
                                    const Pose2D& pose) {
    if (points.empty()) {
        return 0.0;
    }
    
    // Transform scan points to world coordinates
    std::vector<Point2D> transformed_points = transformPointCloud(points, pose);
    
    double total_score = 0.0;
    int valid_points = 0;
    
    // Compute average occupancy probability at all laser endpoints
    // High score = laser points hit obstacles (good alignment)
    // Low score = laser points hit free space (poor alignment)
    for (const auto& point : transformed_points) {
        double occupancy_prob = getOccupancyProbability(grid, point.x, point.y);
        
        // Skip points outside grid bounds (they return 0.5)
        if (occupancy_prob != 0.5) {
            total_score += occupancy_prob;
            valid_points++;
        }
    }
    
    // Return average score [0.0, 1.0]
    return (valid_points > 0) ? (total_score / valid_points) : 0.0;
}

double ScanMatcher::getOccupancyProbability(const nav_msgs::msg::OccupancyGrid& grid,
                                          double x, double y) {
    // Convert world coordinates to grid coordinates
    int grid_x, grid_y;
    if (!worldToGrid(grid, x, y, grid_x, grid_y)) {
        // Point is outside grid bounds
        return 0.5;  // Unknown probability
    }
    
    // Get the grid value at this location
    int grid_index = grid_y * grid.info.width + grid_x;
    int8_t grid_value = grid.data[grid_index];
    
    // Convert grid value to probability
    if (grid_value == -1) {
        return 0.5;  // Unknown cells
    } else {
        return (grid_value / 100.0);  // Convert 0-100 to 0.0-1.0
    }
}

bool ScanMatcher::worldToGrid(const nav_msgs::msg::OccupancyGrid& grid,
                             double x, double y,
                             int& grid_x, int& grid_y) {
    
    grid_x = static_cast<int>((x - grid.info.origin.position.x) / grid.info.resolution);
    grid_y = static_cast<int>((y - grid.info.origin.position.y) / grid.info.resolution);
    
    return (grid_x >= 0 && grid_x < grid.info.width && grid_y >= 0 && grid_y < grid.info.height);
}
void ScanMatcher::buildOptimizationMatrices(const std::vector<Point2D>& points,
                                          const nav_msgs::msg::OccupancyGrid& grid,
                                          const Pose2D& pose,
                                          Eigen::Matrix3d& hessian,
                                          Eigen::Vector3d& gradient) {
    // Initialize matrices to zero
    hessian.setZero();
    gradient.setZero();
    
    // Process each laser scan point
    for (const auto& point : points) {
        // Get occupancy probability and gradient at this point
        double occupancy_prob = getOccupancyProbability(grid, point.x, point.y);
        Eigen::Vector2d occupancy_gradient = computeOccupancyGradient(grid, point.x, point.y);
        
        // Compute error: we want high occupancy (1.0) at laser endpoints
        double error = 1.0 - occupancy_prob;
        
        // Compute Jacobian: how does point position change with robot pose?
        // point = (x, y) in world frame
        // robot pose = (px, py, ptheta)
        // Jacobian = d(point)/d(pose) = [dx/dpx, dx/dpy, dx/dptheta]
        //                               [dy/dpx, dy/dpy, dy/dptheta]
        
        Eigen::Matrix<double, 2, 3> jacobian;
        jacobian(0, 0) = 1.0;  // dx/dpx = 1
        jacobian(0, 1) = 0.0;  // dx/dpy = 0
        jacobian(1, 0) = 0.0;  // dy/dpx = 0  
        jacobian(1, 1) = 1.0;  // dy/dpy = 1
        
        // Compute rotation derivatives: how point position changes with robot rotation
        
        // Transform point back to robot frame
        double cos_theta = std::cos(pose.theta);
        double sin_theta = std::sin(pose.theta);
        double point_robot_x = cos_theta * (point.x - pose.x) + sin_theta * (point.y - pose.y);
        double point_robot_y = -sin_theta * (point.x - pose.x) + cos_theta * (point.y - pose.y);
        
        jacobian(0, 2) = -sin_theta * point_robot_x - cos_theta * point_robot_y;
        jacobian(1, 2) = cos_theta * point_robot_x - sin_theta * point_robot_y;
        
        // Build optimization matrices using chain rule
        // gradient += J^T * occupancy_gradient * error
        // hessian += J^T * occupancy_gradient * occupancy_gradient^T * J
        
        Eigen::Vector3d point_gradient = jacobian.transpose() * occupancy_gradient * error;
        Eigen::Matrix3d point_hessian = jacobian.transpose() * occupancy_gradient * occupancy_gradient.transpose() * jacobian;
        
        gradient += point_gradient;
        hessian += point_hessian;
    }
}

Eigen::Vector2d ScanMatcher::computeOccupancyGradient(const nav_msgs::msg::OccupancyGrid& grid,
                                                    double x, double y) {
    // Compute gradient using finite differences
    double resolution = grid.info.resolution;
    
    // Get occupancy probabilities at neighboring points
    double prob_x_plus = getOccupancyProbability(grid, x + resolution, y);
    double prob_x_minus = getOccupancyProbability(grid, x - resolution, y);
    double prob_y_plus = getOccupancyProbability(grid, x, y + resolution);
    double prob_y_minus = getOccupancyProbability(grid, x, y - resolution);
    
    // Compute gradients using central differences
    
    double gradient_x = (prob_x_plus - prob_x_minus) / (2 * resolution);
    double gradient_y = (prob_y_plus - prob_y_minus) / (2 * resolution);
    
    return Eigen::Vector2d(gradient_x, gradient_y);
}

} // namespace hector_slam_custom
