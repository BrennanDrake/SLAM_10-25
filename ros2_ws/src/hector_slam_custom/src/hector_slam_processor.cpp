/**
 * @file hector_slam_processor.cpp
 * @brief Implementation of the main Hector SLAM processor node
 * 
 * This integrates scan matching (Phase 2.1) and map management (Phase 2.2)
 * into a complete SLAM system.
 */

#include "hector_slam_custom/hector_slam_processor.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <cmath>

namespace hector_slam_custom {

// ============================================================================
// CONSTRUCTOR - The Birth of Our SLAM System
// ============================================================================
HectorSlamProcessor::HectorSlamProcessor(const rclcpp::NodeOptions& options)
    : Node("hector_slam_processor", options) {
    
    RCLCPP_INFO(this->get_logger(), "Initializing Hector SLAM Processor");
    
    // Step 1: Load configuration parameters
    initializeParameters();
    
    // Step 2: Create the SLAM components
    initializeSlamComponents();
    
    // Step 3: Set up ROS 2 communication
    // Subscribe to laser scans
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&HectorSlamProcessor::scanCallback, this, std::placeholders::_1));
    
    // Subscribe to odometry (wheel encoder or other) for motion prediction
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&HectorSlamProcessor::odomCallback, this, std::placeholders::_1));
    
    // Publish the map and robot pose
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);
    
    // Create timers for periodic publishing
    map_publish_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / map_publish_rate_),
        std::bind(&HectorSlamProcessor::mapPublishTimerCallback, this));
    
    pose_publish_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / pose_publish_rate_),
        std::bind(&HectorSlamProcessor::posePublishTimerCallback, this));
    
    // Step 4: Initialize TF2 (coordinate frame management)
    // Jazzy requires: clock, tf2::Duration, node, QoS
    // Note: We can't use shared_from_this() in constructor, so we pass 'this'
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(
        this->get_clock(),
        tf2::durationFromSec(10.0)  // 10 second cache
    );
    
    // TransformListener needs the buffer and node
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);
    
    // TransformBroadcaster needs the node
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    // Initialize EKF process noise from parameters
    ekf_.setProcessNoise(q_pos_, q_theta_);
    last_predict_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Hector SLAM Processor initialized successfully");
}

// ============================================================================
// PARAMETER INITIALIZATION - Loading Our Configuration
// ============================================================================
void HectorSlamProcessor::initializeParameters() {
    // Frame IDs - These define our coordinate systems
    // Think of frames like different perspectives:
    // - map: The global world coordinate system
    // - base_link: The robot's body
    // - laser: Where the laser scanner is mounted
    // - odom: Odometry reference (we'll override this with SLAM)
    
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("laser_frame", "laser");
    
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    laser_frame_ = this->get_parameter("laser_frame").as_string();
    
    // Publishing rates - How often we share updates
    this->declare_parameter("map_publish_rate", 2.0);   // 2 Hz = every 0.5 seconds
    this->declare_parameter("pose_publish_rate", 20.0); // 20 Hz = every 0.05 seconds
    
    map_publish_rate_ = this->get_parameter("map_publish_rate").as_double();
    pose_publish_rate_ = this->get_parameter("pose_publish_rate").as_double();
    
    // Scan matching parameters
    this->declare_parameter("scan_match_threshold", 0.01);  // Convergence threshold
    this->declare_parameter("max_scan_match_iterations", 10);  // Max iterations
    
    scan_match_threshold_ = this->get_parameter("scan_match_threshold").as_double();
    max_scan_match_iterations_ = this->get_parameter("max_scan_match_iterations").as_int();
    
    // Map parameters
    this->declare_parameter("map_resolution", 0.05);
    this->declare_parameter("map_size", 20.0);
    this->declare_parameter("map_update_distance_threshold", 0.4);
    this->declare_parameter("map_update_angle_threshold", 0.2);
    
    // Initial map building
    this->declare_parameter("initial_scan_count", 5);
    
    map_resolution_ = this->get_parameter("map_resolution").as_double();
    map_size_ = this->get_parameter("map_size").as_double();
    map_update_distance_threshold_ = this->get_parameter("map_update_distance_threshold").as_double();
    map_update_angle_threshold_ = this->get_parameter("map_update_angle_threshold").as_double();
    initial_scan_count_ = this->get_parameter("initial_scan_count").as_int();
    
    // EKF / Odometry parameters (Phase 3)
    this->declare_parameter("use_odom", true);
    this->declare_parameter("q_pos", 0.01);
    this->declare_parameter("q_theta", 0.01);
    this->declare_parameter("r_pos", 0.05);
    this->declare_parameter("r_theta", 0.05);
    this->declare_parameter("initial_cov_pos", 0.1);
    this->declare_parameter("initial_cov_theta", 0.1);

    use_odom_ = this->get_parameter("use_odom").as_bool();
    q_pos_ = this->get_parameter("q_pos").as_double();
    q_theta_ = this->get_parameter("q_theta").as_double();
    r_pos_ = this->get_parameter("r_pos").as_double();
    r_theta_ = this->get_parameter("r_theta").as_double();
    init_cov_pos_ = this->get_parameter("initial_cov_pos").as_double();
    init_cov_theta_ = this->get_parameter("initial_cov_theta").as_double();
    
    RCLCPP_INFO(this->get_logger(), "Parameters loaded: resolution=%.3fm, map_size=%.1fm", 
                map_resolution_, map_size_);
}

// ============================================================================
// SLAM COMPONENT INITIALIZATION - Creating Our Tools
// ============================================================================
void HectorSlamProcessor::initializeSlamComponents() {
    // Create the scan matcher (from Phase 2.1)
    scan_matcher_ = std::make_unique<ScanMatcher>();
    // Note: ScanMatcher parameters are currently hardcoded in the class
    // TODO: Add setter methods to ScanMatcher class
    
    // Create the map manager (from Phase 2.2)
    // We'll use 3 resolution levels for multi-resolution matching
    map_manager_ = std::make_unique<MapManager>(
        map_resolution_,  // Base resolution (finest)
        map_size_,        // Map size in meters
        3                 // Number of resolution levels
    );
    
    RCLCPP_INFO(this->get_logger(), "SLAM components initialized");
}

// ============================================================================
// SCAN CALLBACK - The Heart of SLAM (Called for Every Laser Scan)
// ============================================================================
void HectorSlamProcessor::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // This is where the magic happens!
    // Every time we get a new laser scan, we:
    // 1. Figure out where we are (scan matching)
    // 2. Update our map with what we see
    
    if (!first_scan_received_) {
        // First scan - initialize everything at origin
        RCLCPP_INFO(this->get_logger(), "Received first scan, initializing SLAM");
        
        current_pose_.x = 0.0;
        current_pose_.y = 0.0;
        current_pose_.theta = 0.0;
        current_pose_.timestamp = scan->header.stamp;
        
        last_scan_match_pose_ = current_pose_;
        first_scan_received_ = true;
        map_initialized_ = true;
        
        // CRITICAL: Update map with first scan BEFORE any scan matching
        // This populates the map so scan matching has something to match against
        updateMap(scan, current_pose_);
        
        // Log map origin and robot pose for debugging
        nav_msgs::msg::OccupancyGrid debug_map = map_manager_->getFinestOccupancyGrid(map_frame_);
        const auto &origin = debug_map.info.origin.position;
        RCLCPP_INFO(this->get_logger(), "[DEBUG] Map origin: (%.3f, %.3f), Robot initial pose: (%.3f, %.3f, %.3f)",
            origin.x, origin.y, current_pose_.x, current_pose_.y, current_pose_.theta);
        
        // Initialize EKF state and covariance from current pose
        Eigen::Vector3d x0(current_pose_.x, current_pose_.y, current_pose_.theta);
        Eigen::Matrix3d P0 = Eigen::Matrix3d::Zero();
        P0(0,0) = init_cov_pos_;
        P0(1,1) = init_cov_pos_;
        P0(2,2) = init_cov_theta_;
        ekf_.initialize(x0, P0);
        ekf_.setProcessNoise(q_pos_, q_theta_);
        last_predict_time_ = scan->header.stamp;
        
        RCLCPP_INFO(this->get_logger(), "Initial map created from first scan");
        scan_count_++;
        return;
    }
    
    // For the first few scans, just update the map without scan matching
    // This builds up enough map content for scan matching to work
    if (scan_count_ < initial_scan_count_) {
        updateMap(scan, current_pose_);
        scan_count_++;
        RCLCPP_INFO(this->get_logger(), "Building initial map... scan %d/%d", scan_count_, initial_scan_count_);
        return;
    }
    
    // Process the scan through our SLAM pipeline
    if (processScan(scan)) {
        last_scan_time_ = scan->header.stamp;
    }
}

// ============================================================================
// PROCESS SCAN - The SLAM Pipeline
// ============================================================================
bool HectorSlamProcessor::processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 0: EKF predict using latest odometry
    if (use_odom_ && have_odom_) {
        // Use scan timestamp to compute dt for predict
        rclcpp::Time now_t = scan->header.stamp;
        double dt = (now_t - last_predict_time_).seconds();
        if (dt > 0.0) {
            ekf_.predict(odom_lin_vel_, odom_ang_vel_, dt);
            last_predict_time_ = now_t;
        }
    }

    // Predicted pose from EKF (before correction)
    Eigen::Vector3d x_pred = ekf_.getState();
    RobotPose pred_pose;
    pred_pose.x = x_pred(0);
    pred_pose.y = x_pred(1);
    pred_pose.theta = x_pred(2);

    // Step 1: Perform scan matching starting from predicted pose
    RobotPose scanmatch_pose = performScanMatching(scan, pred_pose);

    // Step 2: EKF measurement update with scan-matching result (if converged)
    Eigen::Vector3d z;
    z << scanmatch_pose.x, scanmatch_pose.y, scanmatch_pose.theta;
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0,0) = r_pos_;
    R(1,1) = r_pos_;
    R(2,2) = r_theta_;

    // Determine if the last scan matching converged by comparing to pred (simple check: rely on pose change rate)
    // Here we always attempt update; performScanMatching already falls back to initial pose when not converged
    ekf_.update(z, R);

    // Step 3: Use fused EKF state as the new pose
    Eigen::Vector3d x_fused = ekf_.getState();
    RobotPose new_pose;
    new_pose.x = x_fused(0);
    new_pose.y = x_fused(1);
    new_pose.theta = x_fused(2);
    
    // Step 2: Check if we've moved enough to update the map
    if (shouldUpdateMap(new_pose, last_scan_match_pose_)) {
        updateMap(scan, new_pose);
        last_scan_match_pose_ = new_pose;
    }
    
    // Step 3: Update our current pose estimate
    current_pose_ = new_pose;
    current_pose_.timestamp = scan->header.stamp;
    
    return true;
}

// ============================================================================
// ODOMETRY CALLBACK - Store latest v and omega
// ============================================================================
void HectorSlamProcessor::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract linear velocity along x and angular velocity around z
    odom_lin_vel_ = msg->twist.twist.linear.x;
    odom_ang_vel_ = msg->twist.twist.angular.z;
    have_odom_ = true;
    last_odom_time_ = msg->header.stamp;
}

// ============================================================================
// PERFORM SCAN MATCHING - Finding Our Position
// ============================================================================
HectorSlamProcessor::RobotPose HectorSlamProcessor::performScanMatching(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    const RobotPose& initial_pose) {
    
    // Convert laser scan to point cloud
    std::vector<Point2D> scan_points;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float range = scan->ranges[i];
        
        // Skip invalid measurements
        if (std::isnan(range) || std::isinf(range) ||
            range < scan->range_min || range > scan->range_max) {
            continue;
        }
        
        // Convert to Cartesian coordinates (in laser frame)
        float angle = scan->angle_min + i * scan->angle_increment;
        Point2D point;
        point.x = range * std::cos(angle);
        point.y = range * std::sin(angle);
        scan_points.push_back(point);
    }
    
    // Get the current map for matching
    // We use the finest resolution (level 0) for accurate matching
    nav_msgs::msg::OccupancyGrid current_map = 
        map_manager_->getOccupancyGrid(0, map_frame_);
    
    // Perform scan matching
    Pose2D initial_pose_2d;
    initial_pose_2d.x = initial_pose.x;
    initial_pose_2d.y = initial_pose.y;
    initial_pose_2d.theta = initial_pose.theta;
    
    // Note: Current ScanMatcher expects LaserScan, not points
    // For now, we'll use the scan directly
    ScanMatchResult result = scan_matcher_->matchScan(
        *scan, current_map, initial_pose_2d);
    
    // Convert result back to RobotPose
    RobotPose corrected_pose;
    corrected_pose.x = result.corrected_pose.x;
    corrected_pose.y = result.corrected_pose.y;
    corrected_pose.theta = result.corrected_pose.theta;
    
    // Log detailed info about scan matching result
    RCLCPP_INFO_THROTTLE(this->get_logger(), 
                        *this->get_clock(), 
                        2000,  // Every 2 seconds
                        "Scan match: converged=%s, score=%.3f, iterations=%d, pose=(%.2f, %.2f, %.2f)",
                        result.converged ? "true" : "false",
                        result.match_score, result.iterations_used,
                        result.corrected_pose.x, result.corrected_pose.y, result.corrected_pose.theta);
    
    // Accept scan match if it converged with reasonable score
    // Lower threshold since we're in a static environment with good map
    if (result.converged && result.match_score > 0.1) {  // Lowered from 0.3
        RCLCPP_DEBUG(this->get_logger(), 
                    "Scan matching converged: score=%.3f, iterations=%d",
                    result.match_score, result.iterations_used);
    } else {
        // Scan matching failed - use the initial pose (no correction)
        // This is common when the map is sparse or the robot hasn't moved much
        if (!result.converged) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), 
                                *this->get_clock(), 
                                1000,  // Only warn once per second
                                "Scan matching did not converge (score=%.3f), using prediction",
                                result.match_score);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), 
                                *this->get_clock(), 
                                1000,  // Only warn once per second
                                "Scan matching converged but score too low (%.3f < 0.1), using prediction",
                                result.match_score);
        }
        
        // Still update the map even if scan matching failed
        // This helps populate the map for future matching
        corrected_pose = initial_pose;
    }
    
    return corrected_pose;
}

// ============================================================================
// UPDATE MAP - Adding New Information to Our World Model
// ============================================================================
void HectorSlamProcessor::updateMap(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                                   const RobotPose& robot_pose) {
    // Convert RobotPose to geometry_msgs::Pose for the map manager
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = robot_pose.x;
    pose_msg.position.y = robot_pose.y;
    pose_msg.position.z = 0.0;
    
    // Convert theta to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_pose.theta);
    pose_msg.orientation = tf2::toMsg(q);
    
    // Update all map levels
    map_manager_->updateMaps(*scan, pose_msg);
    
    RCLCPP_DEBUG(this->get_logger(), 
                "Map updated at pose: (%.2f, %.2f, %.2f)",
                robot_pose.x, robot_pose.y, robot_pose.theta);
}

// ============================================================================
// SHOULD UPDATE MAP - Deciding When to Update
// ============================================================================
bool HectorSlamProcessor::shouldUpdateMap(const RobotPose& current_pose, 
                                         const RobotPose& last_pose) const {
    // Calculate how far we've moved
    double dx = current_pose.x - last_pose.x;
    double dy = current_pose.y - last_pose.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Calculate how much we've rotated
    double dtheta = std::abs(current_pose.theta - last_pose.theta);
    // Normalize angle to [-pi, pi]
    while (dtheta > M_PI) dtheta -= 2 * M_PI;
    
    // Update if we've moved OR rotated enough
    return (distance > map_update_distance_threshold_ ||
            std::abs(dtheta) > map_update_angle_threshold_);
}

// ============================================================================
// PUBLISH POSE - Sharing Our Location with the World
// ============================================================================
void HectorSlamProcessor::publishPose() {
    // Publish the robot's pose in two ways:
    // 1. As a TF transform (for navigation stack)
    // 2. As a PoseWithCovariance message (for visualization/logging)
    
    // Create TF transform from map to base_link
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_pose_.timestamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = base_frame_;
    
    // Set translation
    transform.transform.translation.x = current_pose_.x;
    transform.transform.translation.y = current_pose_.y;
    transform.transform.translation.z = 0.0;
    
    // Set rotation (convert theta to quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.theta);
    transform.transform.rotation = tf2::toMsg(q);
    
    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform);
    
    // Also publish as PoseWithCovarianceStamped
    auto pose_msg = poseToPoseWithCovariance(current_pose_);
    pose_publisher_->publish(pose_msg);
}

// ============================================================================
// PUBLISH MAP - Sharing Our World Model
// ============================================================================
void HectorSlamProcessor::publishMap() {
    if (!map_initialized_) {
        return;  // No map to publish yet
    }
    
    // Get the finest resolution map for publishing
    nav_msgs::msg::OccupancyGrid map_msg = 
        map_manager_->getFinestOccupancyGrid(map_frame_);
    
    // Set timestamp
    map_msg.header.stamp = this->now();
    
    // Publish the map
    map_publisher_->publish(map_msg);
}

// ============================================================================
// TIMER CALLBACKS - Periodic Publishing
// ============================================================================
void HectorSlamProcessor::mapPublishTimerCallback() {
    // Called at map_publish_rate_ Hz
    publishMap();
}

void HectorSlamProcessor::posePublishTimerCallback() {
    // Called at pose_publish_rate_ Hz
    if (first_scan_received_) {
        publishPose();
    }
}

// ============================================================================
// UTILITY FUNCTIONS - Helper Methods
// ============================================================================
geometry_msgs::msg::Transform HectorSlamProcessor::poseToTransform(const RobotPose& pose) const {
    geometry_msgs::msg::Transform transform;
    
    // Translation
    transform.translation.x = pose.x;
    transform.translation.y = pose.y;
    transform.translation.z = 0.0;
    
    // Rotation (theta to quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    transform.rotation = tf2::toMsg(q);
    
    return transform;
}

geometry_msgs::msg::PoseWithCovarianceStamped HectorSlamProcessor::poseToPoseWithCovariance(
    const RobotPose& pose) const {
    
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    
    // Header
    msg.header.stamp = pose.timestamp;
    msg.header.frame_id = map_frame_;
    
    // Pose
    msg.pose.pose.position.x = pose.x;
    msg.pose.pose.position.y = pose.y;
    msg.pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    msg.pose.pose.orientation = tf2::toMsg(q);
    
    // Covariance (6x6 matrix, row-major)
    // For now, we'll use fixed uncertainties
    // In a full implementation, this would come from scan matching
    // Format: [x, y, z, rot_x, rot_y, rot_z]
    msg.pose.covariance[0] = 0.01;  // x variance
    msg.pose.covariance[7] = 0.01;  // y variance
    msg.pose.covariance[35] = 0.01; // theta variance
    
    // Note: covariance is a 36-element array (6x6 matrix)
    // Diagonal elements are variances, off-diagonal are covariances
    // We're only setting diagonal elements (assuming independence)
    
    return msg;
}

} // namespace hector_slam_custom
