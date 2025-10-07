/**
 * @file hector_slam_processor.hpp
 * @brief Main Hector SLAM processor node implementation
 * @author Brennan Drake
 * 
 * This is the main ROS 2 node that coordinates scan matching, mapping, and pose estimation
 * for the Hector SLAM algorithm implementation.
 */

#ifndef HECTOR_SLAM_CUSTOM__HECTOR_SLAM_PROCESSOR_HPP_
#define HECTOR_SLAM_CUSTOM__HECTOR_SLAM_PROCESSOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// Include native TF2 headers for Jazzy
// The weird double directory is ONLY for the include path, not the namespace
#ifdef ROS_JAZZY_TF2_WORKAROUND
  // Jazzy has this weird double directory structure
  #include "tf2_ros/tf2_ros/buffer.h"
  #include "tf2_ros/tf2_ros/transform_listener.h"
  #include "tf2_ros/tf2_ros/transform_broadcaster.h"
  #include "tf2/time.h"
#else
  // Normal ROS 2 includes (for other distros)
  #include "tf2_ros/buffer.h"
  #include "tf2_ros/transform_listener.h"
  #include "tf2_ros/transform_broadcaster.h"
  #include "tf2/time.h"
#endif

// Include our SLAM components
#include "hector_slam_custom/scan_matcher.hpp"
#include "hector_slam_custom/map_manager.hpp"
#include "hector_slam_custom/ekf_filter.hpp"

namespace hector_slam_custom {

/**
 * @brief Main Hector SLAM processor node
 * 
 * This node implements the core Hector SLAM algorithm:
 * 1. Receives laser scans
 * 2. Performs scan matching against current map
 * 3. Updates robot pose estimate
 * 4. Updates occupancy grid map
 * 5. Publishes updated pose and map
 */
class HectorSlamProcessor : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     * @param options Node options for ROS 2 configuration
     */
    explicit HectorSlamProcessor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Destructor
     */
    virtual ~HectorSlamProcessor() = default;

private:
    // =============================================================================
    // ROS 2 INTERFACE
    // =============================================================================
    
    /**
     * @brief Callback for incoming laser scan messages
     * @param scan Laser scan message
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    /**
     * @brief Timer callback for periodic map publishing
     */
    void mapPublishTimerCallback();

    /**
     * @brief Timer callback for periodic pose publishing
     */
    void posePublishTimerCallback();

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;

    // Timers
    rclcpp::TimerBase::SharedPtr map_publish_timer_;
    rclcpp::TimerBase::SharedPtr pose_publish_timer_;

    // TF2 components (using native Jazzy types)
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // =============================================================================
    // SLAM COMPONENTS
    // =============================================================================
    
    // Core SLAM algorithms
    std::unique_ptr<ScanMatcher> scan_matcher_;
    std::unique_ptr<MapManager> map_manager_;

    // EKF for pose fusion (Phase 3)
    EkfFilter ekf_;

    // =============================================================================
    // STATE VARIABLES
    // =============================================================================
    
    // Current robot pose estimate (x, y, theta)
    struct RobotPose {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        rclcpp::Time timestamp;
        
        RobotPose() : timestamp(rclcpp::Time(0)) {}
    };
    
    RobotPose current_pose_;
    RobotPose last_scan_match_pose_;
    
    // Scan processing state
    bool first_scan_received_ = false;
    rclcpp::Time last_scan_time_;
    rclcpp::Time last_predict_time_;
    
    // Map state
    bool map_initialized_ = false;

    // =============================================================================
    // EKF / ODOMETRY PARAMETERS & STATE
    // =============================================================================
    bool use_odom_ = true;
    double q_pos_ = 0.01;      // process noise for x,y
    double q_theta_ = 0.01;    // process noise for theta
    double r_pos_ = 0.05;      // measurement noise for x,y
    double r_theta_ = 0.05;    // measurement noise for theta
    double init_cov_pos_ = 0.1;   // initial covariance diag for x,y
    double init_cov_theta_ = 0.1; // initial covariance diag for theta

    // Latest odometry twist (v, omega) and timestamp
    bool have_odom_ = false;
    double odom_lin_vel_ = 0.0;   // m/s
    double odom_ang_vel_ = 0.0;   // rad/s
    rclcpp::Time last_odom_time_;
    
    // Scan counter for initial map building
    int scan_count_ = 0;
    
    // =============================================================================
    // PARAMETERS
    // =============================================================================
    
    // Frame IDs
    std::string map_frame_;
    std::string base_frame_;
    std::string odom_frame_;
    std::string laser_frame_;
    
    // Publishing rates
    double map_publish_rate_;
    double pose_publish_rate_;
    
    // Scan matching parameters
    double scan_match_threshold_;
    int max_scan_match_iterations_;
    
    // Map parameters
    double map_resolution_;
    double map_size_;
    double map_update_distance_threshold_;
    double map_update_angle_threshold_;
    
    // Initial map building
    int initial_scan_count_;  // Number of scans to build map before scan matching
    
    // =============================================================================
    // PRIVATE METHODS
    // =============================================================================
    
    /**
     * @brief Initialize all parameters from ROS parameter server
     */
    void initializeParameters();
    
    /**
     * @brief Initialize SLAM components (scan matcher, map manager)
     */
    void initializeSlamComponents();

    /**
     * @brief Odometry callback to store latest linear/angular velocities
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    /**
     * @brief Process a laser scan through the SLAM pipeline
     * @param scan Laser scan to process
     * @return True if scan was successfully processed
     */
    bool processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
    /**
     * @brief Perform scan matching to estimate robot pose
     * @param scan Current laser scan
     * @param initial_pose Initial pose estimate
     * @return Corrected pose estimate
     */
    RobotPose performScanMatching(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                                  const RobotPose& initial_pose);
    
    /**
     * @brief Update the occupancy grid map with new scan data
     * @param scan Laser scan data
     * @param robot_pose Robot pose when scan was taken
     */
    void updateMap(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                   const RobotPose& robot_pose);
    
    /**
     * @brief Publish current robot pose as TF transform and pose message
     */
    void publishPose();
    
    /**
     * @brief Publish current occupancy grid map
     */
    void publishMap();
    
    /**
     * @brief Check if robot has moved enough to warrant map update
     * @param current_pose Current robot pose
     * @param last_pose Last pose when map was updated
     * @return True if robot has moved significantly
     */
    bool shouldUpdateMap(const RobotPose& current_pose, const RobotPose& last_pose) const;
    
    /**
     * @brief Convert RobotPose to geometry_msgs::msg::Transform
     * @param pose Robot pose to convert
     * @return Transform message
     */
    geometry_msgs::msg::Transform poseToTransform(const RobotPose& pose) const;
    
    /**
     * @brief Convert RobotPose to geometry_msgs::msg::PoseWithCovarianceStamped
     * @param pose Robot pose to convert
     * @return Pose with covariance message
     */
    geometry_msgs::msg::PoseWithCovarianceStamped poseToPoseWithCovariance(const RobotPose& pose) const;
};

} // namespace hector_slam_custom

#endif // HECTOR_SLAM_CUSTOM__HECTOR_SLAM_PROCESSOR_HPP_
