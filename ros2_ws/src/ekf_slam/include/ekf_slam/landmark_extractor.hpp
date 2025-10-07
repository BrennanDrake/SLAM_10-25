/**
 * @file landmark_extractor.hpp
 * @brief Extract landmarks from laser scan data
 * 
 * Landmarks are distinctive features in the environment that can be:
 * 1. Consistently detected across multiple scans
 * 2. Uniquely identified (data association)
 * 3. Used to correct robot pose estimate
 * 
 * Common landmark types:
 * - Corners (wall intersections)
 * - Spikes (poles, pillars)
 * - Line endpoints
 * 
 * This implementation uses spike detection for simplicity.
 */

#ifndef EKF_SLAM__LANDMARK_EXTRACTOR_HPP_
#define EKF_SLAM__LANDMARK_EXTRACTOR_HPP_

#include <vector>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "ekf_slam/ekf_slam.hpp"

namespace ekf_slam {

/**
 * @brief Extract landmarks from laser scan data
 */
class LandmarkExtractor {
public:
    /**
     * @brief Constructor
     * @param spike_threshold Minimum range difference to detect spike (meters)
     * @param min_range Minimum valid range (meters)
     * @param max_range Maximum valid range (meters)
     */
    LandmarkExtractor(double spike_threshold = 0.3,
                     double min_range = 0.5,
                     double max_range = 10.0);
    
    /**
     * @brief Extract landmarks from laser scan
     * 
     * Process:
     * 1. Iterate through scan points
     * 2. Detect spikes (sudden range changes)
     * 3. Convert to robot-frame coordinates
     * 4. Return as landmark observations
     * 
     * @param scan Laser scan message
     * @return Vector of landmark observations (range, bearing)
     */
    std::vector<LandmarkObservation> extractLandmarks(
        const sensor_msgs::msg::LaserScan& scan);
    
    /**
     * @brief Extract landmarks using corner detection
     * 
     * More sophisticated approach:
     * 1. Fit lines to scan segments
     * 2. Find line intersections (corners)
     * 3. Return corners as landmarks
     * 
     * @param scan Laser scan message
     * @return Vector of landmark observations
     */
    std::vector<LandmarkObservation> extractCorners(
        const sensor_msgs::msg::LaserScan& scan);
    
    /**
     * @brief Set spike detection threshold
     * @param threshold Threshold in meters
     */
    void setSpikeThreshold(double threshold) { spike_threshold_ = threshold; }
    
private:
    double spike_threshold_;  // Minimum range difference for spike
    double min_range_;       // Minimum valid range
    double max_range_;       // Maximum valid range
    
    /**
     * @brief Check if point is a spike (isolated feature)
     * 
     * A spike is detected when:
     * - Current point is much closer than neighbors
     * - Indicates a pole, corner, or isolated object
     * 
     * @param ranges Scan ranges
     * @param index Current point index
     * @return True if spike detected
     */
    bool isSpike(const std::vector<float>& ranges, size_t index);
    
    /**
     * @brief Convert scan point to Cartesian coordinates
     * @param range Distance to point
     * @param angle Angle to point (radians)
     * @param x Output x coordinate
     * @param y Output y coordinate
     */
    void polarToCartesian(double range, double angle, double& x, double& y);
    
    /**
     * @brief Check if range is valid
     * @param range Range value
     * @return True if valid
     */
    bool isValidRange(float range);
};

/**
 * @brief Simple landmark simulator for testing
 * 
 * Creates known landmarks in the environment for testing EKF-SLAM
 * without needing real sensor data.
 */
class LandmarkSimulator {
public:
    /**
     * @brief Constructor
     */
    LandmarkSimulator();
    
    /**
     * @brief Add a landmark to the environment
     * @param x X position (meters)
     * @param y Y position (meters)
     * @param id Landmark ID
     */
    void addLandmark(double x, double y, int id);
    
    /**
     * @brief Simulate landmark observations from robot pose
     * 
     * For each landmark:
     * 1. Compute range and bearing from robot
     * 2. Add measurement noise
     * 3. Check if within sensor range/FOV
     * 
     * @param robot_x Robot x position
     * @param robot_y Robot y position
     * @param robot_theta Robot orientation (radians)
     * @param max_range Maximum sensor range
     * @param fov Field of view (radians)
     * @param range_noise Range measurement noise std dev
     * @param bearing_noise Bearing measurement noise std dev
     * @return Vector of simulated observations
     */
    std::vector<LandmarkObservation> simulateObservations(
        double robot_x, double robot_y, double robot_theta,
        double max_range = 10.0,
        double fov = M_PI,
        double range_noise = 0.1,
        double bearing_noise = 0.05);
    
    /**
     * @brief Get all landmarks
     * @return Vector of landmarks
     */
    std::vector<Landmark> getLandmarks() const { return landmarks_; }
    
    /**
     * @brief Clear all landmarks
     */
    void clear() { landmarks_.clear(); }
    
private:
    std::vector<Landmark> landmarks_;
};

} // namespace ekf_slam

#endif // EKF_SLAM__LANDMARK_EXTRACTOR_HPP_
