/**
 * @file landmark_extractor.cpp
 * @brief Implementation of landmark extraction from laser scans
 */

#include "ekf_slam/landmark_extractor.hpp"
#include <cmath>
#include <random>

namespace ekf_slam {

// ============================================================================
// LandmarkExtractor Implementation
// ============================================================================

LandmarkExtractor::LandmarkExtractor(double spike_threshold,
                                     double min_range,
                                     double max_range)
    : spike_threshold_(spike_threshold)
    , min_range_(min_range)
    , max_range_(max_range) {
}

std::vector<LandmarkObservation> LandmarkExtractor::extractLandmarks(
    const sensor_msgs::msg::LaserScan& scan) {
    
    std::vector<LandmarkObservation> landmarks;
    
    // Convert scan to vector for easier processing
    std::vector<float> ranges(scan.ranges.begin(), scan.ranges.end());
    
    // Iterate through scan points looking for spikes
    for (size_t i = 1; i < ranges.size() - 1; ++i) {
        if (!isValidRange(ranges[i])) {
            continue;
        }
        
        // Check if this point is a spike
        if (isSpike(ranges, i)) {
            // Compute angle for this point
            double angle = scan.angle_min + i * scan.angle_increment;
            double range = ranges[i];
            
            // Create landmark observation (range, bearing)
            // Note: ID is -1 (unknown) - will be assigned during data association
            landmarks.emplace_back(range, angle, -1);
        }
    }
    
    return landmarks;
}

bool LandmarkExtractor::isSpike(const std::vector<float>& ranges, size_t index) {
    // A spike is detected when current point is significantly closer
    // than both neighbors (indicates isolated feature like pole/corner)
    
    float current = ranges[index];
    float prev = ranges[index - 1];
    float next = ranges[index + 1];
    
    // Check if neighbors are valid
    if (!isValidRange(prev) || !isValidRange(next)) {
        return false;
    }
    
    // Spike detected if current is much closer than both neighbors
    bool spike_from_prev = (prev - current) > spike_threshold_;
    bool spike_from_next = (next - current) > spike_threshold_;
    
    return spike_from_prev && spike_from_next;
}

std::vector<LandmarkObservation> LandmarkExtractor::extractCorners(
    const sensor_msgs::msg::LaserScan& scan) {
    // TODO: Implement corner detection using line fitting
    // For now, return empty vector
    // This is a more advanced technique for future implementation
    return std::vector<LandmarkObservation>();
}

void LandmarkExtractor::polarToCartesian(double range, double angle, 
                                        double& x, double& y) {
    x = range * std::cos(angle);
    y = range * std::sin(angle);
}

bool LandmarkExtractor::isValidRange(float range) {
    return std::isfinite(range) && 
           range >= min_range_ && 
           range <= max_range_;
}

// ============================================================================
// LandmarkSimulator Implementation
// ============================================================================

LandmarkSimulator::LandmarkSimulator() {
}

void LandmarkSimulator::addLandmark(double x, double y, int id) {
    landmarks_.emplace_back(id, x, y);
}

std::vector<LandmarkObservation> LandmarkSimulator::simulateObservations(
    double robot_x, double robot_y, double robot_theta,
    double max_range, double fov,
    double range_noise, double bearing_noise) {
    
    std::vector<LandmarkObservation> observations;
    
    // Random number generator for noise
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<double> range_dist(0.0, range_noise);
    std::normal_distribution<double> bearing_dist(0.0, bearing_noise);
    
    // For each landmark, check if visible and create observation
    for (const auto& landmark : landmarks_) {
        // Compute relative position
        double dx = landmark.x - robot_x;
        double dy = landmark.y - robot_y;
        
        // Compute range and bearing in world frame
        double range = std::sqrt(dx * dx + dy * dy);
        double bearing_world = std::atan2(dy, dx);
        
        // Convert to robot frame
        double bearing_robot = bearing_world - robot_theta;
        
        // Normalize bearing to [-pi, pi]
        while (bearing_robot > M_PI) bearing_robot -= 2.0 * M_PI;
        while (bearing_robot < -M_PI) bearing_robot += 2.0 * M_PI;
        
        // Check if landmark is within sensor range and FOV
        if (range <= max_range && std::abs(bearing_robot) <= fov / 2.0) {
            // Add measurement noise
            double noisy_range = range + range_dist(gen);
            double noisy_bearing = bearing_robot + bearing_dist(gen);
            
            // Create observation with known landmark ID
            observations.emplace_back(noisy_range, noisy_bearing, landmark.id);
        }
    }
    
    return observations;
}

} // namespace ekf_slam
