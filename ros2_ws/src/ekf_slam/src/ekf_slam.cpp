/**
 * @file ekf_slam.cpp
 * @brief Implementation of Extended Kalman Filter SLAM
 * 
 * EKF-SLAM Algorithm:
 * 1. PREDICT: Use motion model to predict new robot pose
 * 2. CORRECT: Use landmark observations to correct pose and update landmarks
 * 
 * State representation:
 * mu = [x, y, theta, lm1_x, lm1_y, lm2_x, lm2_y, ...]
 * P = Covariance matrix (grows with landmarks)
 */

#include "ekf_slam/ekf_slam.hpp"
#include <cmath>
#include <iostream>

namespace ekf_slam {

// ============================================================================
// Constructor
// ============================================================================

EkfSlam::EkfSlam(double motion_noise_v,
                 double motion_noise_w,
                 double sensor_noise_range,
                 double sensor_noise_bearing)
    : motion_noise_v_(motion_noise_v)
    , motion_noise_w_(motion_noise_w)
    , sensor_noise_range_(sensor_noise_range)
    , sensor_noise_bearing_(sensor_noise_bearing)
    , next_landmark_id_(0) {
    
    // Initialize state: robot at origin
    mu_ = Eigen::VectorXd::Zero(3);  // [x, y, theta]
    
    // Initialize covariance: small initial uncertainty
    P_ = Eigen::MatrixXd::Identity(3, 3) * 0.1;
}

// ============================================================================
// PREDICTION STEP
// ============================================================================

void EkfSlam::predict(const MotionCommand& motion) {
    // Extract current robot pose
    Eigen::Vector3d robot_pose = mu_.head<3>();
    
    // 1. Predict new robot pose using motion model
    Eigen::Vector3d predicted_pose = motionModel(robot_pose, motion);
    
    // 2. Compute Jacobian of motion model
    Eigen::Matrix3d G = motionJacobian(robot_pose, motion);
    
    // 3. Motion noise covariance (in control space)
    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();
    M(0, 0) = motion_noise_v_ * motion_noise_v_;
    M(1, 1) = motion_noise_w_ * motion_noise_w_;
    
    // 4. Map motion noise to state space
    // V is the Jacobian of motion model w.r.t. control inputs [v, w]
    Eigen::Matrix<double, 3, 2> V;
    double theta = robot_pose(2);
    double dt = motion.dt;
    
    V << dt * std::cos(theta), 0,
         dt * std::sin(theta), 0,
         0,                    dt;
    
    // Motion noise in state space: V * M * V^T
    Eigen::Matrix3d R = V * M * V.transpose();
    
    // 5. Update state estimate (only robot pose changes)
    mu_.head<3>() = predicted_pose;
    
    // 6. Update covariance
    // P = G * P * G^T + R (with proper block structure)
    
    int state_size = mu_.size();
    
    // Create augmented Jacobian (robot part changes, landmarks stay same)
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_size, state_size);
    F.block<3, 3>(0, 0) = G;
    
    // Create augmented noise matrix
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(state_size, state_size);
    Q.block<3, 3>(0, 0) = R;
    
    // Update covariance: P = F * P * F^T + Q
    P_ = F * P_ * F.transpose() + Q;
}

// ============================================================================
// CORRECTION STEP
// ============================================================================

void EkfSlam::correct(const std::vector<LandmarkObservation>& observations) {
    Eigen::Vector3d robot_pose = mu_.head<3>();
    
    for (const auto& obs : observations) {
        // 1. Data association: match observation to landmark
        int landmark_id = dataAssociation(obs);
        
        // 2. If new landmark, initialize it
        if (landmark_id == -1) {
            landmark_id = addLandmark(obs);
            continue;  // Skip correction for newly initialized landmarks
        }
        
        // 3. Get landmark position from state
        int lm_idx = landmark_to_state_[landmark_id];
        Eigen::Vector2d landmark_pos = mu_.segment<2>(lm_idx);
        
        // 4. Compute expected observation
        Eigen::Vector2d z_expected = measurementModel(robot_pose, landmark_pos);
        
        // 5. Compute innovation (measurement residual)
        Eigen::Vector2d z_actual(obs.range, obs.bearing);
        Eigen::Vector2d innovation = z_actual - z_expected;
        
        // Normalize bearing innovation to [-pi, pi]
        innovation(1) = normalizeAngle(innovation(1));
        
        // 6. Compute measurement Jacobian
        Eigen::Matrix<double, 2, 3> H_robot;
        Eigen::Matrix<double, 2, 2> H_landmark;
        measurementJacobian(robot_pose, landmark_pos, H_robot, H_landmark);
        
        // 7. Build full Jacobian H (2 x state_size)
        int state_size = mu_.size();
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, state_size);
        H.block<2, 3>(0, 0) = H_robot;           // Robot part
        H.block<2, 2>(0, lm_idx) = H_landmark;   // Landmark part
        
        // 8. Measurement noise covariance
        Eigen::Matrix2d Q;
        Q << sensor_noise_range_ * sensor_noise_range_, 0,
             0, sensor_noise_bearing_ * sensor_noise_bearing_;
        
        // 9. Compute Kalman gain
        // K = P * H^T * (H * P * H^T + Q)^(-1)
        Eigen::Matrix2d S = H * P_ * H.transpose() + Q;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        // 10. Update state estimate
        mu_ = mu_ + K * innovation;
        
        // Normalize robot orientation
        mu_(2) = normalizeAngle(mu_(2));
        
        // 11. Update covariance
        // P = (I - K * H) * P
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_size, state_size);
        P_ = (I - K * H) * P_;
    }
}

// ============================================================================
// Motion Model
// ============================================================================

Eigen::Vector3d EkfSlam::motionModel(const Eigen::Vector3d& pose,
                                     const MotionCommand& motion) {
    // Simple velocity motion model
    // x' = x + v * cos(theta) * dt
    // y' = y + v * sin(theta) * dt
    // theta' = theta + omega * dt
    
    double x = pose(0);
    double y = pose(1);
    double theta = pose(2);
    
    double v = motion.linear_velocity;
    double w = motion.angular_velocity;
    double dt = motion.dt;
    
    Eigen::Vector3d new_pose;
    new_pose(0) = x + v * std::cos(theta) * dt;
    new_pose(1) = y + v * std::sin(theta) * dt;
    new_pose(2) = normalizeAngle(theta + w * dt);
    
    return new_pose;
}

Eigen::Matrix3d EkfSlam::motionJacobian(const Eigen::Vector3d& pose,
                                        const MotionCommand& motion) {
    // Jacobian of motion model w.r.t. robot pose
    // G = d(motion_model)/d(x, y, theta)
    
    double theta = pose(2);
    double v = motion.linear_velocity;
    double dt = motion.dt;
    
    Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
    
    // Partial derivatives
    G(0, 2) = -v * std::sin(theta) * dt;  // dx/dtheta
    G(1, 2) = v * std::cos(theta) * dt;   // dy/dtheta
    
    return G;
}

// ============================================================================
// Measurement Model
// ============================================================================

Eigen::Vector2d EkfSlam::measurementModel(const Eigen::Vector3d& robot_pose,
                                          const Eigen::Vector2d& landmark_pos) {
    // Compute expected observation of landmark from robot pose
    // Returns [range, bearing]
    
    double dx = landmark_pos(0) - robot_pose(0);
    double dy = landmark_pos(1) - robot_pose(1);
    
    double range = std::sqrt(dx * dx + dy * dy);
    double bearing_world = std::atan2(dy, dx);
    double bearing_robot = normalizeAngle(bearing_world - robot_pose(2));
    
    Eigen::Vector2d z;
    z << range, bearing_robot;
    return z;
}

void EkfSlam::measurementJacobian(const Eigen::Vector3d& robot_pose,
                                  const Eigen::Vector2d& landmark_pos,
                                  Eigen::Matrix<double, 2, 3>& H_robot,
                                  Eigen::Matrix<double, 2, 2>& H_landmark) {
    // Compute Jacobian of measurement model
    
    double dx = landmark_pos(0) - robot_pose(0);
    double dy = landmark_pos(1) - robot_pose(1);
    double q = dx * dx + dy * dy;
    double sqrt_q = std::sqrt(q);
    
    // Jacobian w.r.t. robot pose [x, y, theta]
    H_robot << -dx / sqrt_q,  -dy / sqrt_q,  0,
               dy / q,        -dx / q,       -1;
    
    // Jacobian w.r.t. landmark position [lm_x, lm_y]
    H_landmark << dx / sqrt_q,  dy / sqrt_q,
                  -dy / q,      dx / q;
}

// ============================================================================
// Landmark Management
// ============================================================================

int EkfSlam::addLandmark(const LandmarkObservation& observation) {
    // Initialize new landmark from observation
    
    Eigen::Vector3d robot_pose = mu_.head<3>();
    
    // Convert observation to global coordinates
    double lm_x = robot_pose(0) + observation.range * std::cos(robot_pose(2) + observation.bearing);
    double lm_y = robot_pose(1) + observation.range * std::sin(robot_pose(2) + observation.bearing);
    
    // Assign new landmark ID
    int landmark_id = next_landmark_id_++;
    
    // Store mapping from landmark ID to state index
    int state_idx = mu_.size();
    landmark_to_state_[landmark_id] = state_idx;
    
    // Expand state vector
    int old_size = mu_.size();
    mu_.conservativeResize(old_size + 2);
    mu_(old_size) = lm_x;
    mu_(old_size + 1) = lm_y;
    
    // Expand covariance matrix
    P_.conservativeResize(old_size + 2, old_size + 2);
    
    // Initialize new landmark covariance (high uncertainty)
    P_.block<2, 2>(old_size, old_size) = Eigen::Matrix2d::Identity() * 10.0;
    
    // Initialize cross-covariances
    P_.block(0, old_size, old_size, 2).setZero();
    P_.block(old_size, 0, 2, old_size).setZero();
    
    return landmark_id;
}

int EkfSlam::dataAssociation(const LandmarkObservation& observation) {
    // Simple nearest-neighbor data association
    // In practice, should use Mahalanobis distance with gating
    
    if (landmark_to_state_.empty()) {
        return -1;  // No landmarks yet
    }
    
    Eigen::Vector3d robot_pose = mu_.head<3>();
    double min_distance = std::numeric_limits<double>::max();
    int best_match = -1;
    
    // Check each known landmark
    for (const auto& [landmark_id, state_idx] : landmark_to_state_) {
        Eigen::Vector2d landmark_pos = mu_.segment<2>(state_idx);
        
        // Compute expected observation
        Eigen::Vector2d z_expected = measurementModel(robot_pose, landmark_pos);
        
        // Compute innovation
        Eigen::Vector2d innovation;
        innovation << observation.range - z_expected(0),
                     normalizeAngle(observation.bearing - z_expected(1));
        
        // Simple Euclidean distance (should use Mahalanobis)
        double distance = innovation.norm();
        
        if (distance < min_distance) {
            min_distance = distance;
            best_match = landmark_id;
        }
    }
    
    // Gating threshold: if too far, treat as new landmark
    const double GATE_THRESHOLD = 1.0;  // meters + radians combined
    if (min_distance > GATE_THRESHOLD) {
        return -1;
    }
    
    return best_match;
}

// ============================================================================
// Utility Functions
// ============================================================================

Eigen::Vector3d EkfSlam::getRobotPose() const {
    return mu_.head<3>();
}

std::unordered_map<int, Landmark> EkfSlam::getLandmarks() const {
    std::unordered_map<int, Landmark> landmarks;
    
    for (const auto& [landmark_id, state_idx] : landmark_to_state_) {
        Landmark lm;
        lm.id = landmark_id;
        lm.x = mu_(state_idx);
        lm.y = mu_(state_idx + 1);
        lm.initialized = true;
        landmarks[landmark_id] = lm;
    }
    
    return landmarks;
}

void EkfSlam::reset() {
    mu_ = Eigen::VectorXd::Zero(3);
    P_ = Eigen::MatrixXd::Identity(3, 3) * 0.1;
    landmark_to_state_.clear();
    next_landmark_id_ = 0;
}

double EkfSlam::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

} // namespace ekf_slam
