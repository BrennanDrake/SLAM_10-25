#ifndef HECTOR_SLAM_CUSTOM__EKF_FILTER_HPP_
#define HECTOR_SLAM_CUSTOM__EKF_FILTER_HPP_

#include <Eigen/Dense>
#include <cmath>

namespace hector_slam_custom {

// Simple EKF for 2D pose [x, y, theta]
class EkfFilter {
public:
    EkfFilter();

    // Initialize state and covariance
    void initialize(const Eigen::Vector3d &x0, const Eigen::Matrix3d &P0);

    // Predict step using linear velocity v (m/s), angular velocity omega (rad/s), and dt (s)
    void predict(double v, double omega, double dt);

    // Update step with absolute pose measurement z = [x, y, theta]
    // R is measurement covariance (3x3)
    void update(const Eigen::Vector3d &z, const Eigen::Matrix3d &R);

    // Accessors
    const Eigen::Vector3d &getState() const { return x_; }
    const Eigen::Matrix3d &getCovariance() const { return P_; }

    // Tuning (process noise)
    void setProcessNoise(double q_pos, double q_theta);

private:
    // Normalize angle to [-pi, pi]
    static double normalizeAngle(double a);

    Eigen::Vector3d x_;   // [x, y, theta]
    Eigen::Matrix3d P_;   // state covariance
    Eigen::Matrix3d Q_;   // process noise covariance
};

} // namespace hector_slam_custom

#endif // HECTOR_SLAM_CUSTOM__EKF_FILTER_HPP_
