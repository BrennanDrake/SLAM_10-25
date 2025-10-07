#include "hector_slam_custom/ekf_filter.hpp"

namespace hector_slam_custom {

EkfFilter::EkfFilter() {
    x_.setZero();
    P_.setIdentity();
    Q_.setZero();
}

void EkfFilter::initialize(const Eigen::Vector3d &x0, const Eigen::Matrix3d &P0) {
    x_ = x0;
    P_ = P0;
}

void EkfFilter::setProcessNoise(double q_pos, double q_theta) {
    Q_.setZero();
    Q_(0,0) = q_pos;
    Q_(1,1) = q_pos;
    Q_(2,2) = q_theta;
}

void EkfFilter::predict(double v, double omega, double dt) {
    if (dt <= 0.0) return;

    const double c = std::cos(x_(2));
    const double s = std::sin(x_(2));

    // Nonlinear state update
    x_(0) += v * c * dt;
    x_(1) += v * s * dt;
    x_(2) = normalizeAngle(x_(2) + omega * dt);

    // Jacobian of motion model w.r.t state
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -v * s * dt;
    F(1,2) =  v * c * dt;

    // Simple additive process noise in state space
    P_ = F * P_ * F.transpose() + Q_;
}

void EkfFilter::update(const Eigen::Vector3d &z, const Eigen::Matrix3d &R) {
    // Measurement model: identity (we directly measure x, y, theta in map frame)
    Eigen::Vector3d y = z - x_;
    // Normalize angle residual
    y(2) = normalizeAngle(y(2));

    Eigen::Matrix3d S = P_ + R;                 // innovation covariance
    Eigen::Matrix3d K = P_ * S.inverse();       // Kalman gain

    x_ = x_ + K * y;                            // state update
    x_(2) = normalizeAngle(x_(2));

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    P_ = (I - K) * P_;                          // covariance update
}

double EkfFilter::normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

} // namespace hector_slam_custom
