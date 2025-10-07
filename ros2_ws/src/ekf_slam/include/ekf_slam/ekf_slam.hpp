/**
 * @file ekf_slam.hpp
 * @brief Extended Kalman Filter SLAM implementation
 * 
 * This implements landmark-based SLAM using EKF.
 * State vector: [robot_x, robot_y, robot_theta, landmark_1_x, landmark_1_y, ...]
 * 
 * Key differences from Hector SLAM:
 * - Tracks specific landmarks instead of occupancy grid
 * - Maintains full covariance matrix for robot and all landmarks
 * - Handles data association problem
 */

#ifndef EKF_SLAM__EKF_SLAM_HPP_
#define EKF_SLAM__EKF_SLAM_HPP_

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <unordered_map>

namespace ekf_slam {

/**
 * @brief 2D landmark representation
 */
struct Landmark {
    int id;
    double x;
    double y;
    bool initialized;
    int observations;  // Number of times observed
    
    Landmark() : id(-1), x(0), y(0), initialized(false), observations(0) {}
    Landmark(int id_, double x_, double y_) 
        : id(id_), x(x_), y(y_), initialized(true), observations(1) {}
};

/**
 * @brief Landmark observation from sensor
 */
struct LandmarkObservation {
    double range;      // Distance to landmark
    double bearing;    // Angle to landmark (in robot frame)
    int id;           // Landmark ID (-1 if unknown)
    
    LandmarkObservation(double r, double b, int i = -1) 
        : range(r), bearing(b), id(i) {}
};

/**
 * @brief Motion command (control input)
 */
struct MotionCommand {
    double linear_velocity;   // v (m/s)
    double angular_velocity;  // omega (rad/s)
    double dt;               // Time step
    
    MotionCommand(double v = 0, double w = 0, double t = 0.1) 
        : linear_velocity(v), angular_velocity(w), dt(t) {}
};

/**
 * @brief EKF-SLAM implementation
 * 
 * State vector organization:
 * [0:2] - Robot pose (x, y, theta)
 * [3:4] - First landmark (x, y)
 * [5:6] - Second landmark (x, y)
 * ... and so on
 */
class EkfSlam {
public:
    /**
     * @brief Constructor
     * @param motion_noise_v Linear velocity noise std dev
     * @param motion_noise_w Angular velocity noise std dev
     * @param sensor_noise_range Range measurement noise std dev
     * @param sensor_noise_bearing Bearing measurement noise std dev
     */
    EkfSlam(double motion_noise_v = 0.1,
            double motion_noise_w = 0.1,
            double sensor_noise_range = 0.1,
            double sensor_noise_bearing = 0.05);
    
    /**
     * @brief Destructor
     */
    ~EkfSlam() = default;
    
    /**
     * @brief Prediction step - update robot pose based on motion
     * @param motion Motion command
     */
    void predict(const MotionCommand& motion);
    
    /**
     * @brief Correction step - update state based on landmark observations
     * @param observations Vector of landmark observations
     */
    void correct(const std::vector<LandmarkObservation>& observations);
    
    /**
     * @brief Get current robot pose estimate
     * @return Robot pose [x, y, theta]
     */
    Eigen::Vector3d getRobotPose() const;
    
    /**
     * @brief Get all landmarks
     * @return Map of landmark ID to landmark
     */
    std::unordered_map<int, Landmark> getLandmarks() const;
    
    /**
     * @brief Get state covariance matrix
     * @return Covariance matrix
     */
    Eigen::MatrixXd getCovariance() const { return P_; }
    
    /**
     * @brief Reset the filter
     */
    void reset();
    
private:
    // State vector and covariance
    Eigen::VectorXd mu_;      // State estimate [robot_pose, landmarks...]
    Eigen::MatrixXd P_;       // Covariance matrix
    
    // Noise parameters
    double motion_noise_v_;      // Linear velocity noise
    double motion_noise_w_;      // Angular velocity noise  
    double sensor_noise_range_;  // Range measurement noise
    double sensor_noise_bearing_; // Bearing measurement noise
    
    // Landmark management
    std::unordered_map<int, int> landmark_to_state_; // Maps landmark ID to state index
    int next_landmark_id_;
    
    /**
     * @brief Motion model - predict new robot pose
     * @param motion Motion command
     * @return Predicted pose
     */
    Eigen::Vector3d motionModel(const Eigen::Vector3d& pose, 
                                const MotionCommand& motion);
    
    /**
     * @brief Compute Jacobian of motion model w.r.t. pose
     * @param pose Current pose
     * @param motion Motion command
     * @return 3x3 Jacobian matrix
     */
    Eigen::Matrix3d motionJacobian(const Eigen::Vector3d& pose,
                                   const MotionCommand& motion);
    
    /**
     * @brief Measurement model - expected observation of landmark
     * @param robot_pose Robot pose
     * @param landmark_pos Landmark position
     * @return Expected [range, bearing]
     */
    Eigen::Vector2d measurementModel(const Eigen::Vector3d& robot_pose,
                                     const Eigen::Vector2d& landmark_pos);
    
    /**
     * @brief Compute Jacobian of measurement model
     * @param robot_pose Robot pose
     * @param landmark_pos Landmark position
     * @param H_robot Output: Jacobian w.r.t. robot pose (2x3)
     * @param H_landmark Output: Jacobian w.r.t. landmark (2x2)
     */
    void measurementJacobian(const Eigen::Vector3d& robot_pose,
                            const Eigen::Vector2d& landmark_pos,
                            Eigen::Matrix<double, 2, 3>& H_robot,
                            Eigen::Matrix<double, 2, 2>& H_landmark);
    
    /**
     * @brief Add new landmark to state
     * @param observation Observation of new landmark
     * @return Landmark ID
     */
    int addLandmark(const LandmarkObservation& observation);
    
    /**
     * @brief Data association - match observation to landmark
     * @param observation Observation to match
     * @return Landmark ID (-1 if new landmark)
     */
    int dataAssociation(const LandmarkObservation& observation);
    
    /**
     * @brief Normalize angle to [-pi, pi]
     * @param angle Angle in radians
     * @return Normalized angle
     */
    double normalizeAngle(double angle);
};

} // namespace ekf_slam

#endif // EKF_SLAM__EKF_SLAM_HPP_
