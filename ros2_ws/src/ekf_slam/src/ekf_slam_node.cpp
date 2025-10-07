/**
 * @file ekf_slam_node.cpp
 * @brief ROS 2 node demonstrating EKF-SLAM with simulated landmarks
 * 
 * This node:
 * 1. Creates a simulated environment with known landmarks
 * 2. Simulates robot motion
 * 3. Generates landmark observations
 * 4. Runs EKF-SLAM algorithm
 * 5. Publishes results for visualization
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>

#include "ekf_slam/ekf_slam.hpp"
#include "ekf_slam/landmark_extractor.hpp"

class EkfSlamNode : public rclcpp::Node {
public:
    EkfSlamNode() : Node("ekf_slam_node") {
        // Declare parameters
        this->declare_parameter("update_rate", 10.0);
        this->declare_parameter("motion_noise_v", 0.1);
        this->declare_parameter("motion_noise_w", 0.1);
        this->declare_parameter("sensor_noise_range", 0.1);
        this->declare_parameter("sensor_noise_bearing", 0.05);
        
        double update_rate = this->get_parameter("update_rate").as_double();
        double motion_noise_v = this->get_parameter("motion_noise_v").as_double();
        double motion_noise_w = this->get_parameter("motion_noise_w").as_double();
        double sensor_noise_range = this->get_parameter("sensor_noise_range").as_double();
        double sensor_noise_bearing = this->get_parameter("sensor_noise_bearing").as_double();
        
        // Initialize EKF-SLAM
        ekf_ = std::make_unique<ekf_slam::EkfSlam>(
            motion_noise_v, motion_noise_w,
            sensor_noise_range, sensor_noise_bearing);
        
        // Initialize landmark simulator
        simulator_ = std::make_unique<ekf_slam::LandmarkSimulator>();
        
        // Create simulated environment with landmarks
        // Place landmarks in a square pattern
        simulator_->addLandmark(5.0, 5.0, 0);    // Top-right
        simulator_->addLandmark(-5.0, 5.0, 1);   // Top-left
        simulator_->addLandmark(-5.0, -5.0, 2);  // Bottom-left
        simulator_->addLandmark(5.0, -5.0, 3);   // Bottom-right
        simulator_->addLandmark(0.0, 0.0, 4);    // Center
        
        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "ekf_pose", 10);
        true_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "true_pose", 10);
        landmarks_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "landmarks", 10);
        true_landmarks_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "true_landmarks", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "ekf_path", 10);
        
        // Timer for simulation updates
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / update_rate),
            std::bind(&EkfSlamNode::update, this));
        
        // Initialize true robot pose
        true_x_ = 0.0;
        true_y_ = 0.0;
        true_theta_ = 0.0;
        
        // Simulation time
        sim_time_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "EKF-SLAM node started");
        RCLCPP_INFO(this->get_logger(), "Simulating robot motion in environment with 5 landmarks");
    }
    
private:
    void update() {
        double dt = 0.1;  // 10 Hz update rate
        
        // 1. Generate motion command (circular motion)
        double v = 0.5;  // 0.5 m/s linear velocity
        double w = 0.2;  // 0.2 rad/s angular velocity
        
        ekf_slam::MotionCommand motion(v, w, dt);
        
        // 2. Update true robot pose (ground truth)
        true_x_ += v * std::cos(true_theta_) * dt;
        true_y_ += v * std::sin(true_theta_) * dt;
        true_theta_ += w * dt;
        
        // Normalize angle
        while (true_theta_ > M_PI) true_theta_ -= 2.0 * M_PI;
        while (true_theta_ < -M_PI) true_theta_ += 2.0 * M_PI;
        
        // 3. EKF Prediction step
        ekf_->predict(motion);
        
        // 4. Generate landmark observations from true pose
        auto observations = simulator_->simulateObservations(
            true_x_, true_y_, true_theta_,
            10.0,  // max range
            M_PI,  // FOV
            0.1,   // range noise
            0.05); // bearing noise
        
        // 5. EKF Correction step
        if (!observations.empty()) {
            ekf_->correct(observations);
        }
        
        // 6. Publish results
        publishPose();
        publishTruePose();
        publishLandmarks();
        publishTrueLandmarks();
        publishPath();
        
        sim_time_ += dt;
        
        // Log status every 2 seconds
        if (static_cast<int>(sim_time_ * 10) % 20 == 0) {
            auto pose = ekf_->getRobotPose();
            auto landmarks = ekf_->getLandmarks();
            RCLCPP_INFO(this->get_logger(),
                       "Time: %.1fs | Est: (%.2f, %.2f, %.2f) | True: (%.2f, %.2f, %.2f) | Landmarks: %zu",
                       sim_time_, pose(0), pose(1), pose(2),
                       true_x_, true_y_, true_theta_,
                       landmarks.size());
        }
    }
    
    void publishPose() {
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        
        auto pose = ekf_->getRobotPose();
        pose_msg.pose.pose.position.x = pose(0);
        pose_msg.pose.pose.position.y = pose(1);
        pose_msg.pose.pose.position.z = 0.0;
        
        // Convert theta to quaternion
        pose_msg.pose.pose.orientation.w = std::cos(pose(2) / 2.0);
        pose_msg.pose.pose.orientation.z = std::sin(pose(2) / 2.0);
        
        // Get covariance
        auto cov = ekf_->getCovariance();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                pose_msg.pose.covariance[i * 6 + j] = cov(i, j);
            }
        }
        
        pose_pub_->publish(pose_msg);
    }
    
    void publishTruePose() {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        
        pose_msg.pose.position.x = true_x_;
        pose_msg.pose.position.y = true_y_;
        pose_msg.pose.position.z = 0.0;
        
        pose_msg.pose.orientation.w = std::cos(true_theta_ / 2.0);
        pose_msg.pose.orientation.z = std::sin(true_theta_ / 2.0);
        
        true_pose_pub_->publish(pose_msg);
    }
    
    void publishLandmarks() {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        auto landmarks = ekf_->getLandmarks();
        
        for (const auto& [id, lm] : landmarks) {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = this->now();
            marker.header.frame_id = "map";
            marker.ns = "estimated_landmarks";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = lm.x;
            marker.pose.position.y = lm.y;
            marker.pose.position.z = 0.0;
            
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
            
            marker_array.markers.push_back(marker);
        }
        
        landmarks_pub_->publish(marker_array);
    }
    
    void publishTrueLandmarks() {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        auto landmarks = simulator_->getLandmarks();
        
        for (const auto& lm : landmarks) {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = this->now();
            marker.header.frame_id = "map";
            marker.ns = "true_landmarks";
            marker.id = lm.id;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = lm.x;
            marker.pose.position.y = lm.y;
            marker.pose.position.z = 0.0;
            
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.5;
            
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            
            marker_array.markers.push_back(marker);
        }
        
        true_landmarks_pub_->publish(marker_array);
    }
    
    void publishPath() {
        auto pose = ekf_->getRobotPose();
        
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = pose(0);
        pose_stamped.pose.position.y = pose(1);
        
        path_.header.stamp = this->now();
        path_.header.frame_id = "map";
        path_.poses.push_back(pose_stamped);
        
        path_pub_->publish(path_);
    }
    
    // EKF-SLAM components
    std::unique_ptr<ekf_slam::EkfSlam> ekf_;
    std::unique_ptr<ekf_slam::LandmarkSimulator> simulator_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr true_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmarks_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr true_landmarks_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // True robot pose (ground truth)
    double true_x_, true_y_, true_theta_;
    
    // Path for visualization
    nav_msgs::msg::Path path_;
    
    // Simulation time
    double sim_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EkfSlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
