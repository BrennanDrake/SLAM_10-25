/**
 * @file fake_scan_publisher.cpp
 * @brief Fake laser scan publisher for testing occupancy grid generation
 * @author Brennan Drake
 */

#include <memory>
#include <vector>
#include <cmath>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/tf2_ros/transform_broadcaster.hpp"

/**
 * @brief Simple fake laser scan publisher for testing occupancy grid generation
 * 
 * Publishes simulated laser scan data and robot odometry for testing purposes.
 * Creates a simple rectangular room environment with some obstacles.
 */
class FakeScanPublisher : public rclcpp::Node
{
public:
  FakeScanPublisher() : Node("fake_scan_publisher"), robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0)
  {
    // Create publisher for laser scan
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    
    // Create TF broadcaster for robot pose
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Create timer for publishing
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz
      std::bind(&FakeScanPublisher::publish_scan, this)
    );
    
    // Initialize random number generator
    rng_.seed(std::chrono::steady_clock::now().time_since_epoch().count());
    
    RCLCPP_INFO(this->get_logger(), "Fake scan publisher started");
  }

private:
  void publish_scan()
  {
    // Update robot position (simple circular motion)
    double dt = 0.1;  // 100ms
    double linear_vel = 0.2;  // m/s
    double angular_vel = 0.1; // rad/s
    
    robot_x_ += linear_vel * std::cos(robot_theta_) * dt;
    robot_y_ += linear_vel * std::sin(robot_theta_) * dt;
    robot_theta_ += angular_vel * dt;
    
    // Keep robot in bounds
    if (robot_x_ > 8.0 || robot_x_ < -8.0 || robot_y_ > 8.0 || robot_y_ < -8.0) {
      robot_theta_ += M_PI;  // Turn around
    }
    
    // Publish TF transform
    publish_transform();
    
    // Create laser scan message
    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan_msg->header.stamp = this->get_clock()->now();
    scan_msg->header.frame_id = "base_scan";
    
    scan_msg->angle_min = -M_PI;
    scan_msg->angle_max = M_PI;
    scan_msg->angle_increment = M_PI / 180.0;  // 1 degree resolution
    scan_msg->time_increment = 0.0;
    scan_msg->scan_time = 0.1;
    scan_msg->range_min = 0.12;
    scan_msg->range_max = 12.0;
    
    // Generate scan data for a simple environment
    int num_readings = static_cast<int>((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
    scan_msg->ranges.resize(num_readings);
    scan_msg->intensities.resize(num_readings);
    
    for (int i = 0; i < num_readings; ++i) {
      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      double global_angle = robot_theta_ + angle;
      
      // Simulate a rectangular room with some obstacles
      double range = simulate_range(global_angle);
      
      // Add some noise
      std::normal_distribution<double> noise(0.0, 0.01);
      range += noise(rng_);
      
      // Clamp to valid range
      range = std::max(static_cast<double>(scan_msg->range_min), std::min(static_cast<double>(scan_msg->range_max), range));
      
      scan_msg->ranges[i] = range;
      scan_msg->intensities[i] = 100.0;  // Arbitrary intensity
    }
    
    scan_pub_->publish(std::move(scan_msg));
  }
  
  void publish_transform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    
    transform.transform.translation.x = robot_x_;
    transform.transform.translation.y = robot_y_;
    transform.transform.translation.z = 0.0;
    
    // Convert yaw to quaternion
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = std::sin(robot_theta_ / 2.0);
    transform.transform.rotation.w = std::cos(robot_theta_ / 2.0);
    
    tf_broadcaster_->sendTransform(transform);
    
    // Also publish base_link to base_scan transform
    geometry_msgs::msg::TransformStamped scan_transform;
    scan_transform.header.stamp = this->get_clock()->now();
    scan_transform.header.frame_id = "base_link";
    scan_transform.child_frame_id = "base_scan";
    scan_transform.transform.translation.x = 0.0;
    scan_transform.transform.translation.y = 0.0;
    scan_transform.transform.translation.z = 0.2;  // Laser height
    scan_transform.transform.rotation.w = 1.0;
    
    tf_broadcaster_->sendTransform(scan_transform);
  }
  
  /**
   * @brief Simulate laser range measurement for a simple environment
   * @param angle Global angle of the laser ray
   * @return Simulated range measurement
   */
  double simulate_range(double angle)
  {
    double max_range = 12.0;
    
    // Room boundaries (10m x 10m room)
    double room_size = 10.0;
    
    // Calculate intersection with room walls
    double dx = std::cos(angle);
    double dy = std::sin(angle);
    
    double range_to_wall = max_range;
    
    // Check intersection with each wall
    if (dx > 0) {  // Right wall
      double t = (room_size/2 - robot_x_) / dx;
      if (t > 0) {
        double y_intersect = robot_y_ + t * dy;
        if (std::abs(y_intersect) <= room_size/2) {
          range_to_wall = std::min(range_to_wall, t);
        }
      }
    } else if (dx < 0) {  // Left wall
      double t = (-room_size/2 - robot_x_) / dx;
      if (t > 0) {
        double y_intersect = robot_y_ + t * dy;
        if (std::abs(y_intersect) <= room_size/2) {
          range_to_wall = std::min(range_to_wall, t);
        }
      }
    }
    
    if (dy > 0) {  // Top wall
      double t = (room_size/2 - robot_y_) / dy;
      if (t > 0) {
        double x_intersect = robot_x_ + t * dx;
        if (std::abs(x_intersect) <= room_size/2) {
          range_to_wall = std::min(range_to_wall, t);
        }
      }
    } else if (dy < 0) {  // Bottom wall
      double t = (-room_size/2 - robot_y_) / dy;
      if (t > 0) {
        double x_intersect = robot_x_ + t * dx;
        if (std::abs(x_intersect) <= room_size/2) {
          range_to_wall = std::min(range_to_wall, t);
        }
      }
    }
    
    // Add some simple obstacles (cylinders)
    std::vector<std::pair<double, double>> obstacles = {{2.0, 2.0}, {-3.0, 1.0}, {1.0, -3.0}};
    double obstacle_radius = 0.5;
    
    for (const auto& obs : obstacles) {
      double obs_x = obs.first;
      double obs_y = obs.second;
      
      // Vector from robot to obstacle center
      double to_obs_x = obs_x - robot_x_;
      double to_obs_y = obs_y - robot_y_;
      
      // Project onto ray direction
      double proj = to_obs_x * dx + to_obs_y * dy;
      
      if (proj > 0) {  // Obstacle is in front of robot
        // Find closest point on ray to obstacle center
        double closest_x = robot_x_ + proj * dx;
        double closest_y = robot_y_ + proj * dy;
        
        // Distance from obstacle center to ray
        double dist_to_ray = std::sqrt(std::pow(closest_x - obs_x, 2) + std::pow(closest_y - obs_y, 2));
        
        if (dist_to_ray <= obstacle_radius) {
          // Ray intersects obstacle
          // double dist_to_center = std::sqrt(to_obs_x * to_obs_x + to_obs_y * to_obs_y);
          double intersection_dist = proj - std::sqrt(obstacle_radius * obstacle_radius - dist_to_ray * dist_to_ray);
          if (intersection_dist > 0) {
            range_to_wall = std::min(range_to_wall, intersection_dist);
          }
        }
      }
    }
    
    return range_to_wall;
  }
  
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Robot state
  double robot_x_, robot_y_, robot_theta_;
  
  // Random number generator for noise
  std::mt19937 rng_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeScanPublisher>());
  rclcpp::shutdown();
  return 0;
}
