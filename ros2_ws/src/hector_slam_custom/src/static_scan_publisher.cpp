/**
 * @file static_scan_publisher.cpp
 * @brief Publishes static laser scans for testing SLAM
 * 
 * Simulates a robot in a simple rectangular room with consistent walls.
 * This provides repeatable test data for verifying scan matching.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>
#include <random>
#include <limits>

class StaticScanPublisher : public rclcpp::Node {
public:
    StaticScanPublisher() : Node("static_scan_publisher") {
        // Declare parameters
        this->declare_parameter("scan_topic", "scan");
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("room_width", 10.0);   // meters
        this->declare_parameter("room_height", 8.0);   // meters
        this->declare_parameter("robot_x", 0.0);       // robot position
        this->declare_parameter("robot_y", 0.0);
        this->declare_parameter("robot_theta", 0.0);   // robot orientation
        this->declare_parameter("add_noise", false);
        this->declare_parameter("noise_stddev", 0.01);
        
        // Get parameters
        scan_topic_ = this->get_parameter("scan_topic").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        room_width_ = this->get_parameter("room_width").as_double();
        room_height_ = this->get_parameter("room_height").as_double();
        robot_x_ = this->get_parameter("robot_x").as_double();
        robot_y_ = this->get_parameter("robot_y").as_double();
        robot_theta_ = this->get_parameter("robot_theta").as_double();
        add_noise_ = this->get_parameter("add_noise").as_bool();
        noise_stddev_ = this->get_parameter("noise_stddev").as_double();
        
        // Create publisher
        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10);
        
        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate_),
            std::bind(&StaticScanPublisher::publishScan, this));
        
        // Initialize random generator for noise
        if (add_noise_) {
            std::random_device rd;
            rng_ = std::mt19937(rd());
            noise_dist_ = std::normal_distribution<double>(0.0, noise_stddev_);
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "Static scan publisher started: room %.1fx%.1fm, robot at (%.1f, %.1f)",
                   room_width_, room_height_, robot_x_, robot_y_);
    }

private:
    void publishScan() {
        auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        
        // Configure scan parameters
        scan_msg->header.stamp = this->now();
        scan_msg->header.frame_id = "laser";
        scan_msg->angle_min = -M_PI;        // -180 degrees
        scan_msg->angle_max = M_PI;         // +180 degrees
        scan_msg->angle_increment = M_PI / 180.0;  // 1 degree resolution
        scan_msg->time_increment = 0.0;
        scan_msg->scan_time = 1.0 / publish_rate_;
        scan_msg->range_min = 0.1;
        scan_msg->range_max = 20.0;
        
        // Generate scan data - simulate rectangular room
        int num_readings = static_cast<int>(
            (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment) + 1;
        scan_msg->ranges.reserve(num_readings);
        
        for (int i = 0; i < num_readings; ++i) {
            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            double world_angle = angle + robot_theta_;
            
            // Calculate range to walls using ray-box intersection
            double range = calculateRangeToWall(world_angle);
            
            // Add noise if requested
            if (add_noise_ && range < scan_msg->range_max) {
                range += noise_dist_(rng_);
                range = std::max(static_cast<double>(scan_msg->range_min), 
                                std::min(range, static_cast<double>(scan_msg->range_max)));
            }
            
            scan_msg->ranges.push_back(range);
        }
        
        // Publish scan
        scan_publisher_->publish(std::move(scan_msg));
    }
    
    double calculateRangeToWall(double angle) {
        // Room boundaries (centered at origin)
        double x_min = -room_width_ / 2.0;
        double x_max = room_width_ / 2.0;
        double y_min = -room_height_ / 2.0;
        double y_max = room_height_ / 2.0;
        
        // Ray direction
        double dx = std::cos(angle);
        double dy = std::sin(angle);
        
        // Calculate intersections with all four walls
        double t_min = std::numeric_limits<double>::max();
        
        // Check intersection with left wall (x = x_min)
        if (dx < 0) {
            double t = (x_min - robot_x_) / dx;
            double y = robot_y_ + t * dy;
            if (t > 0 && y >= y_min && y <= y_max) {
                t_min = std::min(t_min, t);
            }
        }
        
        // Check intersection with right wall (x = x_max)
        if (dx > 0) {
            double t = (x_max - robot_x_) / dx;
            double y = robot_y_ + t * dy;
            if (t > 0 && y >= y_min && y <= y_max) {
                t_min = std::min(t_min, t);
            }
        }
        
        // Check intersection with bottom wall (y = y_min)
        if (dy < 0) {
            double t = (y_min - robot_y_) / dy;
            double x = robot_x_ + t * dx;
            if (t > 0 && x >= x_min && x <= x_max) {
                t_min = std::min(t_min, t);
            }
        }
        
        // Check intersection with top wall (y = y_max)
        if (dy > 0) {
            double t = (y_max - robot_y_) / dy;
            double x = robot_x_ + t * dx;
            if (t > 0 && x >= x_min && x <= x_max) {
                t_min = std::min(t_min, t);
            }
        }
        
        // Return distance to nearest wall
        return (t_min < std::numeric_limits<double>::max()) ? t_min : 20.0;
    }
    
    // Member variables
    std::string scan_topic_;
    double publish_rate_;
    double room_width_;
    double room_height_;
    double robot_x_;
    double robot_y_;
    double robot_theta_;
    bool add_noise_;
    double noise_stddev_;
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Random number generation for noise
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticScanPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
