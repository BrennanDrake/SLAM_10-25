/**
 * @file hector_slam_node.cpp
 * @brief Main executable for Hector SLAM node
 * 
 * This creates the ROS 2 node that runs our SLAM system.
 * It's the entry point that gets launched.
 */

#include "hector_slam_custom/hector_slam_processor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create the SLAM processor node
    auto slam_node = std::make_shared<hector_slam_custom::HectorSlamProcessor>();
    
    // Log startup
    RCLCPP_INFO(slam_node->get_logger(), 
                "Hector SLAM node started. Waiting for laser scans on 'scan' topic...");
    
    // Spin the node (process callbacks)
    // This runs until shutdown is requested (Ctrl+C)
    rclcpp::spin(slam_node);
    
    // Cleanup
    rclcpp::shutdown();
    
    return 0;
}
