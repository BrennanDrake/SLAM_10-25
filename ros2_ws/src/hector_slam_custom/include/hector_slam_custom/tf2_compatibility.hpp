/**
 * @file tf2_compatibility.hpp
 * @brief Compatibility layer for TF2 in ROS 2 Jazzy
 * @author Brennan Drake
 * 
 * This header works around the nested directory structure issues in ROS 2 Jazzy
 * where TF2 headers are located at tf2_ros/tf2_ros/ instead of tf2_ros/
 */

#ifndef HECTOR_SLAM_CUSTOM__TF2_COMPATIBILITY_HPP_
#define HECTOR_SLAM_CUSTOM__TF2_COMPATIBILITY_HPP_

// =============================================================================
// COMPATIBILITY INCLUDES
// =============================================================================
// These are the problematic includes that we need to handle carefully

#ifdef ROS_JAZZY_TF2_WORKAROUND

// Direct includes with full paths to avoid nested directory issues
#include "/opt/ros/jazzy/include/tf2/tf2/buffer_core.hpp"
#include "/opt/ros/jazzy/include/tf2_ros/tf2_ros/transform_listener.hpp"
#include "/opt/ros/jazzy/include/tf2_ros/tf2_ros/buffer.hpp"
#include "/opt/ros/jazzy/include/tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#else

// Standard includes (for other ROS distributions or when fixed)
#include "tf2/buffer_core.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#endif

// Standard ROS 2 includes that work fine
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// =============================================================================
// COMPATIBILITY NAMESPACE
// =============================================================================
namespace tf2_compat {

/**
 * @brief Wrapper for TF2 Buffer with simplified interface
 */
class Buffer {
public:
    explicit Buffer(rclcpp::Clock::SharedPtr clock, 
                   const rclcpp::Duration& cache_time = rclcpp::Duration::from_nanoseconds(10000000000ULL))
        : buffer_(clock, cache_time) {}

    /**
     * @brief Look up transform between two frames
     * @param target_frame Target coordinate frame
     * @param source_frame Source coordinate frame  
     * @param time Time at which to get the transform
     * @return Transform from source to target frame
     */
    geometry_msgs::msg::TransformStamped lookupTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const rclcpp::Time& time) const {
        return buffer_.lookupTransform(target_frame, source_frame, time);
    }

    /**
     * @brief Check if transform is available
     * @param target_frame Target coordinate frame
     * @param source_frame Source coordinate frame
     * @param time Time at which to check
     * @return True if transform is available
     */
    bool canTransform(const std::string& target_frame,
                     const std::string& source_frame,
                     const rclcpp::Time& time) const {
        return buffer_.canTransform(target_frame, source_frame, time);
    }

    /**
     * @brief Transform a pose from one frame to another
     * @param pose_in Input pose
     * @param target_frame Target frame to transform to
     * @return Transformed pose
     */
    geometry_msgs::msg::PoseStamped transform(
        const geometry_msgs::msg::PoseStamped& pose_in,
        const std::string& target_frame) const {
        return buffer_.transform(pose_in, target_frame);
    }

    // Access to underlying buffer if needed
    tf2_ros::Buffer& getBuffer() { return buffer_; }
    const tf2_ros::Buffer& getBuffer() const { return buffer_; }

private:
    tf2_ros::Buffer buffer_;
};

/**
 * @brief Wrapper for TF2 TransformListener with RAII management
 */
class TransformListener {
public:
    explicit TransformListener(Buffer& buffer, rclcpp::Node::SharedPtr node)
        : listener_(buffer.getBuffer(), node) {}

    // The listener automatically subscribes to /tf and /tf_static
    // No additional interface needed - it works in the background

private:
    tf2_ros::TransformListener listener_;
};

/**
 * @brief Helper functions for common transform operations
 */
namespace utils {

/**
 * @brief Get robot pose in map frame
 * @param buffer TF2 buffer
 * @param robot_frame Robot base frame (usually "base_link")
 * @param map_frame Map frame (usually "map")
 * @param time Time at which to get pose
 * @return Robot pose in map frame, or nullptr if transform unavailable
 */
inline std::optional<geometry_msgs::msg::PoseStamped> getRobotPose(
    const Buffer& buffer,
    const std::string& robot_frame = "base_link",
    const std::string& map_frame = "map",
    const rclcpp::Time& time = rclcpp::Time(0)) {
    
    try {
        auto transform = buffer.lookupTransform(map_frame, robot_frame, time);
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header = transform.header;
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;
        
        return pose;
    } catch (const tf2::TransformException& ex) {
        return std::nullopt;
    }
}

/**
 * @brief Convert transform to 2D pose (x, y, theta)
 * @param transform Input transform
 * @param x Output x position
 * @param y Output y position  
 * @param theta Output orientation (yaw angle)
 */
inline void transformToPose2D(const geometry_msgs::msg::TransformStamped& transform,
                             double& x, double& y, double& theta) {
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    
    // Convert quaternion to yaw angle
    const auto& q = transform.transform.rotation;
    theta = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

} // namespace utils

} // namespace tf2_compat

// =============================================================================
// CONVENIENCE MACROS
// =============================================================================

// Enable the workaround by default for Jazzy
#ifndef ROS_JAZZY_TF2_WORKAROUND
#define ROS_JAZZY_TF2_WORKAROUND
#endif

// Convenience macro for error handling
#define TF2_COMPAT_TRY_TRANSFORM(buffer, target, source, time, result) \
    try { \
        result = buffer.lookupTransform(target, source, time); \
    } catch (const tf2::TransformException& ex) { \
        RCLCPP_WARN(rclcpp::get_logger("tf2_compat"), \
                   "Transform lookup failed: %s", ex.what()); \
        return false; \
    }

#endif  // HECTOR_SLAM_CUSTOM__TF2_COMPATIBILITY_HPP_
