#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class OdomSimulator : public rclcpp::Node {
public:
  OdomSimulator() : rclcpp::Node("odom_simulator") {
    // Parameters
    this->declare_parameter("frame_id", std::string("odom"));
    this->declare_parameter("child_frame_id", std::string("base_link"));
    this->declare_parameter("publish_rate", 20.0);        // Hz
    frame_id_ = this->get_parameter("frame_id").as_string();
    child_frame_id_ = this->get_parameter("child_frame_id").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        lin_vel_ = msg->linear.x;
        ang_vel_ = msg->angular.z;
      });

    last_time_ = this->now();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&OdomSimulator::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "Odom simulator started: listening to /cmd_vel");
  }

private:
  void onTimer() {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) dt = 1.0 / publish_rate_;
    last_time_ = now;

    // Integrate simple unicycle model
    x_ += lin_vel_ * std::cos(theta_) * dt;
    y_ += lin_vel_ * std::sin(theta_) * dt;
    theta_ += ang_vel_ * dt;
    normalizeAngle(theta_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;

    // Pose
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // Twist (in child frame)
    odom.twist.twist.linear.x = lin_vel_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = ang_vel_;

    odom_pub_->publish(odom);
  }

  static void normalizeAngle(double &a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
  }

  // Params
  std::string frame_id_;
  std::string child_frame_id_;
  double publish_rate_{};

  // Velocity (set by /cmd_vel)
  double lin_vel_{};
  double ang_vel_{};

  // State
  double x_{};
  double y_{};
  double theta_{};

  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomSimulator>());
  rclcpp::shutdown();
  return 0;
}
