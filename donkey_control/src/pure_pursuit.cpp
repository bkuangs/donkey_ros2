#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class PurePursuit() : public rclcpp::Node
{
public:
  PurePursuit()
  : Node("pure_pursuit")
  {
    // params
    // TODO: tune
    wheel_base_ = this->declare_parameter<double>("wheel_base", 0.33);
    lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 1.0);
    max_steering_angle_ = this->declare_parameter<double>("max_steering_angle", 0.4189); // ~24 deg

    target_speed_ = this->declare_parameter<double>("target_speed", 1.0);
    min_speed_ = this->declare_parameter<double>("min_speed", 0.0);
    max_speed_ = this->declare_parameter<double>("max_speed", 2.0);
    max_accel_ = this->declare_parameter<double>("max_accel", 2.0);

    kp_ = this->declare_parameter<double>("kp", 1.0);
    ki_ = this->declare_parameter<double>("ki", 0.0);
    kd_ = this->declare_parameter<double>("kd", 0.05);
    integral_limit_ = this->declare_parameter<double>("integral_limit", 5.0);

    goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.25);
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 20.0);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&PurePursuit::odomCallback, this, std::placeholders::_1)
    );

    path_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/nav/path",
      10,
      std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1)
    );

    drive_pub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/drive",
      10,
    );

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Pure pursuit node started.");
  }
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  return 0;
}
