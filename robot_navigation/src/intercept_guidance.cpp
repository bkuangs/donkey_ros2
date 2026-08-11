#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class InterceptGuidance : public rclcpp::Node
{
public:
  InterceptGuidance()
  : Node("intercept_guidance")
  {
    odometry_topic_ = this->declare_parameter<std::string>(
      "odometry_topic", "/odometry/filtered");
    target_topic_ = this->declare_parameter<std::string>(
      "target_topic", "/tracking/target_state");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/navigation/intercept_pose");
    planning_frame_ = this->declare_parameter<std::string>("planning_frame", "odom");
    intercept_speed_ = this->declare_parameter<double>("intercept_speed", 1.5);
    max_intercept_time_ = this->declare_parameter<double>("max_intercept_time", 5.0);
    target_timeout_ = this->declare_parameter<double>("target_timeout", 0.5);
    odometry_timeout_ = this->declare_parameter<double>("odometry_timeout", 0.5);
    publish_rate_ = this->declare_parameter<double>("publish_rate", 10.0);

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, 10,
      std::bind(&InterceptGuidance::odometryCallback, this, std::placeholders::_1));
    target_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      target_topic_, 10,
      std::bind(&InterceptGuidance::targetCallback, this, std::placeholders::_1));
    intercept_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      output_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&InterceptGuidance::publishIntercept, this));
  }

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id != planning_frame_) {
      return;
    }
    robot_ = *msg;
    have_robot_ = true;
  }

  void targetCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id != planning_frame_) {
      return;
    }
    target_ = *msg;
    have_target_ = true;
  }

  double interceptTime(
    const double relative_x, const double relative_y,
    const double velocity_x, const double velocity_y) const
  {
    const double a =
      velocity_x * velocity_x + velocity_y * velocity_y -
      intercept_speed_ * intercept_speed_;
    const double b = 2.0 * (relative_x * velocity_x + relative_y * velocity_y);
    const double c = relative_x * relative_x + relative_y * relative_y;

    if (std::abs(a) < 1.0e-9) {
      if (std::abs(b) < 1.0e-9) {
        return c < 1.0e-9 ? 0.0 : std::numeric_limits<double>::quiet_NaN();
      }
      const double time = -c / b;
      return time >= 0.0 ? time : std::numeric_limits<double>::quiet_NaN();
    }

    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const double root = std::sqrt(discriminant);
    const double first = (-b - root) / (2.0 * a);
    const double second = (-b + root) / (2.0 * a);
    double result = std::numeric_limits<double>::infinity();
    if (first >= 0.0) {
      result = first;
    }
    if (second >= 0.0) {
      result = std::min(result, second);
    }
    return std::isfinite(result) ? result : std::numeric_limits<double>::quiet_NaN();
  }

  void publishIntercept()
  {
    if (!have_robot_ || !have_target_ || intercept_speed_ <= 0.0) {
      return;
    }

    const rclcpp::Time now = this->now();
    const double target_age = (now - rclcpp::Time(target_.header.stamp)).seconds();
    const double odometry_age = (now - rclcpp::Time(robot_.header.stamp)).seconds();
    if (
      target_age < 0.0 || target_age > target_timeout_ ||
      odometry_age < 0.0 || odometry_age > odometry_timeout_)
    {
      return;
    }

    const double target_x =
      target_.pose.pose.position.x + target_.twist.twist.linear.x * target_age;
    const double target_y =
      target_.pose.pose.position.y + target_.twist.twist.linear.y * target_age;
    const double relative_x = target_x - robot_.pose.pose.position.x;
    const double relative_y = target_y - robot_.pose.pose.position.y;
    const double time = interceptTime(
      relative_x, relative_y,
      target_.twist.twist.linear.x, target_.twist.twist.linear.y);

    if (!std::isfinite(time) || time > max_intercept_time_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "No reachable intercept within the configured horizon.");
      return;
    }

    geometry_msgs::msg::PoseStamped intercept;
    intercept.header.stamp = now;
    intercept.header.frame_id = planning_frame_;
    intercept.pose.position.x = target_x + target_.twist.twist.linear.x * time;
    intercept.pose.position.y = target_y + target_.twist.twist.linear.y * time;
    const double heading = std::atan2(
      intercept.pose.position.y - robot_.pose.pose.position.y,
      intercept.pose.position.x - robot_.pose.pose.position.x);
    intercept.pose.orientation.z = std::sin(0.5 * heading);
    intercept.pose.orientation.w = std::cos(0.5 * heading);
    intercept_pub_->publish(intercept);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr intercept_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry robot_;
  nav_msgs::msg::Odometry target_;
  std::string odometry_topic_;
  std::string target_topic_;
  std::string output_topic_;
  std::string planning_frame_;
  double intercept_speed_;
  double max_intercept_time_;
  double target_timeout_;
  double odometry_timeout_;
  double publish_rate_;
  bool have_robot_ = false;
  bool have_target_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterceptGuidance>());
  rclcpp::shutdown();
  return 0;
}
