#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_navigation/intercept_math.hpp"

class InterceptController : public rclcpp::Node
{
public:
  InterceptController()
  : Node("intercept_controller")
  {
    odometry_topic_ = this->declare_parameter<std::string>(
      "odometry_topic", "/ground_truth/odom");
    target_topic_ = this->declare_parameter<std::string>(
      "target_topic", "/tracking/target_state");
    command_topic_ = this->declare_parameter<std::string>(
      "command_topic", "/cmd_vel/terminal");
    planning_frame_ = this->declare_parameter<std::string>("planning_frame", "odom");
    maximum_speed_ = this->declare_parameter<double>("maximum_speed", 1.0);
    minimum_speed_ = this->declare_parameter<double>("minimum_speed", 0.15);
    maximum_acceleration_ = this->declare_parameter<double>(
      "maximum_acceleration", 1.5);
    maximum_deceleration_ = this->declare_parameter<double>(
      "maximum_deceleration", 2.0);
    maximum_steering_angle_ = this->declare_parameter<double>(
      "maximum_steering_angle", 0.6);
    wheelbase_ = this->declare_parameter<double>("wheelbase", 0.24);
    slowdown_distance_ = this->declare_parameter<double>("slowdown_distance", 0.75);
    capture_radius_ = this->declare_parameter<double>("capture_radius", 0.45);
    max_intercept_time_ = this->declare_parameter<double>("max_intercept_time", 8.0);
    target_timeout_ = this->declare_parameter<double>("target_timeout", 0.2);
    odometry_timeout_ = this->declare_parameter<double>("odometry_timeout", 0.2);
    control_rate_ = this->declare_parameter<double>("control_rate", 20.0);

    if (
      maximum_speed_ <= 0.0 || minimum_speed_ < 0.0 ||
      minimum_speed_ > maximum_speed_ || maximum_acceleration_ <= 0.0 ||
      maximum_deceleration_ <= 0.0 || wheelbase_ <= 0.0 ||
      maximum_steering_angle_ <= 0.0 ||
      maximum_steering_angle_ >= 0.5 * std::acos(-1.0) ||
      slowdown_distance_ <= 0.0 || capture_radius_ <= 0.0 ||
      max_intercept_time_ <= 0.0 || target_timeout_ <= 0.0 ||
      odometry_timeout_ <= 0.0 || control_rate_ <= 0.0)
    {
      throw std::invalid_argument(
        "Intercept controller parameters must be positive and consistent.");
    }

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&InterceptController::odometryCallback, this, std::placeholders::_1));
    target_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      target_topic_, 10,
      std::bind(&InterceptController::targetCallback, this, std::placeholders::_1));
    command_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(command_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&InterceptController::update, this));
  }

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id != planning_frame_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring ego odometry outside the configured planning frame.");
      return;
    }
    robot_ = *msg;
    have_robot_ = true;
  }

  void targetCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id != planning_frame_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring target state outside the configured planning frame.");
      return;
    }
    target_ = *msg;
    have_target_ = true;
  }

  static double yaw(const geometry_msgs::msg::Quaternion & orientation)
  {
    const double sin_yaw = 2.0 *
      (orientation.w * orientation.z + orientation.x * orientation.y);
    const double cos_yaw = 1.0 - 2.0 *
      (orientation.y * orientation.y + orientation.z * orientation.z);
    return std::atan2(sin_yaw, cos_yaw);
  }

  void publishStop()
  {
    command_pub_->publish(geometry_msgs::msg::Twist());
    previous_speed_ = 0.0;
  }

  double rateLimitedSpeed(const double requested_speed)
  {
    const double period = 1.0 / control_rate_;
    const double lower = previous_speed_ - maximum_deceleration_ * period;
    const double upper = previous_speed_ + maximum_acceleration_ * period;
    previous_speed_ = std::clamp(requested_speed, lower, upper);
    return previous_speed_;
  }

  void update()
  {
    if (!have_robot_ || !have_target_) {
      publishStop();
      return;
    }

    const rclcpp::Time now = this->now();
    const double target_age = (now - rclcpp::Time(target_.header.stamp)).seconds();
    const double odometry_age = (now - rclcpp::Time(robot_.header.stamp)).seconds();
    if (
      target_age < 0.0 || target_age > target_timeout_ ||
      odometry_age < 0.0 || odometry_age > odometry_timeout_)
    {
      publishStop();
      return;
    }

    const double target_x =
      target_.pose.pose.position.x + target_.twist.twist.linear.x * target_age;
    const double target_y =
      target_.pose.pose.position.y + target_.twist.twist.linear.y * target_age;
    const double relative_x = target_x - robot_.pose.pose.position.x;
    const double relative_y = target_y - robot_.pose.pose.position.y;
    const double target_distance = std::hypot(relative_x, relative_y);
    if (target_distance <= capture_radius_) {
      publishStop();
      return;
    }

    const auto intercept_time = robot_navigation::solveInterceptTime(
      relative_x, relative_y,
      target_.twist.twist.linear.x, target_.twist.twist.linear.y,
      maximum_speed_);

    if (!intercept_time || *intercept_time > max_intercept_time_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "No reachable intercept within the configured horizon.");
      publishStop();
      return;
    }

    const double intercept_x =
      target_x + target_.twist.twist.linear.x * *intercept_time;
    const double intercept_y =
      target_y + target_.twist.twist.linear.y * *intercept_time;
    auto command = robot_navigation::pursuitCommand(
      intercept_x - robot_.pose.pose.position.x,
      intercept_y - robot_.pose.pose.position.y,
      yaw(robot_.pose.pose.orientation),
      target_distance,
      maximum_speed_,
      minimum_speed_,
      slowdown_distance_,
      wheelbase_,
      maximum_steering_angle_);
    const double requested_speed = command.linear_velocity;
    command.linear_velocity = rateLimitedSpeed(requested_speed);
    if (requested_speed > 1.0e-9) {
      command.angular_velocity *= command.linear_velocity / requested_speed;
    }

    geometry_msgs::msg::Twist output;
    output.linear.x = command.linear_velocity;
    output.angular.z = command.angular_velocity;
    command_pub_->publish(output);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry robot_;
  nav_msgs::msg::Odometry target_;
  std::string odometry_topic_;
  std::string target_topic_;
  std::string command_topic_;
  std::string planning_frame_;
  double maximum_speed_;
  double minimum_speed_;
  double maximum_acceleration_;
  double maximum_deceleration_;
  double maximum_steering_angle_;
  double wheelbase_;
  double slowdown_distance_;
  double capture_radius_;
  double max_intercept_time_;
  double target_timeout_;
  double odometry_timeout_;
  double control_rate_;
  double previous_speed_ = 0.0;
  bool have_robot_ = false;
  bool have_target_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterceptController>());
  rclcpp::shutdown();
  return 0;
}
