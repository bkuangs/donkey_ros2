#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


/*
TODO:
Simple version: (Current)
- assumes path and odom are same frame
- manually subtracts vehicle position from path point
- uses fixed lookahead
- weak stale-data handling
- okay for first sim

Serious version:
- uses TF2 to transform path points into base_link
- checks timestamps
- rejects stale data
- rejects invalid lookahead points
- has emergency stop behavior
- uses adaptive lookahead
- separates planner speed from controller speed
- safer for real robot / messy sim
*/


//// /odom + /nav/path -> /drive
class PurePursuit : public rclcpp::Node
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

    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 20.0);
    path_timeout_sec_ = tthis->declare_parameter<double>("path_timeout_sec", 0.5);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&PurePursuit::odomCallback, this, std::placeholders::_1)
    );

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/nav/path",
      10,
      std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1)
    );

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped(
      "/drive",
      10,
    );

    const auto period_ms = static_cast<int>(1000.0 / control_rate_hz_);

    // set periodic execution for control loop (node heartbeat)
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&PurePursuit::controlLoop, this)
    );

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Pure pursuit node started.");
  }

private:
  static double clamp(double value, double min_value, double max_value)
  {
    return std::max(min_value, std::min(value, max_value));
  }

  // odom contains 4D quaternion, we only care about yaw (direction)
  static double quaternionToYaw(const geometry_msgs::msg::Quaternion& q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double distance(
    double x1,
    double y1,
    double x2,
    double y2)
  {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_curr_ = msg->pose.pose.position.x;
    y_curr_ = msg->pose.pose.position.y;
    yaw_curr_ = quaternionToYaw(msg->pose.pose.orientation);
    current_speed_ = msg->twist.twist.linear.x;
    have_odom_ = true;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_path_ = *msg;
    have_path_ = !latest_path_.poses.empty();
    last_path_time_ = this->now();

    if (have_path_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Received path with %zu poses.",
        latest_path_.poses.size()
      );
    }
  }

  // @brief 
  bool findLookaheadPoint(geometry_msgs::msg::Point & lookahead_point)
  {
    if (!have_path_)
      return false;

    double nearest_dist = std::numeric_limits<double>::max();
    size_t nearest_index = 0;

    // find nearest path point
    for (size_t i = 0; i < latest_path_.poses.size(); ++i) {
      const auto &p = latest_path_.poses[i].pose.position;
      const double d = distance(x_curr_, y_curr_, p.x, p.y);

      if (d < nearest_dist) {
        nearest_dist = d;
        nearest_index = i;
      }
    }

    // from that point, find first point at least lookahead dist
    for (size_t i = nearest_index; i < latest_path_.poses.size(); ++i) {
      const auto & p = latest_path_.poses[i].pose.position;
      const double d = distance(x_curr_, y_curr_, p.x, p.y);

      if (d >= lookahead_distance_) {
        // TODO: reject points behind vehicle
        const double dx = p.x - x_curr_;
        const double dy = p.y - y_curr_;

        const double x_vehicle = std::cos(yaw_curr_) * dx + std::sin(yaw_curr_) * dy;

        if (x_vehicle > 0.0) {
          lookahead_point = p;
          return true;
        }
      }
    }

    // if no point is far enough, use the final path point only if it's ahead
    const auto & last = latest_path_.poses.back().pose.position;
    const double dx = last.x - x_curr_;
    const double dy = last.y - y_curr_;
    const double x_vehicle = std::cos(yaw_curr_) * dx + std::sin(yaw_curr_) * dy;

    if (x_vehicle > 0.0) {
      lookahead_point = last;
      return true;
    }

    // nothing valid ahead of the vehicle
    return false;
  }

  double purePursuit(const geometry_msgs::msg::Point &target_point)
  {
    const double dx = target_point.x - x_curr_;
    const double dy = target_point.y - y_curr_;

    // transform lookahead point from world to vehicle frame
    const double x_vehicle = std::cos(yaw_curr_) * dx + std::sin(yaw_curr_) * dy;
    const double y_vehicle = -std::sin(yaw_curr_) * dx + std::cos(yaw_curr_) * dy;
    const double lookahead = std::sqrt(x_vehicle * x_vehicle + y_vehicle * y_vehicle);

    if (lookahead < 1e-6)
      return 0.0;

    const double alpha = std::atan2(y_vehicle, x_vehicle);

    // pure pursuit steering: steering_angle = atan(2 * L * sin(alpha) / Ld)
    double steering_angle = std::atan2(
      2.0 * wheel_base_ * std::sin(alpha),
      lookahead
    );

    steering_angle = clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

    return steering_angle;
  }

  double pidControl(double desired_speed, double dt)
  {
    const double error = desired_speed - current_speed_;

    integral_error_ += error * dt;
    integral_error_ = clamp(integral_error_, -integral_limit_, integral_limit_);

    double derivative = 0.0;
    
    if (have_previous_error_ && dt > 1e-6) {
      derivative = (error - previous_error_) / dt;
    }

    previous_error_ = error;
    have_previous_error_ = true;

    double accel_cmd = 
      kp_ * error + 
      ki_ * integral_error_ + 
      kd_ * derivative;

    accel_cmd = clamp(accel_cmd, -max_accel_, max_accel_);

    return accel_cmd;
  }

  void publishStop()
  {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";

    cmd.drive.steering_angle = 0.0;
    cmd.drive.speed = 0.0;
    cmd.drive.acceleration = 0.0;

    drive_pub_->publish(cmd);
  }

  void controlLoop()
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Waiting for odometry..."
      );
      publishStop();
      return;
    }

    if ((this->now() - last_path_time_).seconds() > path_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Path is stale; stopping."
      );
      have_path_ = false;
      publishStop();
      return;
    }

    if (!have_path_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Waiting for path..."
      );
      publishStop();
      return;
    }

    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 1.0)
      dt = 1.0 / control_rate_hz_;

    geometry_msgs::msg::Point lookahead_point;

    if (!findLookaheadPoint(lookahead_point)) {
      publishStop();
      return;
    }

    const double steering_angle = purePursuit(lookahead_point);

    // slow down for sharper turns
    const double steering_fraction = std::abs(steering_angle) / std::max(max_steering_angle_, 1e-6);
    const double speed_scale = clamp(1.0 - 0.5 * steering_fraction, 0.35, 1.0);

    const double desired_speed = clamp(
      target_speed_ * speed_scale,
      min_speed_,
      max_speed_
    );
    const double accel_cmd = pidControl(desired_speed, dt);

    // ease speed down as we approach the end of the path (the follow point)
    const auto & goal = latest_path_.poses.back().pose.position;
    const double dist_to_goal = distance(x_curr_, y_curr_, goal.x, goal.y);
    const double approach_scale = clamp(dist_to_goal / std::max(lookahead_distance_, 1e-6), 0.0, 1.0);

    const double desired_speed = clamp(
      target_speed_ * speed_scale * approach_scale,
      min_speed_,
      max_speed_
    );

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now;
    cmd.header.frame_id = "base_link";

    cmd.drive.steering_angle = steering_angle;
    cmd.drive.speed = desired_speed;
    cmd.drive.acceleration = accel_cmd;

    drive_pub_->publish(cmd);

    RCLCPP_DEBUG(
      this->get_logger(),
      "steer=%.3f rad, desired_speed=%.3f m/s, current_speed=%.3f m/s, accel=%.3f",
      steering_angle,
      desired_speed,
      current_speed_,
      accel_cmd
    );
  }

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Topics
  std::string odom_topic_;
  std::string path_topic_;
  std::string drive_topic_;

  // Timing
  double control_rate_hz_;
  double path_timeout_sec_;
  rclcpp::Time last_time_;
  rclcpp::Time last_path_time_;

  // Vehicle state
  double x_curr_ = 0.0;
  double y_curr_ = 0.0;
  double yaw_curr_ = 0.0;
  double current_speed_ = 0.0;

  bool have_odom_ = false;
  bool have_path_ = false;

  nav_msgs::msg::Path latest_path_;

  // Pure pursuit params
  double wheel_base_;
  double lookahead_distance_;
  double max_steering_angle_;

  // Speed params
  double target_speed_;
  double min_speed_;
  double max_speed_;
  double max_accel_;

  // PID params
  double kp_;
  double ki_;
  double kd_;
  double integral_limit_;

  double integral_error_ = 0.0;
  double previous_error_ = 0.0;
  bool have_previous_error_ = false;

  // Timing
  double control_rate_hz_;
  rclcpp::Time last_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuit>());
  rclcpp::shutdown();
  return 0;
}
