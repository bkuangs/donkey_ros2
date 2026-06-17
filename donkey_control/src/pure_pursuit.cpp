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


//// /odom + /nav/path -> /drive
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

    const auto period_ms = static_cast<int>(1000.0 / control_rate_hz);

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
    speed_curr_ = msg->twist.twist.linear.x;
    have_odom_ = true;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_path_ = *msg;
    have_path_ = !latest_path_.poses.empty();

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

    // if no point is far enough, use final path point
    lookahead_point = latest_path_.poses.back().pose.position;
    return true;
  }

  bool reachedGoal() const
  {
    if (!have_path_)
      return false;

    const auto &goal = latest_path_.poses.back().pose.position;
    const double dist_to_goal = distance(x_curr_, y_curr_, goal.x, goal.y);

    return dist_to_goal < goal_tolerance_;
  }

  double purePursuit(const geometry_msgs::msg::Point &target_point)
  {
    const double dx = target_point.x - x_curr_;
    const double dy = target_point.y - y_curr_;
  }
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  return 0;
}
