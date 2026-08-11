#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_tracking/constant_velocity_filter.hpp"

class TargetEkf : public rclcpp::Node
{
public:
  TargetEkf()
  : Node("target_ekf")
  {
    measurement_topic_ = this->declare_parameter<std::string>(
      "measurement_topic", "/perception/target_position");
    state_topic_ = this->declare_parameter<std::string>(
      "state_topic", "/tracking/target_state");
    tracking_frame_ = this->declare_parameter<std::string>("tracking_frame", "odom");
    acceleration_variance_ = this->declare_parameter<double>(
      "acceleration_variance", 2.0);
    default_measurement_variance_ = this->declare_parameter<double>(
      "default_measurement_variance", 0.04);
    maximum_prediction_interval_ = this->declare_parameter<double>(
      "maximum_prediction_interval", 1.0);
    innovation_gate_ = this->declare_parameter<double>("innovation_gate", 25.0);
    if (
      acceleration_variance_ < 0.0 || default_measurement_variance_ <= 0.0 ||
      maximum_prediction_interval_ <= 0.0 || innovation_gate_ <= 0.0)
    {
      throw std::invalid_argument("Invalid target filter parameters.");
    }

    measurement_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      measurement_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TargetEkf::measurementCallback, this, std::placeholders::_1));
    state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(state_topic_, 10);
  }

private:
  static bool finitePosition(
    const geometry_msgs::msg::PoseWithCovarianceStamped & measurement)
  {
    return std::isfinite(measurement.pose.pose.position.x) &&
           std::isfinite(measurement.pose.pose.position.y);
  }

  void initialize(
    const geometry_msgs::msg::PoseWithCovarianceStamped & measurement,
    const rclcpp::Time & stamp)
  {
    filter_.initialize(
      measurement.pose.pose.position.x,
      measurement.pose.pose.position.y,
      measurementVariance(measurement.pose.covariance[0]),
      measurementVariance(measurement.pose.covariance[7]));
    last_stamp_ = stamp;
  }

  double measurementVariance(const double value) const
  {
    return std::isfinite(value) && value > 0.0 ? value : default_measurement_variance_;
  }

  bool update(
    const geometry_msgs::msg::PoseWithCovarianceStamped & measurement,
    const double dt)
  {
    return filter_.update(
      measurement.pose.pose.position.x,
      measurement.pose.pose.position.y,
      measurementVariance(measurement.pose.covariance[0]),
      measurementVariance(measurement.pose.covariance[7]),
      dt,
      acceleration_variance_,
      innovation_gate_);
  }

  void measurementCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr measurement)
  {
    if (!finitePosition(*measurement) || measurement->header.frame_id != tracking_frame_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring invalid target measurement or unexpected frame.");
      return;
    }

    const rclcpp::Time stamp(measurement->header.stamp);
    if (!filter_.initialized()) {
      initialize(*measurement, stamp);
      publish(stamp);
      return;
    }

    const double dt = (stamp - last_stamp_).seconds();
    if (dt < 0.0) {
      RCLCPP_WARN(this->get_logger(), "Ignoring out-of-order target measurement.");
      return;
    }
    if (dt > maximum_prediction_interval_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Resetting target filter after a %.3f second measurement gap.", dt);
      initialize(*measurement, stamp);
      publish(stamp);
      return;
    }
    if (!update(*measurement, dt)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejected target measurement outside the innovation gate.");
    }
    last_stamp_ = stamp;
    publish(stamp);
  }

  void publish(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry output;
    const auto & state = filter_.state();
    const auto & covariance = filter_.covariance();
    output.header.stamp = stamp;
    output.header.frame_id = tracking_frame_;
    output.child_frame_id = "target";
    output.pose.pose.position.x = state(0);
    output.pose.pose.position.y = state(1);
    output.pose.pose.orientation.w = 1.0;
    output.twist.twist.linear.x = state(2);
    output.twist.twist.linear.y = state(3);
    output.pose.covariance[0] = covariance(0, 0);
    output.pose.covariance[1] = covariance(0, 1);
    output.pose.covariance[6] = covariance(1, 0);
    output.pose.covariance[7] = covariance(1, 1);
    output.twist.covariance[0] = covariance(2, 2);
    output.twist.covariance[1] = covariance(2, 3);
    output.twist.covariance[6] = covariance(3, 2);
    output.twist.covariance[7] = covariance(3, 3);
    state_pub_->publish(output);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    measurement_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
  std::string measurement_topic_;
  std::string state_topic_;
  std::string tracking_frame_;
  double acceleration_variance_;
  double default_measurement_variance_;
  double maximum_prediction_interval_;
  double innovation_gate_;
  robot_tracking::ConstantVelocityFilter filter_;
  rclcpp::Time last_stamp_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetEkf>());
  rclcpp::shutdown();
  return 0;
}
