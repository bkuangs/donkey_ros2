#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "Eigen/Dense"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

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

    measurement_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      measurement_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TargetEkf::measurementCallback, this, std::placeholders::_1));
    state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(state_topic_, 10);
  }

private:
  using Vector4d = Eigen::Matrix<double, 4, 1>;
  using Matrix4d = Eigen::Matrix<double, 4, 4>;
  using Vector2d = Eigen::Matrix<double, 2, 1>;
  using Matrix2d = Eigen::Matrix<double, 2, 2>;
  using Matrix2x4d = Eigen::Matrix<double, 2, 4>;

  static bool finitePosition(
    const geometry_msgs::msg::PoseWithCovarianceStamped & measurement)
  {
    return std::isfinite(measurement.pose.pose.position.x) &&
           std::isfinite(measurement.pose.pose.position.y);
  }

  Matrix4d transition(const double dt) const
  {
    Matrix4d result = Matrix4d::Identity();
    result(0, 2) = dt;
    result(1, 3) = dt;
    return result;
  }

  Matrix4d processNoise(const double dt) const
  {
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    Matrix4d noise = Matrix4d::Zero();

    noise(0, 0) = dt4 / 4.0;
    noise(0, 2) = dt3 / 2.0;
    noise(2, 0) = dt3 / 2.0;
    noise(2, 2) = dt2;
    noise(1, 1) = dt4 / 4.0;
    noise(1, 3) = dt3 / 2.0;
    noise(3, 1) = dt3 / 2.0;
    noise(3, 3) = dt2;
    return acceleration_variance_ * noise;
  }

  void initialize(
    const geometry_msgs::msg::PoseWithCovarianceStamped & measurement,
    const rclcpp::Time & stamp)
  {
    state_ << measurement.pose.pose.position.x, measurement.pose.pose.position.y, 0.0, 0.0;
    covariance_.setZero();
    covariance_(0, 0) = measurementVariance(measurement.pose.covariance[0]);
    covariance_(1, 1) = measurementVariance(measurement.pose.covariance[7]);
    covariance_(2, 2) = 4.0;
    covariance_(3, 3) = 4.0;
    last_stamp_ = stamp;
    initialized_ = true;
  }

  double measurementVariance(const double value) const
  {
    return std::isfinite(value) && value > 0.0 ? value : default_measurement_variance_;
  }

  void predict(const double dt)
  {
    const Matrix4d state_transition = transition(dt);
    state_ = state_transition * state_;
    covariance_ =
      state_transition * covariance_ * state_transition.transpose() + processNoise(dt);
  }

  void update(const geometry_msgs::msg::PoseWithCovarianceStamped & measurement)
  {
    Matrix2x4d observation = Matrix2x4d::Zero();
    observation(0, 0) = 1.0;
    observation(1, 1) = 1.0;

    Matrix2d measurement_noise = Matrix2d::Zero();
    measurement_noise(0, 0) = measurementVariance(measurement.pose.covariance[0]);
    measurement_noise(1, 1) = measurementVariance(measurement.pose.covariance[7]);

    const Vector2d measured_position(
      measurement.pose.pose.position.x,
      measurement.pose.pose.position.y);
    const Vector2d innovation = measured_position - observation * state_;
    const Matrix2d innovation_covariance =
      observation * covariance_ * observation.transpose() + measurement_noise;
    const Eigen::Matrix<double, 4, 2> gain =
      covariance_ * observation.transpose() * innovation_covariance.inverse();

    state_ += gain * innovation;
    const Matrix4d identity = Matrix4d::Identity();
    covariance_ =
      (identity - gain * observation) * covariance_ *
      (identity - gain * observation).transpose() +
      gain * measurement_noise * gain.transpose();
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
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
    if (!initialized_) {
      initialize(*measurement, stamp);
      publish(stamp);
      return;
    }

    const double dt = (stamp - last_stamp_).seconds();
    if (dt < 0.0) {
      RCLCPP_WARN(this->get_logger(), "Ignoring out-of-order target measurement.");
      return;
    }
    if (dt > 0.0) {
      predict(dt);
    }
    update(*measurement);
    last_stamp_ = stamp;
    publish(stamp);
  }

  void publish(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry output;
    output.header.stamp = stamp;
    output.header.frame_id = tracking_frame_;
    output.child_frame_id = "target";
    output.pose.pose.position.x = state_(0);
    output.pose.pose.position.y = state_(1);
    output.pose.pose.orientation.w = 1.0;
    output.twist.twist.linear.x = state_(2);
    output.twist.twist.linear.y = state_(3);
    output.pose.covariance[0] = covariance_(0, 0);
    output.pose.covariance[1] = covariance_(0, 1);
    output.pose.covariance[6] = covariance_(1, 0);
    output.pose.covariance[7] = covariance_(1, 1);
    output.twist.covariance[0] = covariance_(2, 2);
    output.twist.covariance[1] = covariance_(2, 3);
    output.twist.covariance[6] = covariance_(3, 2);
    output.twist.covariance[7] = covariance_(3, 3);
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
  Vector4d state_ = Vector4d::Zero();
  Matrix4d covariance_ = Matrix4d::Identity();
  rclcpp::Time last_stamp_;
  bool initialized_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetEkf>());
  rclcpp::shutdown();
  return 0;
}
