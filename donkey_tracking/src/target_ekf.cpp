#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "Eigen/Dense"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

/*
Example target EKF node.

Input:
  /perception/target_centroid (geometry_msgs/PointStamped)
    point.x = target centroid u pixel coordinate
    point.y = target centroid v pixel coordinate
    point.z = contour area, used here as a simple apparent-size measurement

State:
  x = [u, v, area, u_dot, v_dot, area_dot]^T

Model:
  Constant velocity in image/measurement space.

Output:
  /tracking/target_state (nav_msgs/Odometry)
    pose.position.x/y/z = filtered u/v/area
    twist.linear.x/y/z = filtered u_dot/v_dot/area_dot

  /tracking/target_centroid_filtered (geometry_msgs/PointStamped)
    point.x/y/z = filtered u/v/area
*/
class TargetEkf : public rclcpp::Node
{
public:
  TargetEkf()
  : Node("target_ekf")
  {
    target_topic_ = this->declare_parameter<std::string>(
      "target_topic", "/perception/target_centroid");
    state_topic_ = this->declare_parameter<std::string>(
      "state_topic", "/tracking/target_state");
    filtered_centroid_topic_ = this->declare_parameter<std::string>(
      "filtered_centroid_topic", "/tracking/target_centroid_filtered");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 30.0);
    stale_timeout_sec_ = this->declare_parameter<double>("stale_timeout_sec", 0.5);

    process_noise_position_ = this->declare_parameter<double>("process_noise_position", 25.0);
    process_noise_area_ = this->declare_parameter<double>("process_noise_area", 2500.0);
    process_noise_velocity_ = this->declare_parameter<double>("process_noise_velocity", 100.0);
    process_noise_area_velocity_ = this->declare_parameter<double>("process_noise_area_velocity", 10000.0);

    measurement_noise_position_ = this->declare_parameter<double>("measurement_noise_position", 25.0);
    measurement_noise_area_ = this->declare_parameter<double>("measurement_noise_area", 2500.0);

    resetFilter();

    target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      target_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&TargetEkf::targetCallback, this, std::placeholders::_1));

    state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(state_topic_, 10);
    filtered_centroid_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      filtered_centroid_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TargetEkf::publishTimerCallback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Target EKF started. Subscribing to %s, publishing %s.",
      target_topic_.c_str(),
      state_topic_.c_str());
  }

private:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using Matrix3d = Eigen::Matrix<double, 3, 3>;
  using Matrix3x6d = Eigen::Matrix<double, 3, 6>;

  static bool measurementIsValid(const geometry_msgs::msg::PointStamped & msg)
  {
    return std::isfinite(msg.point.x) && std::isfinite(msg.point.y) &&
      std::isfinite(msg.point.z) && msg.point.z > 0.0;
  }

  void resetFilter()
  {
    x_.setZero();
    p_.setIdentity();
    p_ *= 1.0e3;
    initialized_ = false;
    have_prediction_time_ = false;
    last_frame_id_ = "camera";
  }

  Matrix6d transitionMatrix(const double dt) const
  {
    Matrix6d f = Matrix6d::Identity();
    f(0, 3) = dt;
    f(1, 4) = dt;
    f(2, 5) = dt;
    return f;
  }

  Matrix6d processNoise(const double dt) const
  {
    Matrix6d q = Matrix6d::Zero();
    const double dt2 = dt * dt;

    q(0, 0) = process_noise_position_ * dt2;
    q(1, 1) = process_noise_position_ * dt2;
    q(2, 2) = process_noise_area_ * dt2;
    q(3, 3) = process_noise_velocity_ * dt;
    q(4, 4) = process_noise_velocity_ * dt;
    q(5, 5) = process_noise_area_velocity_ * dt;

    return q;
  }

  Vector3d measurementModel(const Vector6d & x) const
  {
    Vector3d z_pred;
    z_pred << x(0), x(1), x(2);
    return z_pred;
  }

  Matrix3x6d measurementJacobian() const
  {
    Matrix3x6d h = Matrix3x6d::Zero();
    h(0, 0) = 1.0;
    h(1, 1) = 1.0;
    h(2, 2) = 1.0;
    return h;
  }

  Matrix3d measurementNoise() const
  {
    Matrix3d r = Matrix3d::Zero();
    r(0, 0) = measurement_noise_position_;
    r(1, 1) = measurement_noise_position_;
    r(2, 2) = measurement_noise_area_;
    return r;
  }

  void initialize(const geometry_msgs::msg::PointStamped & msg)
  {
    x_.setZero();
    x_(0) = msg.point.x;
    x_(1) = msg.point.y;
    x_(2) = msg.point.z;

    p_.setIdentity();
    p_(0, 0) = measurement_noise_position_;
    p_(1, 1) = measurement_noise_position_;
    p_(2, 2) = measurement_noise_area_;
    p_(3, 3) = 1.0e3;
    p_(4, 4) = 1.0e3;
    p_(5, 5) = 1.0e5;

    initialized_ = true;
    have_prediction_time_ = true;
    last_prediction_time_ = rclcpp::Time(msg.header.stamp);
    last_measurement_time_ = rclcpp::Time(msg.header.stamp);
    last_frame_id_ = msg.header.frame_id.empty() ? "camera" : msg.header.frame_id;
  }

  void predictTo(const rclcpp::Time & stamp)
  {
    if (!initialized_) {
      return;
    }

    if (!have_prediction_time_) {
      last_prediction_time_ = stamp;
      have_prediction_time_ = true;
      return;
    }

    double dt = (stamp - last_prediction_time_).seconds();
    if (dt <= 0.0) {
      return;
    }

    dt = std::min(dt, 1.0);

    const Matrix6d f = transitionMatrix(dt);
    x_ = f * x_;
    p_ = f * p_ * f.transpose() + processNoise(dt);
    p_ = 0.5 * (p_ + p_.transpose());

    x_(2) = std::max(1.0, x_(2));
    last_prediction_time_ = stamp;
  }

  void update(const geometry_msgs::msg::PointStamped & msg)
  {
    const Vector3d z(msg.point.x, msg.point.y, msg.point.z);
    const Matrix3x6d h = measurementJacobian();
    const Matrix3d r = measurementNoise();

    const Vector3d innovation = z - measurementModel(x_);
    const Matrix3d s = h * p_ * h.transpose() + r;
    const Eigen::Matrix<double, 6, 3> k = p_ * h.transpose() * s.inverse();

    x_ = x_ + k * innovation;
    const Matrix6d i = Matrix6d::Identity();
    p_ = (i - k * h) * p_ * (i - k * h).transpose() + k * r * k.transpose();
    p_ = 0.5 * (p_ + p_.transpose());

    x_(2) = std::max(1.0, x_(2));
    last_measurement_time_ = rclcpp::Time(msg.header.stamp);
    last_frame_id_ = msg.header.frame_id.empty() ? "camera" : msg.header.frame_id;
  }

  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!measurementIsValid(*msg)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring invalid target measurement.");
      return;
    }

    if (!initialized_) {
      initialize(*msg);
      publishState(rclcpp::Time(msg->header.stamp));
      return;
    }

    predictTo(rclcpp::Time(msg->header.stamp));
    update(*msg);
    publishState(rclcpp::Time(msg->header.stamp));
  }

  void publishTimerCallback()
  {
    if (!initialized_) {
      return;
    }

    const rclcpp::Time now = this->now();
    predictTo(now);

    const double measurement_age = (now - last_measurement_time_).seconds();
    if (measurement_age > stale_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Target measurement is stale: %.2f seconds old.", measurement_age);
    }

    publishState(now);
  }

  void publishState(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry state_msg;
    state_msg.header.stamp = stamp;
    state_msg.header.frame_id = last_frame_id_;
    state_msg.child_frame_id = "target";

    state_msg.pose.pose.position.x = x_(0);
    state_msg.pose.pose.position.y = x_(1);
    state_msg.pose.pose.position.z = x_(2);
    state_msg.pose.pose.orientation.w = 1.0;

    state_msg.twist.twist.linear.x = x_(3);
    state_msg.twist.twist.linear.y = x_(4);
    state_msg.twist.twist.linear.z = x_(5);

    state_msg.pose.covariance[0] = p_(0, 0);
    state_msg.pose.covariance[7] = p_(1, 1);
    state_msg.pose.covariance[14] = p_(2, 2);
    state_msg.twist.covariance[0] = p_(3, 3);
    state_msg.twist.covariance[7] = p_(4, 4);
    state_msg.twist.covariance[14] = p_(5, 5);

    state_pub_->publish(state_msg);

    geometry_msgs::msg::PointStamped filtered_msg;
    filtered_msg.header = state_msg.header;
    filtered_msg.point.x = x_(0);
    filtered_msg.point.y = x_(1);
    filtered_msg.point.z = x_(2);
    filtered_centroid_pub_->publish(filtered_msg);
  }

  std::string target_topic_;
  std::string state_topic_;
  std::string filtered_centroid_topic_;
  std::string last_frame_id_;

  double publish_rate_hz_ = 30.0;
  double stale_timeout_sec_ = 0.5;
  double process_noise_position_ = 25.0;
  double process_noise_area_ = 2500.0;
  double process_noise_velocity_ = 100.0;
  double process_noise_area_velocity_ = 10000.0;
  double measurement_noise_position_ = 25.0;
  double measurement_noise_area_ = 2500.0;

  Vector6d x_;
  Matrix6d p_;

  bool initialized_ = false;
  bool have_prediction_time_ = false;
  rclcpp::Time last_prediction_time_;
  rclcpp::Time last_measurement_time_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr filtered_centroid_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetEkf>());
  rclcpp::shutdown();
  return 0;
}
