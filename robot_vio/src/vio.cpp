#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class VioAdapter : public rclcpp::Node
{
public:
  VioAdapter()
  : Node("vio_adapter"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "/ov_msckf/odomimu");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/vio/odom");
    input_child_frame_ = this->declare_parameter<std::string>(
      "input_child_frame", "imu_link");
    base_frame_ = this->declare_parameter<std::string>(
      "base_frame", "base_footprint");
    output_frame_ = this->declare_parameter<std::string>("output_frame", "odom");
    position_variance_ = this->declare_parameter<double>("position_variance", 0.05);
    yaw_variance_ = this->declare_parameter<double>("yaw_variance", 0.02);

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VioAdapter::odometryCallback, this, std::placeholders::_1));
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic_, 10);
  }

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr input)
  {
    const std::string sensor_frame =
      input_child_frame_.empty() ? input->child_frame_id : input_child_frame_;
    if (sensor_frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "VIO input has no sensor frame.");
      return;
    }

    tf2::Transform sensor_from_base = tf2::Transform::getIdentity();
    if (sensor_frame != base_frame_) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          sensor_frame, base_frame_, tf2::TimePointZero);
        tf2::fromMsg(transform.transform, sensor_from_base);
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Cannot convert VIO pose from %s to %s: %s",
          sensor_frame.c_str(), base_frame_.c_str(), error.what());
        return;
      }
    }

    tf2::Transform world_from_sensor;
    tf2::fromMsg(input->pose.pose, world_from_sensor);
    const tf2::Transform world_from_base = world_from_sensor * sensor_from_base;

    if (!have_origin_) {
      odom_from_world_ = world_from_base.inverse();
      have_origin_ = true;
    }
    const tf2::Transform odom_from_base = odom_from_world_ * world_from_base;

    nav_msgs::msg::Odometry output;
    output.header.stamp = input->header.stamp;
    output.header.frame_id = output_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = odom_from_base.getOrigin().x();
    output.pose.pose.position.y = odom_from_base.getOrigin().y();
    output.pose.pose.position.z = odom_from_base.getOrigin().z();
    output.pose.pose.orientation = tf2::toMsg(odom_from_base.getRotation());
    output.pose.covariance.fill(0.0);
    output.pose.covariance[0] = position_variance_;
    output.pose.covariance[7] = position_variance_;
    output.pose.covariance[14] = 1.0e6;
    output.pose.covariance[21] = 1.0e6;
    output.pose.covariance[28] = 1.0e6;
    output.pose.covariance[35] = yaw_variance_;
    output.twist.twist = geometry_msgs::msg::Twist();
    output.twist.covariance.fill(0.0);
    output.twist.covariance[0] = 1.0e6;
    output.twist.covariance[7] = 1.0e6;
    output.twist.covariance[14] = 1.0e6;
    output.twist.covariance[21] = 1.0e6;
    output.twist.covariance[28] = 1.0e6;
    output.twist.covariance[35] = 1.0e6;
    odometry_pub_->publish(output);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2::Transform odom_from_world_;
  std::string input_topic_;
  std::string output_topic_;
  std::string input_child_frame_;
  std::string base_frame_;
  std::string output_frame_;
  double position_variance_;
  double yaw_variance_;
  bool have_origin_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VioAdapter>());
  rclcpp::shutdown();
  return 0;
}
