#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class TwistToAckNode : public rclcpp::Node
{
public:
  TwistToAckNode() : Node("twist_to_ack")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cmd_vel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd/teleop");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    wheelbase_ = declare_parameter<double>("wheelbase", 0.26);
    max_speed_ = declare_parameter<double>("max_speed", 1.0);
    max_reverse_speed_ = declare_parameter<double>("max_reverse_speed", 1.0);
    max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.6);
    min_turning_speed_ = declare_parameter<double>("min_turning_speed", 1.0e-3);

    if (wheelbase_ <= 0.0) {
      throw std::runtime_error("wheelbase must be greater than zero");
    }
    if (max_speed_ < 0.0 || max_reverse_speed_ < 0.0 || max_steering_angle_ < 0.0) {
      throw std::runtime_error(
        "max_speed, max_reverse_speed, and max_steering_angle must be non-negative");
    }

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      output_topic_, 10);

    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_,
      10,
      std::bind(&TwistToAckNode::onTwist, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Converting %s Twist commands to %s Ackermann commands",
      input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  static double clamp(const double value, const double lower, const double upper)
  {
    return std::min(std::max(value, lower), upper);
  }

  void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double speed = clamp(msg->linear.x, -max_reverse_speed_, max_speed_);

    double steering_angle = 0.0;
    if (std::abs(speed) > min_turning_speed_) {
      steering_angle = std::atan(wheelbase_ * msg->angular.z / speed);
      steering_angle = clamp(steering_angle, -max_steering_angle_, max_steering_angle_);
    }

    ackermann_msgs::msg::AckermannDriveStamped output;
    output.header.stamp = now();
    output.header.frame_id = frame_id_;
    output.drive.speed = speed;
    output.drive.steering_angle = steering_angle;

    drive_pub_->publish(output);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;

  double wheelbase_;
  double max_speed_;
  double max_reverse_speed_;
  double max_steering_angle_;
  double min_turning_speed_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToAckNode>());
  rclcpp::shutdown();
  return 0;
}
