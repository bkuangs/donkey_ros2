#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class AckToJointCmdNode : public rclcpp::Node
{
public:
  AckToJointCmdNode() : Node("ack_to_joint_cmd")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/drive_cmd");
    steering_topic_ = declare_parameter<std::string>(
      "steering_topic", "/front_steer_controller/commands");
    wheel_topic_ = declare_parameter<std::string>(
      "wheel_topic", "/rear_wheel_controller/commands");

    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.033);
    max_speed_ = declare_parameter<double>("max_speed", 1.0);
    max_reverse_speed_ = declare_parameter<double>("max_reverse_speed", 1.0);
    max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.6);

    if (wheel_radius_ <= 0.0) {
      throw std::runtime_error("wheel_radius must be greater than zero");
    }
    if (max_speed_ < 0.0 || max_reverse_speed_ < 0.0 || max_steering_angle_ < 0.0) {
      throw std::runtime_error(
        "max_speed, max_reverse_speed, and max_steering_angle must be non-negative");
    }

    steering_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(steering_topic_, 10);
    wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(wheel_topic_, 10);

    drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      input_topic_,
      10,
      std::bind(&AckToJointCmdNode::onDriveCommand, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Converting %s to %s and %s",
      input_topic_.c_str(),
      steering_topic_.c_str(),
      wheel_topic_.c_str());
  }

private:
  static double clamp(const double value, const double lower, const double upper)
  {
    return std::min(std::max(value, lower), upper);
  }

  void onDriveCommand(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    const double speed = clamp(msg->drive.speed, -max_reverse_speed_, max_speed_);
    const double steering_angle = clamp(
      msg->drive.steering_angle,
      -max_steering_angle_,
      max_steering_angle_);

    std_msgs::msg::Float64MultiArray steering_cmd;
    steering_cmd.data = {steering_angle, steering_angle};

    const double wheel_angular_velocity = speed / wheel_radius_;
    std_msgs::msg::Float64MultiArray wheel_cmd;
    wheel_cmd.data = {wheel_angular_velocity, wheel_angular_velocity};

    steering_pub_->publish(steering_cmd);
    wheel_pub_->publish(wheel_cmd);
  }

  std::string input_topic_;
  std::string steering_topic_;
  std::string wheel_topic_;

  double wheel_radius_;
  double max_speed_;
  double max_reverse_speed_;
  double max_steering_angle_;

  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckToJointCmdNode>());
  rclcpp::shutdown();
  return 0;
}
