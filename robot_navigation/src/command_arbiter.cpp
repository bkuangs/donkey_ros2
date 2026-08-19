#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_navigation/control_logic.hpp"
#include "std_msgs/msg/u_int8.hpp"

class CommandArbiter : public rclcpp::Node
{
public:
  CommandArbiter()
  : Node("command_arbiter")
  {
    nav2_topic_ = this->declare_parameter<std::string>(
      "nav2_command_topic", "/cmd_vel/nav2");
    terminal_topic_ = this->declare_parameter<std::string>(
      "terminal_command_topic", "/cmd_vel/terminal");
    output_topic_ = this->declare_parameter<std::string>(
      "output_command_topic", "/cmd_vel");
    owner_topic_ = this->declare_parameter<std::string>(
      "owner_topic", "/navigation/cmd_vel_owner");
    command_timeout_ = this->declare_parameter<double>("command_timeout", 0.25);
    control_rate_ = this->declare_parameter<double>("control_rate", 20.0);

    if (command_timeout_ <= 0.0 || control_rate_ <= 0.0) {
      throw std::invalid_argument(
        "Command arbiter timeout and control rate must be positive.");
    }

    output_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    nav2_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      nav2_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        nav2_command_ = *message;
        nav2_received_at_ = std::chrono::steady_clock::now();
        have_nav2_command_ = true;
      });
    terminal_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      terminal_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        terminal_command_ = *message;
        terminal_received_at_ = std::chrono::steady_clock::now();
        have_terminal_command_ = true;
      });

    auto owner_qos = rclcpp::QoS(1).reliable().transient_local();
    owner_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      owner_topic_, owner_qos,
      std::bind(&CommandArbiter::ownerCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CommandArbiter::update, this));
  }

private:
  void publishStop()
  {
    output_pub_->publish(geometry_msgs::msg::Twist());
  }

  void ownerCallback(const std_msgs::msg::UInt8::SharedPtr message)
  {
    if (message->data > static_cast<uint8_t>(robot_navigation::CommandOwner::TERMINAL)) {
      RCLCPP_WARN(
        this->get_logger(), "Ignoring invalid command owner value %u.",
        static_cast<unsigned int>(message->data));
      return;
    }

    const auto requested = static_cast<robot_navigation::CommandOwner>(message->data);
    if (owner_ != requested) {
      publishStop();
      owner_ = requested;
    }
  }

  void update()
  {
    const auto now = std::chrono::steady_clock::now();
    if (owner_ == robot_navigation::CommandOwner::NAV2) {
      const double age = std::chrono::duration<double>(now - nav2_received_at_).count();
      if (robot_navigation::shouldForwardCommand(
          owner_, robot_navigation::CommandOwner::NAV2,
          have_nav2_command_, age, command_timeout_))
      {
        output_pub_->publish(nav2_command_);
        return;
      }
    } else if (owner_ == robot_navigation::CommandOwner::TERMINAL) {
      const double age =
        std::chrono::duration<double>(now - terminal_received_at_).count();
      if (robot_navigation::shouldForwardCommand(
          owner_, robot_navigation::CommandOwner::TERMINAL,
          have_terminal_command_, age, command_timeout_))
      {
        output_pub_->publish(terminal_command_);
        return;
      }
    }
    publishStop();
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr terminal_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr owner_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist nav2_command_;
  geometry_msgs::msg::Twist terminal_command_;
  std::chrono::steady_clock::time_point nav2_received_at_;
  std::chrono::steady_clock::time_point terminal_received_at_;
  std::string nav2_topic_;
  std::string terminal_topic_;
  std::string output_topic_;
  std::string owner_topic_;
  robot_navigation::CommandOwner owner_ = robot_navigation::CommandOwner::STOP;
  double command_timeout_;
  double control_rate_;
  bool have_nav2_command_ = false;
  bool have_terminal_command_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandArbiter>());
  rclcpp::shutdown();
  return 0;
}
