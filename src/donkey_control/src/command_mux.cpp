#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// TODO: CHANGE ESTOP FROM ACKERMANN TO BOOL
// TODO: CONSIDER CHANGING NEUTRAL TO PREV FOR SMOOTHER ESTOP

class CmdMuxNode : public rclcpp::Node 
{
public:
  CmdMuxNode() : Node("cmd_mux_node")
  {
    output_topic_ = declare_parameter<std::string>("output_topic", "/drive_cmd");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    auto input_names = declare_parameter<std::vector<std::string>>(
      "input_names", {"estop", "teleop", "autonomy"});

    auto input_topics = declare_parameter<std::vector<std::string>>(
      "input_topics", {"/cmd/estop", "/cmd/teleop", "/cmd/autonomy"});

    auto priorities = declare_parameter<std::vector<int64_t>>(
      "priorities", {100, 50, 10});

    auto timeouts = declare_parameter<std::vector<double>>(
      "timeouts", {0.25, 0.50, 0.50});

    if (
      input_names.size() != input_topics.size() ||
      input_names.size() != priorities.size() ||
      input_names.size() != timeouts.size())
    {
      throw std::runtime_error(
        "input_names, input_topics, priorities, and timeouts must have same length");
    }

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      output_topic_, 10);

    // vector of InputSource structs
    inputs_.resize(input_names.size());

    for (size_t i = 0; i < input_names.size(); ++i) {
      // for each struct, set its name, topic, priority, and timeout
      inputs_[i].name = input_names[i];
      inputs_[i].topic = input_topics[i];
      inputs_[i].priority = priorities[i];
      inputs_[i].timeout_s = timeouts[i];

      // create one subscriber per input topic
      // store the latest msg, when it arrived, and mark as received
      inputs_[i].sub = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        inputs_[i].topic,
        10,
        [this, i](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
          inputs_[i].last_msg = *msg;
          inputs_[i].last_received_time = this->now();
          inputs_[i].has_msg = true;
        });

      RCLCPP_INFO(
        get_logger(),
        "Input '%s': topic=%s priority=%ld timeout=%.2f",
        inputs_[i].name.c_str(),
        inputs_[i].topic.c_str(),
        inputs_[i].priority,
        inputs_[i].timeout_s);
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);

    // calls onTimer() to publish at a stable rate (1 / publish_rate)
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&CmdMuxNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "Command mux publishing to %s", output_topic_.c_str());
  }

private:
  struct InputSource
  {
    std::string name;
    std::string topic;
    int64_t priority;
    double timeout_s;

    bool has_msg = false;
    rclcpp::Time last_received_time{0, 0, RCL_ROS_TIME};
    ackermann_msgs::msg::AckermannDriveStamped last_msg;
    
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub;
  };

  // check if input was received timely
  bool isFresh(const InputSource &input, const rclcpp::Time &now)
  {
    if (!input.has_msg) return false;

    const double age_s = (now - input.last_received_time).seconds();
    return age_s <= input.timeout_s;
  }

  // core selection logic
  // estop > teleop > autonomy
  void onTimer()
  {
    const auto current_time = this->now();

    InputSource *best = nullptr;  // initialize best to nothing
    
    for (auto &input : inputs_) { // check every struct
      if (!isFresh(input, current_time)) {
        continue;
      }

      if (best == nullptr || input.priority > best->priority) { // keep updating best
        best = &input;
      }
    }

    ackermann_msgs::msg::AckermannDriveStamped output_msg;
    output_msg.header.stamp = current_time;
    output_msg.header.frame_id = "base_link";

    // if a best source was found, copy it to output msg
    if (best != nullptr) {
      output_msg.drive = best->last_msg.drive;

      RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Selected command source: %s",
        best->name.c_str());
    } else {  // otherwise, publish neutral command
      output_msg.drive.speed = 0.0;
      output_msg.drive.steering_angle = 0.0;

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "No fresh command input. Publishing neutral command.");
    }

    drive_pub_->publish(output_msg);
  }

  std::string output_topic_;
  double publish_rate_hz_;

  std::vector<InputSource> inputs_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdMuxNode>());
  rclcpp::shutdown();
  return 0;
}