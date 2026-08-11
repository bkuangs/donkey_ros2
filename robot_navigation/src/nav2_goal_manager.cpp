#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Nav2GoalManager : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2GoalManager()
  : Node("nav2_goal_manager")
  {
    intercept_topic_ = this->declare_parameter<std::string>(
      "intercept_topic", "/navigation/intercept_pose");
    action_name_ = this->declare_parameter<std::string>(
      "action_name", "navigate_to_pose");
    goal_frame_ = this->declare_parameter<std::string>("goal_frame", "odom");
    update_rate_ = this->declare_parameter<double>("update_rate", 10.0);
    minimum_send_interval_ = this->declare_parameter<double>(
      "minimum_send_interval", 0.5);
    position_update_threshold_ = this->declare_parameter<double>(
      "position_update_threshold", 0.25);
    intercept_timeout_ = this->declare_parameter<double>("intercept_timeout", 0.75);

    intercept_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      intercept_topic_, 10,
      std::bind(&Nav2GoalManager::interceptCallback, this, std::placeholders::_1));
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    const auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Nav2GoalManager::update, this));
  }

private:
  void interceptCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != goal_frame_) {
      RCLCPP_WARN(this->get_logger(), "Ignoring intercept pose in unexpected frame.");
      return;
    }
    latest_intercept_ = *msg;
    have_intercept_ = true;
  }

  double goalDisplacement() const
  {
    return std::hypot(
      latest_intercept_.pose.position.x - last_sent_intercept_.pose.position.x,
      latest_intercept_.pose.position.y - last_sent_intercept_.pose.position.y);
  }

  void cancelStaleGoal()
  {
    if (active_goal_) {
      action_client_->async_cancel_goal(active_goal_);
      active_goal_.reset();
    }
    have_intercept_ = false;
    have_last_sent_ = false;
  }

  void update()
  {
    if (!have_intercept_) {
      return;
    }

    const rclcpp::Time now = this->now();
    const double age = (now - rclcpp::Time(latest_intercept_.header.stamp)).seconds();
    if (age < 0.0 || age > intercept_timeout_) {
      cancelStaleGoal();
      return;
    }
    if (!action_client_->action_server_is_ready() || send_in_progress_) {
      return;
    }
    if (have_last_sent_) {
      const double since_last_send = (now - last_send_time_).seconds();
      if (
        since_last_send < minimum_send_interval_ ||
        goalDisplacement() < position_update_threshold_)
      {
        return;
      }
    }

    NavigateToPose::Goal goal;
    goal.pose = latest_intercept_;
    const std::uint64_t sequence = ++send_sequence_;
    send_in_progress_ = true;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, sequence](const GoalHandle::SharedPtr & handle) {
        send_in_progress_ = false;
        if (!handle) {
          RCLCPP_WARN(this->get_logger(), "Nav2 rejected intercept goal.");
          return;
        }
        if (sequence >= active_sequence_) {
          active_sequence_ = sequence;
          active_goal_ = handle;
        }
      };
    options.result_callback =
      [this, sequence](const GoalHandle::WrappedResult &) {
        if (sequence == active_sequence_) {
          active_goal_.reset();
        }
      };

    action_client_->async_send_goal(goal, options);
    last_sent_intercept_ = latest_intercept_;
    last_send_time_ = now;
    have_last_sent_ = true;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr intercept_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  GoalHandle::SharedPtr active_goal_;
  geometry_msgs::msg::PoseStamped latest_intercept_;
  geometry_msgs::msg::PoseStamped last_sent_intercept_;
  rclcpp::Time last_send_time_;
  std::string intercept_topic_;
  std::string action_name_;
  std::string goal_frame_;
  double update_rate_;
  double minimum_send_interval_;
  double position_update_threshold_;
  double intercept_timeout_;
  std::uint64_t send_sequence_ = 0;
  std::uint64_t active_sequence_ = 0;
  bool have_intercept_ = false;
  bool have_last_sent_ = false;
  bool send_in_progress_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2GoalManager>());
  rclcpp::shutdown();
  return 0;
}
