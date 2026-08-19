#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "action_msgs/srv/cancel_goal.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_navigation/control_logic.hpp"
#include "robot_navigation/intercept_math.hpp"
#include "std_msgs/msg/u_int8.hpp"

class InterceptionSupervisor : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  InterceptionSupervisor()
  : Node("interception_supervisor")
  {
    odometry_topic_ = this->declare_parameter<std::string>(
      "odometry_topic", "/ground_truth/odom");
    target_topic_ = this->declare_parameter<std::string>(
      "target_topic", "/tracking/target_state");
    owner_topic_ = this->declare_parameter<std::string>(
      "owner_topic", "/navigation/cmd_vel_owner");
    action_name_ = this->declare_parameter<std::string>(
      "navigate_to_pose_action", "/navigate_to_pose");
    planning_frame_ = this->declare_parameter<std::string>("planning_frame", "odom");
    intercept_speed_ = this->declare_parameter<double>("intercept_speed", 1.0);
    max_intercept_time_ = this->declare_parameter<double>("max_intercept_time", 8.0);
    terminal_enter_distance_ = this->declare_parameter<double>(
      "terminal_enter_distance", 1.0);
    terminal_exit_distance_ = this->declare_parameter<double>(
      "terminal_exit_distance", 1.25);
    capture_radius_ = this->declare_parameter<double>("capture_radius", 0.45);
    target_timeout_ = this->declare_parameter<double>("target_timeout", 1.0);
    odometry_timeout_ = this->declare_parameter<double>("odometry_timeout", 0.2);
    goal_refresh_period_ = this->declare_parameter<double>(
      "goal_refresh_period", 1.0);
    goal_refresh_distance_ = this->declare_parameter<double>(
      "goal_refresh_distance", 0.5);
    control_rate_ = this->declare_parameter<double>("control_rate", 10.0);

    if (
      intercept_speed_ <= 0.0 || max_intercept_time_ <= 0.0 ||
      terminal_enter_distance_ <= capture_radius_ ||
      terminal_exit_distance_ <= terminal_enter_distance_ ||
      capture_radius_ <= 0.0 || target_timeout_ <= 0.0 ||
      odometry_timeout_ <= 0.0 || goal_refresh_period_ <= 0.0 ||
      goal_refresh_distance_ < 0.0 || control_rate_ <= 0.0)
    {
      throw std::invalid_argument(
        "Interception supervisor parameters must be positive and consistent.");
    }

    auto owner_qos = rclcpp::QoS(1).reliable().transient_local();
    owner_pub_ = this->create_publisher<std_msgs::msg::UInt8>(owner_topic_, owner_qos);
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&InterceptionSupervisor::odometryCallback, this, std::placeholders::_1));
    target_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      target_topic_, 10,
      std::bind(&InterceptionSupervisor::targetCallback, this, std::placeholders::_1));
    navigate_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    publishOwner(robot_navigation::CommandOwner::STOP);
    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&InterceptionSupervisor::update, this));
  }

private:
  enum class NavGoalState
  {
    IDLE,
    REQUESTING,
    ACTIVE,
    CANCELLING,
  };

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id != planning_frame_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring ego odometry outside the configured planning frame.");
      return;
    }
    robot_ = *message;
    have_robot_ = true;
  }

  void targetCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id != planning_frame_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring target state outside the configured planning frame.");
      return;
    }
    target_ = *message;
    have_target_ = true;
  }

  void publishOwner(const robot_navigation::CommandOwner owner)
  {
    if (owner == published_owner_) {
      return;
    }
    std_msgs::msg::UInt8 message;
    message.data = static_cast<uint8_t>(owner);
    owner_pub_->publish(message);
    published_owner_ = owner;
  }

  void cancelNavigation(const robot_navigation::CommandOwner owner_after_cancel)
  {
    desired_owner_ = owner_after_cancel;
    if (nav_goal_state_ == NavGoalState::IDLE) {
      last_goal_time_.reset();
      publishOwner(desired_owner_);
      return;
    }
    publishOwner(robot_navigation::CommandOwner::STOP);
    if (
      nav_goal_state_ != NavGoalState::ACTIVE ||
      !navigate_client_->action_server_is_ready())
    {
      return;
    }

    nav_goal_state_ = NavGoalState::CANCELLING;
    const auto goal_handle = goal_handle_;
    navigate_client_->async_cancel_goal(
      goal_handle,
      [this, goal_handle](
        const action_msgs::srv::CancelGoal::Response::SharedPtr response)
      {
        if (goal_handle != goal_handle_ ||
          nav_goal_state_ != NavGoalState::CANCELLING)
        {
          return;
        }
        if (response &&
          response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE)
        {
          return;
        }
        RCLCPP_WARN(this->get_logger(), "Nav2 goal cancellation was rejected.");
        nav_goal_state_ = NavGoalState::ACTIVE;
      });
  }

  void sendGoal(
    const double goal_x, const double goal_y, const rclcpp::Time & now)
  {
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = planning_frame_;
    goal.pose.header.stamp = now;
    goal.pose.pose.position.x = goal_x;
    goal.pose.pose.position.y = goal_y;
    const double heading = std::atan2(
      goal_y - robot_.pose.pose.position.y,
      goal_x - robot_.pose.pose.position.x);
    goal.pose.pose.orientation.z = std::sin(0.5 * heading);
    goal.pose.pose.orientation.w = std::cos(0.5 * heading);

    nav_goal_state_ = NavGoalState::REQUESTING;
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & goal_handle) {
        if (nav_goal_state_ != NavGoalState::REQUESTING) {
          return;
        }
        if (goal_handle) {
          goal_handle_ = goal_handle;
        } else {
          RCLCPP_WARN(this->get_logger(), "Nav2 rejected the interception goal.");
        }
        nav_goal_state_ =
          goal_handle_ ? NavGoalState::ACTIVE : NavGoalState::IDLE;

        if (desired_owner_ != robot_navigation::CommandOwner::NAV2) {
          cancelNavigation(desired_owner_);
          return;
        }
        publishOwner(
          nav_goal_state_ == NavGoalState::ACTIVE ?
          robot_navigation::CommandOwner::NAV2 :
          robot_navigation::CommandOwner::STOP);
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        if (!goal_handle_ || result.goal_id != goal_handle_->get_goal_id()) {
          return;
        }

        goal_handle_.reset();
        if (nav_goal_state_ == NavGoalState::REQUESTING) {
          // A rolling replacement may still be waiting for its goal response.
          return;
        }
        const bool was_cancelling = nav_goal_state_ == NavGoalState::CANCELLING;
        nav_goal_state_ = NavGoalState::IDLE;
        if (was_cancelling) {
          last_goal_time_.reset();
        }
        publishOwner(
          desired_owner_ == robot_navigation::CommandOwner::NAV2 ?
          robot_navigation::CommandOwner::STOP : desired_owner_);
      };
    navigate_client_->async_send_goal(goal, options);
    last_goal_x_ = goal_x;
    last_goal_y_ = goal_y;
    last_goal_time_ = now;
  }

  void update()
  {
    const rclcpp::Time now = this->now();
    if (!have_robot_ || !have_target_) {
      terminal_active_ = false;
      cancelNavigation(robot_navigation::CommandOwner::STOP);
      return;
    }

    const double target_age = (now - rclcpp::Time(target_.header.stamp)).seconds();
    const double odometry_age = (now - rclcpp::Time(robot_.header.stamp)).seconds();
    if (
      !robot_navigation::inputIsFresh(true, target_age, target_timeout_) ||
      !robot_navigation::inputIsFresh(true, odometry_age, odometry_timeout_))
    {
      terminal_active_ = false;
      cancelNavigation(robot_navigation::CommandOwner::STOP);
      return;
    }

    const double target_x =
      target_.pose.pose.position.x + target_.twist.twist.linear.x * target_age;
    const double target_y =
      target_.pose.pose.position.y + target_.twist.twist.linear.y * target_age;
    const double relative_x = target_x - robot_.pose.pose.position.x;
    const double relative_y = target_y - robot_.pose.pose.position.y;
    const double distance = std::hypot(relative_x, relative_y);
    if (distance <= capture_radius_) {
      terminal_active_ = false;
      cancelNavigation(robot_navigation::CommandOwner::STOP);
      return;
    }

    terminal_active_ = robot_navigation::updateTerminalState(
      terminal_active_, distance, terminal_enter_distance_, terminal_exit_distance_);
    if (terminal_active_) {
      cancelNavigation(robot_navigation::CommandOwner::TERMINAL);
      return;
    }

    desired_owner_ = robot_navigation::CommandOwner::NAV2;
    if (
      (nav_goal_state_ != NavGoalState::IDLE &&
      nav_goal_state_ != NavGoalState::ACTIVE) ||
      !navigate_client_->action_server_is_ready())
    {
      publishOwner(robot_navigation::CommandOwner::STOP);
      return;
    }

    const auto intercept_time = robot_navigation::solveInterceptTime(
      relative_x, relative_y,
      target_.twist.twist.linear.x, target_.twist.twist.linear.y,
      intercept_speed_);
    if (!intercept_time || *intercept_time > max_intercept_time_) {
      cancelNavigation(robot_navigation::CommandOwner::STOP);
      return;
    }

    const double goal_x =
      target_x + target_.twist.twist.linear.x * *intercept_time;
    const double goal_y =
      target_y + target_.twist.twist.linear.y * *intercept_time;
    const double elapsed =
      last_goal_time_ ? (now - *last_goal_time_).seconds() : 0.0;
    const double displacement = std::hypot(
      goal_x - last_goal_x_, goal_y - last_goal_y_);
    if (!last_goal_time_ || robot_navigation::shouldRefreshGoal(
        nav_goal_state_ == NavGoalState::ACTIVE, elapsed, displacement,
        goal_refresh_period_, goal_refresh_distance_))
    {
      sendGoal(goal_x, goal_y, now);
    }
    publishOwner(
      nav_goal_state_ == NavGoalState::ACTIVE ?
      robot_navigation::CommandOwner::NAV2 :
      robot_navigation::CommandOwner::STOP);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr owner_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Odometry robot_;
  nav_msgs::msg::Odometry target_;
  std::optional<rclcpp::Time> last_goal_time_;
  GoalHandle::SharedPtr goal_handle_;
  std::string odometry_topic_;
  std::string target_topic_;
  std::string owner_topic_;
  std::string action_name_;
  std::string planning_frame_;
  robot_navigation::CommandOwner published_owner_ =
    robot_navigation::CommandOwner::TERMINAL;
  robot_navigation::CommandOwner desired_owner_ =
    robot_navigation::CommandOwner::STOP;
  NavGoalState nav_goal_state_ = NavGoalState::IDLE;
  double intercept_speed_;
  double max_intercept_time_;
  double terminal_enter_distance_;
  double terminal_exit_distance_;
  double capture_radius_;
  double target_timeout_;
  double odometry_timeout_;
  double goal_refresh_period_;
  double goal_refresh_distance_;
  double control_rate_;
  double last_goal_x_ = 0.0;
  double last_goal_y_ = 0.0;
  bool have_robot_ = false;
  bool have_target_ = false;
  bool terminal_active_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InterceptionSupervisor>());
  rclcpp::shutdown();
  return 0;
}
