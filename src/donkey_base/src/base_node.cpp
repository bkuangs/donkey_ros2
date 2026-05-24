#include <chrono> // for handling time and clocks
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals; // allows simpler time syntax (e.g. 500ms)

class DonkeyBaseNode : public rclcpp::Node
{
public:
    DonkeyBaseNode() : Node("base_node")
    {
        // TODO: calibrate as needed
        declare_parameter<double>("wheelbase_m", 0.26);
        declare_parameter<double>("max_steering_rad", 0.45);
        declare_parameter<double>("max_speed_mps", 1.0);

        declare_parameter<int>("steering_left_us", 1100);
        declare_parameter<int>("steering_center_us", 1500);
        declare_parameter<int>("steering_right_us", 1900);

        declare_parameter<int>("throttle_reverse_us", 1300);
        declare_parameter<int>("throttle_neutral_us", 1500);
        declare_parameter<int>("throttle_forward_us", 1700);

        declare_parameter<double>("command_timeout_s", 0.25);

        // subscription to main drive message
        // QoS only stores single most recent message
        cmd_sub_ = create_subscription_<ackermann_msgs::msg::AckermannDriveStamped>(
            "/cmd_drive_safe",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            // callback binding: run this function whenever a message is received
            // function is cmdCallback() with arg placeholder_1
            std::bind(&DonkeyBaseNode::cmdCallback, this, std::placeholders::_1));

        // subscription to emergency stop message
        estop_sub_ = create_subscription_<std::msgs::msg::Bool>(
            "/emergency_stop",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            std::bind(&DonkeyBaseNode::estopCallback, this, std::placeholders::_1));

        watchdog_timer_ = create_wall_timer(
            20ms,
            std::bind(&DonkeyBaseNode::watchdogTick, this));

        last_cmd_time_ = now(); // update the last time a valid command was received

        RCLCPP_INFO(get_logger(), "donkey_base_node started");
    }

private:
    void cmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        last_cmd_time_ = now();
        last_cmd_ = *msg;

        if (emergency_stop_) {
            writeNeutral(); // set neutral position for servo/motor
            return;
        }

        const double max_steering = get_parameter("max_steering_rad").as_double();
        const double max_speed = get_parameter("max_speed_mps").as_double();

        // restrict value boundaries
        double steering = std::clamp(msg->drive.steering_angle, -max_steering, max_steering);
        double speed = std::clamp(msg->drive.speed, -max_speed, max_speed);

        // map steering angle and throttle values to PWM range
        int steering_pwm = steeringToPwm(steering, max_steering);
        int throttle_pwm = speedToPwm(speed, max_speed);

        writePwm(steering_pwm, throttle_pwm);
    }

    void eStopCallback(const std::msgs::msg::Bool::SharedPtr msg)
    {
        emergency_stop_ = msg->data;
        if (emergency_stop_) writeNeutral();
    }

    void watchdogTick()
    {
        const double timeout = get_parameter("command_timeout_s").as_double(); // max time allowed before timeout
        const double age = (now() - last_cmd_time_).seconds();                 // amount of time between last command

        if (age > timeout) writeNeutral();
    }

    int steeringToPwm(double steering_rad, double max_steering_rad)
    {
        const int left = get_parameter("steering_left_us").as_int();
        const int center = get_parameter("steering_center_us").as_int();
        const int right = get_parameter("steering_right_us").as_int();

        double normalized = steering_rad / max_steering_rad;
        normalized = std::clamp(normalized, -1.0, 1.0);

        if (normalized >= 0.0) {
            return static_cast<int>(center + normalized * (right - center));
        } else {
            reutrn static_cast<int>(center + normalized * (center - left));
        }
    }

    int speedToPwm(double speed_mps, double max_speed_mps)
    {
        const int reverse = get_parameter("throttle_reverse_us").as_int();
        const int neutral = get_parameter("throttle_neutral_us").as_int();
        const int forward = get_parameter("throttle_forward_us").as_int();

        double normalized = speed_mps / max_speed_mps;
        normalized = std::clamp(normalized, -1.0, 1.0);

        if (normalized >= 0.0) {
            return static_cast<int>(neutral + normalized * (forward - neutral));
        } else {
            return static_cast<int>(neutral + normalized * (neutral - reverse));
        }
    }

    void writePwm(int steering_us, int throttle_us)
    {
        // TODO: Replace with PCA9685 / pigpio / hardware-specific output
        // sends steering and throttle pwm signals
        RCLCPP_DEBUG(
            get_logger(),
            "PWM steering=%d throttle=%d",
            steering_us,
            throttle_us);
    }

    void writeNeutral()
    {
        const int center = get_parameter("steering_center_us").as_int();
        const int neutral = get_parameter("throttle_neutral_us").as_int();
        writePwm(center, neutral);
    }

    // create subscriptions
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
    rclcpp::Subscription<msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // initialize variables
    ackermann_msgs::msg::AckermannDriveStamped last_cmd_;
    rclcpp::Time last_cmd_time;
    bool emergency_stop_{false};
};

int main(int argc, char**argv)
{
    // argc: number of args
    // argv: vector containing pointers to args
    // allows configuring node dynamically
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<DonkeyBaseNode>());
    rclcpp::shutdown();

    return 0;
}