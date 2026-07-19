#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Thin adapter: subscribes to OpenVINS odometry output and republishes it as
// /vio/odom in a form the robot_localization EKF can fuse (odom1).
//
// OpenVINS (ov_msckf) publishes its estimate on /ov_msckf/odomimu by default.
// This node remaps frames and stamps a covariance so the EKF trusts VIO
// appropriately relative to wheel odom + IMU.
class VisualOdom : public rclcpp::Node
{
public:
  VisualOdom()
  : Node("vio_adapter")
  {
    // Params
    in_topic_          = this->declare_parameter<std::string>("vio_in_topic", "/ov_msckf/odomimu");
    out_topic_         = this->declare_parameter<std::string>("vio_out_topic", "/vio/odom");
    output_frame_      = this->declare_parameter<std::string>("output_frame", "odom");
    child_frame_       = this->declare_parameter<std::string>("child_frame", "base_link");
    // Diagonal covariance values used if the incoming message has none.
    pos_variance_      = this->declare_parameter<double>("position_variance", 0.05);
    yaw_variance_      = this->declare_parameter<double>("yaw_variance", 0.02);
    vel_variance_      = this->declare_parameter<double>("velocity_variance", 0.10);
    override_cov_      = this->declare_parameter<bool>("override_covariance", true);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      in_topic_, 10,
      std::bind(&VisualOdom::odomCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(out_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
      "VIO adapter: %s -> %s (frame=%s, child=%s)",
      in_topic_.c_str(), out_topic_.c_str(),
      output_frame_.c_str(), child_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry out = *msg;

    // Normalize frames so the EKF accepts this as an odom source.
    out.header.frame_id = output_frame_;
    out.child_frame_id  = child_frame_;

    if (override_cov_) {
      // Pose covariance (6x6 row-major): x,y,z,roll,pitch,yaw
      out.pose.covariance.fill(0.0);
      out.pose.covariance[0]  = pos_variance_;   // x
      out.pose.covariance[7]  = pos_variance_;   // y
      out.pose.covariance[14] = 1e6;             // z (unused in 2D)
      out.pose.covariance[21] = 1e6;             // roll
      out.pose.covariance[28] = 1e6;             // pitch
      out.pose.covariance[35] = yaw_variance_;   // yaw

      // Twist covariance: vx,vy,vz,wx,wy,wz
      out.twist.covariance.fill(0.0);
      out.twist.covariance[0]  = vel_variance_;  // vx
      out.twist.covariance[7]  = vel_variance_;  // vy
      out.twist.covariance[14] = 1e6;            // vz
      out.twist.covariance[21] = 1e6;            // wx
      out.twist.covariance[28] = 1e6;            // wy
      out.twist.covariance[35] = yaw_variance_;  // wz (yaw rate)
    }

    pub_->publish(out);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  std::string in_topic_, out_topic_, output_frame_, child_frame_;
  double pos_variance_, yaw_variance_, vel_variance_;
  bool override_cov_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualOdom>());
  rclcpp::shutdown();
  return 0;
}