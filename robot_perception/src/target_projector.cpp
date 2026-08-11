#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "robot_interfaces/msg/target_detection2_d.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class TargetProjector : public rclcpp::Node
{
public:
  using Detection = robot_interfaces::msg::TargetDetection2D;
  using DepthImage = sensor_msgs::msg::Image;
  using Odometry = nav_msgs::msg::Odometry;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<Detection, DepthImage, Odometry>;

  TargetProjector()
  : Node("target_projector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    detection_topic_ = this->declare_parameter<std::string>(
      "detection_topic", "/perception/target_detection");
    depth_topic_ = this->declare_parameter<std::string>(
      "depth_topic", "/camera/depth_image");
    camera_info_topic_ = this->declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera_info");
    odometry_topic_ = this->declare_parameter<std::string>(
      "odometry_topic", "/ground_truth/odom");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/perception/target_position");
    camera_frame_ = this->declare_parameter<std::string>(
      "camera_frame", "camera_optical_frame");
    base_frame_ = this->declare_parameter<std::string>(
      "base_frame", "base_footprint");
    output_frame_ = this->declare_parameter<std::string>("output_frame", "odom");
    min_depth_ = this->declare_parameter<double>("min_depth", 0.1);
    max_depth_ = this->declare_parameter<double>("max_depth", 20.0);
    target_radius_ = this->declare_parameter<double>("target_radius", 0.2);
    sample_fraction_ = this->declare_parameter<double>("sample_fraction", 0.5);
    depth_variance_base_ = this->declare_parameter<double>(
      "depth_variance_base", 0.0025);
    depth_variance_scale_ = this->declare_parameter<double>(
      "depth_variance_scale", 0.001);
    pixel_variance_ = this->declare_parameter<double>("pixel_variance", 1.0);
    sync_tolerance_ = this->declare_parameter<double>("sync_tolerance", 0.05);
    transform_timeout_ = this->declare_parameter<double>("transform_timeout", 0.05);
    if (
      min_depth_ <= 0.0 || max_depth_ <= min_depth_ || target_radius_ < 0.0 ||
      sample_fraction_ <= 0.0 || depth_variance_base_ <= 0.0 ||
      depth_variance_scale_ < 0.0 || pixel_variance_ <= 0.0 ||
      sync_tolerance_ <= 0.0 || transform_timeout_ <= 0.0)
    {
      throw std::invalid_argument("Invalid target projection parameters.");
    }

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TargetProjector::cameraInfoCallback, this, std::placeholders::_1));

    detection_sub_.subscribe(this, detection_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
    odometry_sub_.subscribe(this, odometry_topic_, rmw_qos_profile_sensor_data);
    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(20), detection_sub_, depth_sub_, odometry_sub_);
    synchronizer_->registerCallback(
      std::bind(
        &TargetProjector::measurementCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic_, 10);
  }

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring camera info with invalid focal lengths.");
      return;
    }
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    have_camera_info_ = true;
  }

  double medianDepth(
    const Detection & detection,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg) const
  {
    cv_bridge::CvImageConstPtr depth;
    try {
      depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
    } catch (const cv_bridge::Exception &) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const double fraction = std::max(0.1, std::min(sample_fraction_, 1.0));
    const int half_width = std::max(
      1, static_cast<int>(0.5 * fraction * detection.width));
    const int half_height = std::max(
      1, static_cast<int>(0.5 * fraction * detection.height));
    const int center_u = static_cast<int>(std::round(detection.center_u));
    const int center_v = static_cast<int>(std::round(detection.center_v));
    const int min_u = std::max(0, center_u - half_width);
    const int max_u = std::min(depth->image.cols - 1, center_u + half_width);
    const int min_v = std::max(0, center_v - half_height);
    const int max_v = std::min(depth->image.rows - 1, center_v + half_height);

    std::vector<double> samples;
    samples.reserve(
      static_cast<std::size_t>((max_u - min_u + 1) * (max_v - min_v + 1)));
    for (int v = min_v; v <= max_v; ++v) {
      for (int u = min_u; u <= max_u; ++u) {
        double value = std::numeric_limits<double>::quiet_NaN();
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
          value = depth->image.at<float>(v, u);
        } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
          value = 0.001 * depth->image.at<std::uint16_t>(v, u);
        } else {
          return std::numeric_limits<double>::quiet_NaN();
        }
        if (std::isfinite(value) && value >= min_depth_ && value <= max_depth_) {
          samples.push_back(value);
        }
      }
    }

    if (samples.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const auto middle = samples.begin() + samples.size() / 2;
    std::nth_element(samples.begin(), middle, samples.end());
    return *middle;
  }

  void measurementCallback(
    const Detection::ConstSharedPtr & detection,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const Odometry::ConstSharedPtr & odometry)
  {
    if (!have_camera_info_ || odometry->header.frame_id != output_frame_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for camera intrinsics or odometry in the configured output frame.");
      return;
    }
    const double depth_delta = std::abs(
      (rclcpp::Time(detection->header.stamp) -
      rclcpp::Time(depth_msg->header.stamp)).seconds());
    const double odometry_delta = std::abs(
      (rclcpp::Time(detection->header.stamp) -
      rclcpp::Time(odometry->header.stamp)).seconds());
    if (depth_delta > sync_tolerance_ || odometry_delta > sync_tolerance_) {
      return;
    }

    const double depth = medianDepth(*detection, depth_msg);
    if (!std::isfinite(depth)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No valid target depth samples.");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        base_frame_, camera_frame_, tf2::TimePointZero,
        tf2::durationFromSec(transform_timeout_));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Cannot resolve camera extrinsic from %s to %s: %s",
        camera_frame_.c_str(), base_frame_.c_str(), error.what());
      return;
    }

    const double normalized_x = (detection->center_u - cx_) / fx_;
    const double normalized_y = (detection->center_v - cy_) / fy_;
    const tf2::Vector3 target_surface_camera(
      normalized_x * depth, normalized_y * depth, depth);
    const tf2::Vector3 target_camera =
      target_surface_camera +
      target_radius_ * target_surface_camera.normalized();
    tf2::Quaternion base_from_camera_rotation;
    tf2::fromMsg(transform.transform.rotation, base_from_camera_rotation);
    const tf2::Vector3 base_from_camera_origin(
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z);
    const tf2::Vector3 target_base =
      base_from_camera_origin +
      tf2::quatRotate(base_from_camera_rotation, target_camera);

    tf2::Transform output_from_base;
    tf2::fromMsg(odometry->pose.pose, output_from_base);
    const tf2::Vector3 target_output = output_from_base * target_base;

    const double depth_variance =
      depth_variance_base_ + depth_variance_scale_ * depth * depth;
    const double x_variance =
      depth * depth * pixel_variance_ / (fx_ * fx_) +
      normalized_x * normalized_x * depth_variance;
    const double y_variance =
      depth * depth * pixel_variance_ / (fy_ * fy_) +
      normalized_y * normalized_y * depth_variance;
    const double conservative_variance =
      std::max(depth_variance, std::max(x_variance, y_variance));

    geometry_msgs::msg::PoseWithCovarianceStamped output;
    output.header.stamp = detection->header.stamp;
    output.header.frame_id = output_frame_;
    output.pose.pose.position.x = target_output.x();
    output.pose.pose.position.y = target_output.y();
    output.pose.pose.position.z = target_output.z();
    output.pose.pose.orientation.w = 1.0;
    output.pose.covariance[0] = conservative_variance;
    output.pose.covariance[7] = conservative_variance;
    output.pose.covariance[14] = conservative_variance;
    output.pose.covariance[21] = 1.0e6;
    output.pose.covariance[28] = 1.0e6;
    output.pose.covariance[35] = 1.0e6;
    target_pub_->publish(output);
  }

  message_filters::Subscriber<Detection> detection_sub_;
  message_filters::Subscriber<DepthImage> depth_sub_;
  message_filters::Subscriber<Odometry> odometry_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr target_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string detection_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string odometry_topic_;
  std::string output_topic_;
  std::string camera_frame_;
  std::string base_frame_;
  std::string output_frame_;
  double min_depth_;
  double max_depth_;
  double target_radius_;
  double sample_fraction_;
  double depth_variance_base_;
  double depth_variance_scale_;
  double pixel_variance_;
  double sync_tolerance_;
  double transform_timeout_;
  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;
  bool have_camera_info_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetProjector>());
  rclcpp::shutdown();
  return 0;
}
