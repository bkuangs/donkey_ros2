#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "robot_interfaces/msg/target_detection2_d.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ColorDetection : public rclcpp::Node
{
public:
  ColorDetection()
  : Node("color_detection")
  {
    target_topic_ = this->declare_parameter<std::string>(
      "target_topic", "/perception/target_detection");
    min_contour_area_ = this->declare_parameter<double>("min_contour_area", 100.0);
    minimum_saturation_ = this->declare_parameter<int>("minimum_saturation", 100);
    minimum_value_ = this->declare_parameter<int>("minimum_value", 100);
    low_red_maximum_hue_ = this->declare_parameter<int>("low_red_maximum_hue", 10);
    high_red_minimum_hue_ = this->declare_parameter<int>("high_red_minimum_hue", 170);
    if (
      min_contour_area_ <= 0.0 ||
      minimum_saturation_ < 0 || minimum_saturation_ > 255 ||
      minimum_value_ < 0 || minimum_value_ > 255 ||
      low_red_maximum_hue_ < 0 || low_red_maximum_hue_ > 180 ||
      high_red_minimum_hue_ < 0 || high_red_minimum_hue_ > 180 ||
      low_red_maximum_hue_ >= high_red_minimum_hue_)
    {
      throw std::invalid_argument("Invalid HSV target detector parameters.");
    }

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ColorDetection::imageCallback, this, std::placeholders::_1));
    target_pub_ =
      this->create_publisher<robot_interfaces::msg::TargetDetection2D>(
      target_topic_, 10);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", error.what());
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(cv_image->image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat low_red;
    cv::Mat high_red;
    cv::Mat mask;
    cv::inRange(
      hsv,
      cv::Scalar(0, minimum_saturation_, minimum_value_),
      cv::Scalar(low_red_maximum_hue_, 255, 255),
      low_red);
    cv::inRange(
      hsv,
      cv::Scalar(high_red_minimum_hue_, minimum_saturation_, minimum_value_),
      cv::Scalar(180, 255, 255),
      high_red);
    mask = low_red | high_red;
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
      return;
    }

    const auto largest = std::max_element(
      contours.begin(), contours.end(),
      [](const auto & left, const auto & right) {
        return cv::contourArea(left) < cv::contourArea(right);
      });
    const double area = cv::contourArea(*largest);
    if (area < min_contour_area_) {
      return;
    }

    const cv::Rect bbox = cv::boundingRect(*largest);
    robot_interfaces::msg::TargetDetection2D detection;
    detection.header = msg->header;
    detection.center_u = bbox.x + 0.5 * bbox.width;
    detection.center_v = bbox.y + 0.5 * bbox.height;
    detection.width = bbox.width;
    detection.height = bbox.height;
    detection.confidence = 1.0;
    target_pub_->publish(detection);
  }

  std::string target_topic_;
  double min_contour_area_;
  int minimum_saturation_;
  int minimum_value_;
  int low_red_maximum_hue_;
  int high_red_minimum_hue_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<robot_interfaces::msg::TargetDetection2D>::SharedPtr target_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorDetection>());
  rclcpp::shutdown();
  return 0;
}
