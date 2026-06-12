#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"


/*
subscribe to raw camera images,
convert to openCV cv::Mat,
convert rgb -> hsv,
build a red color mask
find contours and choose largest
compute centroid of that contour using moments
publish point stamped
*/
class ColorDetection : public rclcpp::Node
{
public: 
    ColorDetection()
    : Node("color_detection")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&ColorDetection::image_callback, this, std::placeholders::_1)
        );

        target_pub_ = this->create_publisher<geometry_msgs::PointStamped>(
            "/perception/target_centroid",
            10
        );

        RCLCPP_INFO(this->get_logger(), "Color detection started.");
    }

private:
    // sensor_msgs/Image <-> cv::Mat
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); // deep copy image data from msg
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge_error: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image; // access image info

        // RGB to HSV
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // red target
        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;

        // clean up noise
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        // find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "No target detected.");
            return;
        }

        // pick largest contour
        auto largest = std::max_element(
            contours.begin(),
            contours.end(),
            [](const auto & a, const auto & b) {
                return cv::contourArea(a) < cv::contourArea(b);
            }
        );

        double area = cv::contourArea(*largest);

        // ignore tiny ones
        if (area < 100.0)
            return;

        cv::Moments m = cv::moments(*largest);
        if (m.m00 == 0.0)
            return;

        // define pixel coordinates of centroid
        double cx = m.m10 / m.m00;
        double cy = m.m01 / m.m00;

        // publish
        geometry_msgs::msg::PointStamped target_msg;
        target_msg.header = msg->header;
        target_msg.point.x = cx;
        target_msg.point.y = cy;
        target_msg.point.z = area;  // z is the area of the contour, not depth

        target_pub_->publish(target_msg);

        RCLCPP_INFO(this->get_logger(), 
        "Target detected at x=%.1f, y=%.1f, area=%.1f",
        cx, cy, area);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorDetection>());
  rclcpp::shutdown();
  return 0;
}