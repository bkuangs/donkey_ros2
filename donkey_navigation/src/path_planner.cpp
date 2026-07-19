#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace std::chrono_literals;

class PathPlanner : public rclcpp::Node
{
public:
    PathPlanner()
        : Node("path_planner")
    {
        // params
        follow_distance_ = this->declare_parameter<double>("follow_distance", 1.5);
        prediction_time_ = this->declare_parameter<double>("prediction_time", 0.5);
        num_path_points_ = this->declare_parameter<int>("num_path_points", 10);
        publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
        stale_timeout_sec_ = this->declare_parameter<double>("stale_timeout_sec", 0.5);
        min_target_distance = this->declare_parameter<double>("min_target_distance", 0.5);

        if (num_path_points_ < 2)
        {
            RCLCPP_WARN(this->get_logger(), "num_path_points < 2; clamping to 2");
            num_path_points_ = 2;
        }

        odom_sub_ = this->create_subscription(
            "/odom",
            10,
            std::bind(&PathPlanner::OdomCallback, this, std::placeholders::_1));

        target_sub_ = this->create_subscription(
            "/tracking/target_state",
            10,
            std::bind(&PathPlanner::TargetStateCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/nav/path", 10);

        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&PathPlanner::PublishPath, this));

        RCLCPP_INFO(this->get_logger(), "Path planner node started.");
    }

private:
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_pose_ = msg->pose.pose;
        have_robot_pose_ = true;
    }

    void TargetStateCallback(const geometry_msgs::msg::Image::SharedPtr msg)
    {
        target_pose_ = msg->pose;
        have_target_ = true;
        last_target_stamp_ = this->now();
    }

    static Eigen::Vector3d toVec(const geometry_msgs::msg::Point &p)
    {
        reutrn Eigen::Vector3d(p.x, p.y, p.z);
    }

    void PublishPath()
    {
        if (!have_robot_path_ || !have_target_)
            return;

        // drop stale target data
        if ((this->now() - last_target_stamp_).seconds() > stale_timeout_sec_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Target state stale, not publishing path.");
            return;
        }

        const Eigen::Vector3d p_robot = toVec(robot_pose_.position);
        const Eigen::Vector3d p_target = toVec(target_pose_.position);

        Eigen::Vector3d delta = p_robot - p_target;
        const double dist = delta.norm();
        if (dist < 1e-6)
            return; // robot on top of target
        const Eigen::Vector3d dir = delta / dist;

        const double follow = std::max(follow_distance_, min_target_distance_);
        const Eigen::Vector3d goal = p_target + follow * dir;

        // interpolated straight-line path from the robot to goal
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "odom";

        path.poses.reserve(num_path_points_);
        for (int i = 0; i < num_path_points_; ++i)
        {
            const double t = static_cast<double>(i) / (num_path_points_ - 1);
            const Eigen::Vector3d pt = p_robot + t * (goal - p_robot);

            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = pt.x();
            ps.pose.position.y = pt.y();
            ps.pose.position.z = pt.z();
            ps.pose.orientation.w = 1.0; // no heading interpolation for now
            path.poses.push_back(ps);
        }

        path_pub_->publish(path);
    }

    // params
    double follow_distance_{1.5};
    double prediction_time_{0.5};
    int num_path_points_{10};
    double publish_rate_hz_{10.0};
    double stale_timeout_sec_{0.5};
    double min_target_distance_{0.5};

    // state
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::Pose target_pose_;
    bool have_robot_pose_{false};
    bool have_target_{false};
    rclcpp::Time last_target_stamp_;

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}