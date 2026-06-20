#include

class PathPlanner : public rclcpp::Node
{
public:
    follow_distance_ = this->declare_parameter<double>("follow_distance", 1.5);
    prediction_time_ = this->declare_parameter<double>("prediction_time", 0.5);
    num_path_points_ = this->declare_parameter<int>("num_path_points", 10);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
    stale_timeout_sec_ = this->declare_parameter<double>("stale_timeout_sec", 0.5);
    min_target_distance = this->declare_parameter<double>("min_target_distance", 0.5);

    odom_sub_ = this->create_subscription(
        "/odom",
        10,
        std::bind(&PathPlanner::OdomCallback, this, std::placeholders::_1));

    target_sub_ = this->create_subscription(
        "/tracking/target_state",
        10,
        std::bind(&PathPlanner::TargetStateCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/nav/path",
        10);

    timer_ = this->create_wall_timer(
        500ms, std::bind(&PathPlanner::PathCallback, this, std::placeholders::_1));

private:
    void OdomCallback()
    {

    }

    void TargetStateCallback(const geometry_msgs::msg::Image::SharedPtr msg)
    {
        geometry_msgs::msg::Pose p_robot = 
        geometry_msgs::msg::Pose p_target = msg->pose;
        double dist = 2.0;

        Eigen::Vector3d dir = (p_robot - p_target) / (p_target - p_robot).dot(p_target - p_robot);

        geometry_msgs::msg::Pose goal = p_target - (dist * dir);

        // interpolated straight line path
        std::vector<geometry_msgs::msg::Pose> poses;
        for (int i = 0; i < 3; ++i) {
            poses.push_back(msg->pose + (0.25 * i) * dir);
        }
    }

    odom_sub_ = std::make_shared();
}

int main(argc, argv)
{
    return 0;
}