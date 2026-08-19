from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    use_sim_time_parameter = ParameterValue(use_sim_time, value_type=bool)

    config = PathJoinSubstitution(
        [FindPackageShare("robot_odometry"), "config", "rgbd_odometry.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth_image"),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera_info"
            ),
            DeclareLaunchArgument(
                "odom_topic", default_value="/localization/odom"
            ),
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="rgbd_odometry",
                output="screen",
                parameters=[config, {"use_sim_time": use_sim_time_parameter}],
                remappings=[
                    ("rgb/image", rgb_topic),
                    ("depth/image", depth_topic),
                    ("rgb/camera_info", camera_info_topic),
                    ("odom", odom_topic),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time_parameter}],
                arguments=[
                    "--x",
                    initial_x,
                    "--y",
                    initial_y,
                    "--z",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    initial_yaw,
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "odom",
                ],
            ),
        ]
    )
