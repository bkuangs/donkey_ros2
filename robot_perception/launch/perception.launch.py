import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("robot_perception"),
        "config",
        "target_projection.yaml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    odometry_topic = LaunchConfiguration("odometry_topic")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "odometry_topic", default_value="/ground_truth/odom"
        ),
        Node(
            package="robot_perception",
            executable="color_detection",
            parameters=[config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="robot_perception",
            executable="target_projector",
            parameters=[
                config,
                {
                    "odometry_topic": odometry_topic,
                    "use_sim_time": use_sim_time,
                },
            ],
            output="screen",
        ),
    ])
