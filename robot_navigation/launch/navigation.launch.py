import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('robot_navigation')
    config = os.path.join(package_share, 'config', 'navigation.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='robot_navigation',
            executable='intercept_controller',
            parameters=[config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
