import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_tracking')
    config_file = os.path.join(pkg_share, 'config', 'target_ekf.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from /clock',
        ),
        Node(
            package='robot_tracking',
            executable='target_ekf',
            name='target_ekf',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        )
    ])
