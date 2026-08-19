import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('robot_navigation')
    config = os.path.join(package_share, 'config', 'navigation.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    odometry_topic = LaunchConfiguration('odometry_topic')
    common_parameters = [
        config,
        {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
    ]
    interception_parameters = [
        *common_parameters,
        {'odometry_topic': odometry_topic},
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'odometry_topic', default_value='/ground_truth/odom'
        ),
        Node(
            package='robot_navigation',
            executable='intercept_controller',
            parameters=interception_parameters,
            output='screen',
        ),
        Node(
            package='robot_navigation',
            executable='command_arbiter',
            parameters=common_parameters,
            output='screen',
        ),
        Node(
            package='robot_navigation',
            executable='interception_supervisor',
            parameters=interception_parameters,
            output='screen',
        ),
    ])
