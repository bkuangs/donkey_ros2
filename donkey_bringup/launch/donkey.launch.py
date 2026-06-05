from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

base_config = os.path.join(
    get_package_share_directory("donkey_bringup"),
    "config",
    "calibrate.yaml"
)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="donkey_control",
            executable="heartbeat_node"
        ),

        Node(
            package="donkey_description",
            executable="robot_state_publisher"
        ),

        Node(
            package="donkey_base",
            executable="base_node",
            name="base_node",
            parameters=[base_config],
            output="screen"
        )
    ])