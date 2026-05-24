from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("donkey_bringup"),
        "config",
        "base.yaml"
    )

    return LaunchDescription([
        Node(
            package="donkey_base",
            executable="base_node",
            name="base_node",
            parameters=[config],
            output="screen"
        )
    ])