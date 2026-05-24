from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("donkey_control"),
        "config",
        "cmd_mux.yaml",
    )

    command_mux = Node(
        package="donkey_control",
        executable="command_mux",
        name="command_mux",
        output="screen",
        parameters=[config_file, {"use_sim_time": True}],
    )

    twist_to_ack = Node(
        package="donkey_control",
        executable="twist_to_ack",
        name="twist_to_ack",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_topic": "/cmd_vel",
            "output_topic": "/cmd/teleop",
            "frame_id": "base_link",
            "wheelbase": 0.26,
            "max_speed": 1.0,
            "max_reverse_speed": 1.0,
            "max_steering_angle": 0.6,
        }],
    )

    ack_to_joint_cmd = Node(
        package="donkey_control",
        executable="ack_to_joint_cmd",
        name="ack_to_joint_cmd",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_topic": "/drive_cmd",
            "steering_topic": "/front_steer_controller/commands",
            "wheel_topic": "/rear_wheel_controller/commands",
            "wheel_radius": 0.033,
            "max_speed": 1.0,
            "max_reverse_speed": 1.0,
            "max_steering_angle": 0.6,
        }],
    )

    return LaunchDescription([
        command_mux,
        twist_to_ack,
        ack_to_joint_cmd,
    ])
