import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robot_navigation")
    config = os.path.join(package_share, "config", "navigation.yaml")
    behavior_tree = os.path.join(
        package_share,
        "behavior_trees",
        "intercept_replanning.xml",
    )
    default_map = os.path.join(package_share, "maps", "intercept_arena.yaml")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    common_parameters = [config, {"use_sim_time": use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map", default_value=default_map),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[
                config,
                {
                    "yaml_filename": map_yaml,
                    "use_sim_time": use_sim_time,
                },
            ],
            output="screen",
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            parameters=common_parameters,
            output="screen",
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            parameters=common_parameters,
            remappings=[("cmd_vel", "/cmd_vel")],
            output="screen",
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            parameters=[
                config,
                {
                    "default_nav_to_pose_bt_xml": behavior_tree,
                    "use_sim_time": use_sim_time,
                },
            ],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[{
                "autostart": True,
                "bond_timeout": 4.0,
                "node_names": [
                    "map_server",
                    "planner_server",
                    "controller_server",
                    "bt_navigator",
                ],
                "use_sim_time": use_sim_time,
            }],
            output="screen",
        ),
        Node(
            package="robot_navigation",
            executable="intercept_guidance",
            parameters=common_parameters,
            output="screen",
        ),
        Node(
            package="robot_navigation",
            executable="nav2_goal_manager",
            parameters=common_parameters,
            output="screen",
        ),
    ])
