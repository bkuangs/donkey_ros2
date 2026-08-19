import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("robot_nav2")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    odometry_topic = LaunchConfiguration("odometry_topic")
    use_ground_truth_tf = LaunchConfiguration("use_ground_truth_tf")
    use_sim_time_parameter = ParameterValue(use_sim_time, value_type=bool)
    behavior_tree = os.path.join(
        package_share,
        "behavior_trees",
        "navigate_to_pose_ackermann.xml",
    )

    common_parameters = [params_file, {"use_sim_time": use_sim_time_parameter}]
    lifecycle_nodes = [
        "map_server",
        "planner_server",
        "controller_server",
        "bt_navigator",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "odometry_topic", default_value="/ground_truth/odom"
            ),
            DeclareLaunchArgument("use_ground_truth_tf", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_share, "config", "nav2.yaml"),
            ),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(package_share, "maps", "arena.yaml"),
            ),
            Node(
                package="robot_nav2",
                executable="ground_truth_tf",
                name="ground_truth_tf",
                output="screen",
                parameters=common_parameters,
                condition=IfCondition(use_ground_truth_tf),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "yaml_filename": ParameterValue(map_file, value_type=str),
                        "use_sim_time": use_sim_time_parameter,
                    },
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=common_parameters,
                remappings=[
                    ("cmd_vel", "/cmd_vel/nav2"),
                    ("odom", odometry_topic),
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "default_nav_to_pose_bt_xml": behavior_tree,
                        "odom_topic": odometry_topic,
                        "use_sim_time": use_sim_time_parameter,
                    },
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "autostart": ParameterValue(autostart, value_type=bool),
                        "node_names": lifecycle_nodes,
                        "use_sim_time": use_sim_time_parameter,
                    }
                ],
            ),
        ]
    )
