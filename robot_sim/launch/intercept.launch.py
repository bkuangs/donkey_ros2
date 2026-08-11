from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def package_launch(package, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", filename])
        ),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map")
    default_world = PathJoinSubstitution([
        FindPackageShare("robot_sim"),
        "worlds",
        "empty.sdf",
    ])
    default_map = PathJoinSubstitution([
        FindPackageShare("robot_navigation"),
        "maps",
        "empty_sim.yaml",
    ])

    common = {"use_sim_time": use_sim_time}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("map", default_value=default_map),
        package_launch(
            "robot_sim",
            "sim.launch.py",
            {"use_sim_time": use_sim_time, "world": world},
        ),
        package_launch("robot_vio", "localization.launch.py", common),
        package_launch("robot_perception", "perception.launch.py", common),
        package_launch("robot_tracking", "target_tracking.launch.py", common),
        package_launch(
            "robot_navigation",
            "navigation.launch.py",
            {"use_sim_time": use_sim_time, "map": map_yaml},
        ),
    ])
