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
    gz_args = LaunchConfiguration("gz_args")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_yaw = LaunchConfiguration("robot_yaw")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_yaw = LaunchConfiguration("target_yaw")
    use_rviz = LaunchConfiguration("rviz")
    default_world = PathJoinSubstitution([
        FindPackageShare("robot_sim"),
        "worlds",
        "intercept_arena.sdf",
    ])

    common = {"use_sim_time": use_sim_time}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("gz_args", default_value="-r"),
        DeclareLaunchArgument("robot_x", default_value="0.0"),
        DeclareLaunchArgument("robot_y", default_value="0.0"),
        DeclareLaunchArgument("robot_yaw", default_value="0.0"),
        DeclareLaunchArgument("target_x", default_value="3.0"),
        DeclareLaunchArgument("target_y", default_value="0.0"),
        DeclareLaunchArgument("target_yaw", default_value="1.57079632679"),
        DeclareLaunchArgument("rviz", default_value="true"),
        package_launch(
            "robot_sim",
            "sim.launch.py",
            {
                "use_sim_time": use_sim_time,
                "world": world,
                "gz_args": gz_args,
                "robot_x": robot_x,
                "robot_y": robot_y,
                "robot_yaw": robot_yaw,
                "target_x": target_x,
                "target_y": target_y,
                "target_yaw": target_yaw,
                "rviz": use_rviz,
            },
        ),
        package_launch("robot_perception", "perception.launch.py", common),
        package_launch("robot_tracking", "target_tracking.launch.py", common),
        package_launch("robot_navigation", "navigation.launch.py", common),
    ])
