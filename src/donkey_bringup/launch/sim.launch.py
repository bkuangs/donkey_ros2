import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("rviz")
    start_teleop = LaunchConfiguration("teleop")

    default_world = PathJoinSubstitution([
        FindPackageShare("donkey_sim"),
        "worlds",
        "empty.sdf",
    ])

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("donkey_sim"),
                "launch",
                "sim.launch.py",
            ])
        ),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("donkey_control"),
                "launch",
                "control.launch.py",
            ])
        )
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("donkey_description"),
        "rviz",
        "display.rviz",
    ])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(start_rviz),
        output="screen",
    )

    teleop = ExecuteProcess(
        cmd=["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard", "--ros-args", "-r", "/cmd_vel:=/cmd_vel"],
        condition=IfCondition(start_teleop),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="SDF world file to load in Gazebo Sim",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo /clock for ROS nodes",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz with the robot display config",
        ),
        DeclareLaunchArgument(
            "teleop",
            default_value="false",
            description="Start keyboard teleop from the launch process",
        ),
        sim_launch,
        control_launch,
        rviz,
        teleop,
    ])
