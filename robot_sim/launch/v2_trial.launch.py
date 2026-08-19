from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    output_path = LaunchConfiguration("output_path")
    seed = LaunchConfiguration("seed")
    scenario = LaunchConfiguration("scenario")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    robot_yaw = LaunchConfiguration("robot_yaw")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_yaw = LaunchConfiguration("target_yaw")
    mode_topic = LaunchConfiguration("mode_topic")
    trial_timeout = LaunchConfiguration("trial_timeout")

    intercept = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_sim"),
                "launch",
                "v2_intercept.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gz_args": "-s -r",
            "robot_x": robot_x,
            "robot_y": robot_y,
            "robot_yaw": robot_yaw,
            "target_x": target_x,
            "target_y": target_y,
            "target_yaw": target_yaw,
            "rviz": "false",
        }.items(),
    )
    evaluator = Node(
        package="robot_sim",
        executable="v1_trial_evaluator.py",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "output_path": ParameterValue(output_path, value_type=str),
            "seed": ParameterValue(seed, value_type=int),
            "scenario": ParameterValue(scenario, value_type=str),
            "mode_topic": ParameterValue(mode_topic, value_type=str),
            "capture_radius": 0.45,
            "capture_dwell": 0.2,
            "trial_timeout": ParameterValue(trial_timeout, value_type=float),
            "evaluate_localization": True,
            "measured_ego_topic": "/localization/odom",
            "initial_x": ParameterValue(robot_x, value_type=float),
            "initial_y": ParameterValue(robot_y, value_type=float),
            "initial_yaw": ParameterValue(robot_yaw, value_type=float),
        }],
        output="screen",
    )
    stop_when_finished = RegisterEventHandler(
        OnProcessExit(
            target_action=evaluator,
            on_exit=[EmitEvent(event=Shutdown(reason="v2 trial finished"))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("output_path"),
        DeclareLaunchArgument("seed", default_value="2026081901"),
        DeclareLaunchArgument("scenario", default_value="v1_chicane_01"),
        DeclareLaunchArgument("robot_x", default_value="-2.4"),
        DeclareLaunchArgument("robot_y", default_value="0.0"),
        DeclareLaunchArgument("robot_yaw", default_value="0.0"),
        DeclareLaunchArgument("target_x", default_value="3.0"),
        DeclareLaunchArgument("target_y", default_value="0.0"),
        DeclareLaunchArgument("target_yaw", default_value="1.57079632679"),
        DeclareLaunchArgument("trial_timeout", default_value="45.0"),
        DeclareLaunchArgument(
            "mode_topic",
            default_value="/navigation/cmd_vel_owner",
        ),
        intercept,
        evaluator,
        stop_when_finished,
    ])
