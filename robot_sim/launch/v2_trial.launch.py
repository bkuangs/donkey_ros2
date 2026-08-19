from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    trial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_sim"),
                "launch",
                "v1_trial.launch.py",
            ])
        ),
        launch_arguments={
            "output_path": LaunchConfiguration("output_path"),
            "intercept_launch": "v2_intercept.launch.py",
            "evaluate_localization": "true",
        }.items(),
    )
    return LaunchDescription([trial])
