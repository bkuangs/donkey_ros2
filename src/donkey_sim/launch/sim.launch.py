from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    default_world = PathJoinSubstitution([
        FindPackageShare("donkey_sim"),
        "worlds",
        "empty.sdf",
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare("donkey_sim"),
        "urdf",
        "donkey_sim.urdf.xacro",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]),
            value_type=str,
        ),
        "use_sim_time": use_sim_time,
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ),
        launch_arguments={
            "gz_args": ["-r ", world],
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "donkey_car",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.05",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    front_steer_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "front_steer_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    rear_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rear_wheel_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                front_steer_controller_spawner,
                rear_wheel_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time from Gazebo",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="SDF world file to load in Gazebo Sim",
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        spawn_controllers,
    ])
