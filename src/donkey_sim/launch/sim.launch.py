import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    
    pkg_dir = get_package_share_directory('donkey_sim')
    bridge_yaml_path = os.path.join(pkg_dir, 'config', 'bridge.yaml')

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
            "-z", "0.0",
        ],
        output="screen",
    )

    # --- Spawn controllers ---
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

    ackermann_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # --- Gazebo bridges ---
    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': bridge_yaml_path}],
        output="screen",
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],  # Enhanced mapping syntax
        parameters=[{"qos": "sensor_data"}],
        output="screen",
    )

    cmd_vel_stamper = Node(
        package="donkey_sim",
        executable="cmd_vel_stamper.py",
        name="cmd_vel_stamper",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"frame_id": "base_link"},
            {"input_topic": "/cmd_vel"},
            {"output_topic": "/ackermann_steering_controller/reference"},
        ],
        output="screen",
    )

    # 1. Wait for robot to spawn -> load joint state broadcaster
    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # 2. Wait for joint state broadcaster -> load Ackermann controller
    load_ackermann_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_steering_controller_spawner],
        )
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("donkey_sim"),
        "rviz",
        "display.rviz",
    ])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use sim time from Gazebo",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="SDF world file to load in Gazebo Sim",
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        sensor_bridge,
        image_bridge,
        cmd_vel_stamper,
        
        # Load the sequential event handlers instead of direct nodes
        load_joint_state_broadcaster,
        load_ackermann_controller,
        rviz,
    ])