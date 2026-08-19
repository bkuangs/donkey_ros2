import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
    
    pkg_dir = get_package_share_directory('robot_sim')
    bridge_yaml_path = os.path.join(pkg_dir, 'config', 'bridge.yaml')

    default_world = PathJoinSubstitution([
        FindPackageShare("robot_sim"),
        "worlds",
        "intercept_arena.sdf",
    ])

    model_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(pkg_dir, "models"),
    )

    xacro_file = PathJoinSubstitution([
        FindPackageShare("robot_sim"),
        "urdf",
        "robot_sim.urdf.xacro",
    ])
    target_model = PathJoinSubstitution([
        FindPackageShare("robot_sim"),
        "models",
        "red_ball",
        "model.sdf",
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
            "gz_args": [gz_args, " ", world],
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
        name="spawn_robot",
        arguments=[
            "-name", "robot_car",
            "-topic", "robot_description",
            "-x", robot_x,
            "-y", robot_y,
            "-z", "0.0",
            "-Y", robot_yaw,
        ],
        output="screen",
    )

    spawn_target = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_target",
        arguments=[
            "-name", "red_ball",
            "-file", target_model,
            "-x", target_x,
            "-y", target_y,
            "-z", "0.2",
            "-Y", target_yaw,
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
        name="rgb_image_bridge",
        arguments=["/camera/image_raw"],
        parameters=[{"qos": "sensor_data"}],
        output="screen",
    )

    depth_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="depth_image_bridge",
        arguments=["/camera/depth_image"],
        parameters=[{"qos": "sensor_data"}],
        output="screen",
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_info_bridge",
        arguments=[
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        output="screen",
    )

    cmd_vel_stamper = Node(
        package="robot_sim",
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
        FindPackageShare("robot_sim"),
        "rviz",
        "display.rviz",
    ])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
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
        DeclareLaunchArgument("gz_args", default_value="-r"),
        DeclareLaunchArgument("robot_x", default_value="0.0"),
        DeclareLaunchArgument("robot_y", default_value="0.0"),
        DeclareLaunchArgument("robot_yaw", default_value="0.0"),
        DeclareLaunchArgument("target_x", default_value="3.0"),
        DeclareLaunchArgument("target_y", default_value="0.0"),
        DeclareLaunchArgument("target_yaw", default_value="1.57079632679"),
        DeclareLaunchArgument("rviz", default_value="true"),
        model_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        spawn_target,
        sensor_bridge,
        image_bridge,
        depth_bridge,
        camera_info_bridge,
        cmd_vel_stamper,
        load_joint_state_broadcaster,
        load_ackermann_controller,
        rviz,
    ])