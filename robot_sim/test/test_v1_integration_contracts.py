from pathlib import Path
import xml.etree.ElementTree as ET


REPOSITORY_ROOT = Path(__file__).parents[2]


def read(relative_path):
    return (REPOSITORY_ROOT / relative_path).read_text()


def dependencies(package):
    root = ET.parse(REPOSITORY_ROOT / package / "package.xml").getroot()
    return {
        element.text
        for element in root
        if element.tag in {"depend", "exec_depend"} and element.text
    }


def test_v0_launch_keeps_direct_controller_on_final_command_topic():
    intercept_launch = read("robot_sim/launch/intercept.launch.py")
    navigation_launch = read("robot_navigation/launch/navigation.launch.py")

    assert intercept_launch.count(
        'package_launch("robot_navigation", "navigation.launch.py", common)'
    ) == 1
    assert navigation_launch.count("executable='intercept_controller'") == 1
    assert "command_arbiter" not in navigation_launch
    assert "interception_supervisor" not in navigation_launch
    assert "'command_topic': '/cmd_vel'" in navigation_launch


def test_v1_launch_includes_each_required_stack_once():
    launch = read("robot_sim/launch/v1_intercept.launch.py")
    expected_includes = (
        'package_launch(\n            "robot_sim",',
        'package_launch("robot_perception", "perception.launch.py", common)',
        'package_launch("robot_tracking", "target_tracking.launch.py", common)',
        'package_launch("robot_nav2", "navigation.launch.py", common)',
        'package_launch("robot_navigation", "v1_navigation.launch.py", common)',
    )
    for include in expected_includes:
        assert launch.count(include) == 1
    assert "navigation_package" not in launch
    assert "navigation_launch" not in launch

    trial_launch = read("robot_sim/launch/v1_trial.launch.py")
    trial_runner = read("robot_sim/scripts/run_v1_trials.py")
    assert "navigation_package" not in trial_launch + trial_runner
    assert "navigation_launch" not in trial_launch + trial_runner


def test_application_nodes_do_not_consume_target_truth():
    application_packages = (
        "robot_nav2",
        "robot_navigation",
        "robot_perception",
        "robot_tracking",
    )
    for package in application_packages:
        for path in (REPOSITORY_ROOT / package).rglob("*"):
            if path.is_file() and path.suffix in {".cpp", ".hpp", ".py", ".yaml"}:
                assert "/target/ground_truth/odom" not in path.read_text()


def test_v1_command_ownership_and_mode_type_are_explicit():
    navigation_launch = read("robot_navigation/launch/v1_navigation.launch.py")
    navigation_config = read("robot_navigation/config/navigation.yaml")
    nav2_launch = read("robot_nav2/launch/navigation.launch.py")
    evaluator = read("robot_sim/scripts/v1_trial_evaluator.py")

    for executable in (
        "intercept_controller",
        "command_arbiter",
        "interception_supervisor",
    ):
        assert navigation_launch.count(f"executable='{executable}'") == 1
    assert "command_topic: /cmd_vel/terminal" in navigation_config
    assert "nav2_command_topic: /cmd_vel/nav2" in navigation_config
    assert "terminal_command_topic: /cmd_vel/terminal" in navigation_config
    assert "output_command_topic: /cmd_vel" in navigation_config
    assert '("cmd_vel", "/cmd_vel/nav2")' in nav2_launch
    assert "from std_msgs.msg import UInt8" in evaluator
    assert "String" not in evaluator
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in evaluator
    assert '{0: "stop", 1: "nav2", 2: "terminal"}' in evaluator


def test_frames_and_typed_nav2_launch_parameters_are_consistent():
    navigation_config = read("robot_navigation/config/navigation.yaml")
    supervisor = read("robot_navigation/src/interception_supervisor.cpp")
    nav2_config = read("robot_nav2/config/nav2.yaml")
    nav2_launch = read("robot_nav2/launch/navigation.launch.py")
    ground_truth_tf = read("robot_nav2/robot_nav2/ground_truth_tf.py")

    assert navigation_config.count("planning_frame: odom") == 2
    assert "terminal_enter_distance: 1.0" in navigation_config
    assert "terminal_exit_distance: 1.25" in navigation_config
    assert '"terminal_enter_distance", 1.0' in supervisor
    assert '"terminal_exit_distance", 1.25' in supervisor
    assert "global_frame: map" in nav2_config
    assert "global_frame: odom" in nav2_config
    assert 'transform.header.frame_id = "map"' in ground_truth_tf
    assert 'transform.child_frame_id = "odom"' in ground_truth_tf
    assert 'transform.header.frame_id = "odom"' in ground_truth_tf
    assert 'transform.child_frame_id = "base_footprint"' in ground_truth_tf
    assert "ParameterValue(use_sim_time, value_type=bool)" in nav2_launch
    assert "ParameterValue(map_file, value_type=str)" in nav2_launch
    assert "ParameterValue(autostart, value_type=bool)" in nav2_launch


def test_nav2_stack_is_static_map_ackermann_and_forward_only():
    nav2_config = read("robot_nav2/config/nav2.yaml")
    nav2_launch = read("robot_nav2/launch/navigation.launch.py")
    behavior_tree = read(
        "robot_nav2/behavior_trees/navigate_to_pose_ackermann.xml"
    )

    assert 'plugin: "nav2_smac_planner::SmacPlannerHybrid"' in nav2_config
    assert 'motion_model_for_search: "DUBIN"' in nav2_config
    assert (
        'plugin: "nav2_regulated_pure_pursuit_controller::'
        'RegulatedPurePursuitController"'
    ) in nav2_config
    assert "use_rotate_to_heading: false" in nav2_config
    assert "allow_reversing: false" in nav2_config
    assert nav2_config.count('plugin: "nav2_costmap_2d::StaticLayer"') == 2
    assert "ObstacleLayer" not in nav2_config
    assert "behavior_server" not in nav2_launch
    assert "BackUp" not in behavior_tree
    assert "Spin" not in behavior_tree


def test_package_dependencies_and_nav2_assets_are_installed():
    robot_sim_dependencies = dependencies("robot_sim")
    robot_nav2_dependencies = dependencies("robot_nav2")
    nav2_setup = read("robot_nav2/setup.py")
    sim_cmake = read("robot_sim/CMakeLists.txt")

    assert {"robot_navigation", "robot_nav2"} <= robot_sim_dependencies
    assert "robot_navigation" not in robot_nav2_dependencies
    assert "robot_sim" not in robot_nav2_dependencies
    for asset in ("behavior_trees", "config", "launch", "maps"):
        assert f'"{asset}"' in nav2_setup
    for script in (
        "run_v1_trials.py",
        "v1_obstacle_geometry.py",
        "v1_trial_evaluator.py",
    ):
        assert script in sim_cmake
