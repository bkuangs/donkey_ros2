import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]


def test_odometry_configuration():
    parameters = yaml.safe_load(
        (PACKAGE / "config" / "rgbd_odometry.yaml").read_text()
    )["rgbd_odometry"]["ros__parameters"]
    assert parameters["frame_id"] == "base_footprint"
    assert parameters["odom_frame_id"] == "odom"
    assert parameters["publish_tf"] is False
    assert parameters["subscribe_rgbd"] is False
    assert parameters["approx_sync"] is True
    assert parameters["qos"] == parameters["qos_camera_info"] == 2
    assert parameters["Vis/MinInliers"] == "10"
    assert parameters["Reg/Force3DoF"] == "true"


def test_localization_fuses_wheel_velocity_and_visual_pose():
    parameters = yaml.safe_load(
        (PACKAGE / "config" / "localization_ekf.yaml").read_text()
    )["localization_ekf"]["ros__parameters"]
    assert parameters["predict_to_current_time"] is True
    assert parameters["two_d_mode"] is True
    assert parameters["publish_tf"] is True
    assert parameters["world_frame"] == parameters["odom_frame"] == "odom"
    assert parameters["base_link_frame"] == "base_footprint"
    assert parameters["odom0"] == "/odom"
    assert parameters["odom0_config"] == [
        False, False, False,
        False, False, False,
        True, True, False,
        False, False, True,
        False, False, False,
    ]
    assert parameters["odom1"] == "/localization/visual_odom"
    assert parameters["odom1_config"] == [
        True, True, False,
        False, False, True,
        False, False, False,
        False, False, False,
        False, False, False,
    ]


def test_manifest_runtime_dependencies():
    root = ET.parse(PACKAGE / "package.xml").getroot()
    dependencies = {element.text for element in root.findall("exec_depend")}
    assert dependencies == {
        "launch",
        "launch_ros",
        "robot_localization",
        "rtabmap_odom",
        "tf2_ros",
    }
