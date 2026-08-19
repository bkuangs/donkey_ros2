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
    assert parameters["publish_tf"] is True
    assert parameters["subscribe_rgbd"] is False
    assert parameters["approx_sync"] is True
    assert parameters["qos"] == parameters["qos_camera_info"] == 2
    assert parameters["Vis/MinInliers"] == "10"
    assert parameters["Reg/Force3DoF"] == "true"


def test_manifest_runtime_dependencies():
    root = ET.parse(PACKAGE / "package.xml").getroot()
    dependencies = {element.text for element in root.findall("exec_depend")}
    assert dependencies == {"launch", "launch_ros", "rtabmap_odom", "tf2_ros"}
