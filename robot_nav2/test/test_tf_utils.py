import math

from robot_nav2.tf_utils import GroundTruthGate, canonical_frame, planar_quaternion


def test_canonical_frame_only_removes_ros_leading_slash():
    assert canonical_frame("/odom") == "odom"
    assert canonical_frame("robot/odom") == "robot/odom"


def test_planar_quaternion_normalizes_and_removes_roll_pitch():
    half_yaw = 0.35
    quaternion = planar_quaternion((0.0, 0.0, 2.0 * math.sin(half_yaw), 2.0 * math.cos(half_yaw)))
    assert quaternion is not None
    assert quaternion[:2] == (0.0, 0.0)
    assert math.isclose(quaternion[2], math.sin(half_yaw))
    assert math.isclose(quaternion[3], math.cos(half_yaw))
    assert planar_quaternion((0.0, 0.0, 0.0, 0.0)) is None


def test_gate_rejects_bad_frames_stale_future_and_out_of_order_samples():
    gate = GroundTruthGate(max_age_ns=500, future_tolerance_ns=100)
    assert gate.accept(1000, 900, "odom", "base_footprint")[0]
    assert not gate.accept(1100, 900, "odom", "base_footprint")[0]
    assert not gate.accept(2000, 1400, "odom", "base_footprint")[0]
    assert not gate.accept(2000, 2200, "odom", "base_footprint")[0]
    assert not gate.accept(2000, 1900, "map", "base_footprint")[0]
    assert not gate.accept(2000, 1900, "odom", "base_link")[0]


def test_gate_recovers_after_sim_clock_reset():
    gate = GroundTruthGate(max_age_ns=500, future_tolerance_ns=100)
    assert gate.accept(10_000, 9_900, "odom", "base_footprint")[0]
    assert gate.accept(1_000, 900, "odom", "base_footprint")[0]
