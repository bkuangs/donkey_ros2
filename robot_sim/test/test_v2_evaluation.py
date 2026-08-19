import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
PACKAGE = Path(__file__).parents[1]

from run_v1_trials import V1_SCENARIOS, run_scenarios
from run_v2_trials import evaluate_v2_gate
from trial_evaluation import (
    LocalizationErrors,
    PlanarTransform,
    TrackingErrors,
    localization_is_ready,
    normalize_yaw,
)


def _odometry(x, y, yaw, velocity_x=0.0, velocity_y=0.0):
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                ),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=velocity_x, y=velocity_y)
            )
        ),
    )


def _passing_result(captured=True):
    return {
        "captured": captured,
        "obstacle_contact_count": 0,
        "collision_data_complete": True,
        "localization_data_complete": True,
        "ego_position_rmse": 0.10,
        "ego_yaw_rmse": 0.08,
        "ego_final_position_error": 0.15,
        "localization_availability": 0.98,
    }


def test_localization_readiness_requires_a_fresh_measurement():
    measured = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=4, nanosec=0))
    )
    assert localization_is_ready(4.05, measured, 0.1) is True
    assert localization_is_ready(4.11, measured, 0.1) is False
    assert localization_is_ready(4.0, None, 0.1) is False


def test_v2_trial_forwards_required_output_path():
    launch = (PACKAGE / "launch" / "v2_trial.launch.py").read_text()
    assert '"output_path": LaunchConfiguration("output_path")' in launch


def test_planar_transform_maps_pose_yaw_and_velocity():
    transform = PlanarTransform(3.0, -2.0, math.pi / 2.0)
    x, y, yaw = transform.pose(2.0, 1.0, 3.0 * math.pi / 4.0)
    assert math.isclose(x, 2.0, abs_tol=1e-12)
    assert math.isclose(y, 0.0, abs_tol=1e-12)
    assert math.isclose(yaw, -3.0 * math.pi / 4.0, abs_tol=1e-12)
    velocity_x, velocity_y = transform.velocity(2.0, -1.0)
    assert math.isclose(velocity_x, 1.0, abs_tol=1e-12)
    assert math.isclose(velocity_y, 2.0, abs_tol=1e-12)
    assert normalize_yaw(math.pi) == -math.pi


def test_tracking_errors_transform_relative_target_estimate():
    transform = PlanarTransform(4.0, 5.0, math.pi / 2.0)
    estimate = _odometry(2.0, 0.0, 0.0, velocity_x=1.5)
    target = _odometry(4.0, 7.0, math.pi / 2.0, velocity_x=1.5)
    errors = TrackingErrors()
    errors.add(estimate, target, estimate_transform=transform)
    position_rmse, velocity_rmse = errors.rmses()
    assert math.isclose(position_rmse, 0.0, abs_tol=1e-12)
    assert math.isclose(velocity_rmse, 0.0, abs_tol=1e-12)


def test_localization_metrics_wrap_yaw_and_count_discontinuities():
    errors = LocalizationErrors()
    errors.add((0.1, 0.0, -math.pi + 0.01), (0.0, 0.0, math.pi - 0.01))
    errors.add(
        (1.0, 0.0, -math.pi + 0.01),
        (1.0, 0.0, math.pi - 0.01),
        discontinuity_position=0.5,
    )
    metrics = errors.metrics()
    assert errors.samples == 2
    assert errors.discontinuities == 1
    assert math.isclose(metrics["position_rmse"], math.sqrt(0.005))
    assert math.isclose(metrics["yaw_rmse"], 0.02, abs_tol=1e-12)
    assert metrics["final_position_error"] == 0.0


def test_v2_gate_reuses_ten_scenarios_and_enforces_every_threshold():
    assert len(V1_SCENARIOS) == 10
    results = [_passing_result(index < 8) for index in range(10)]
    gate = evaluate_v2_gate(results)
    assert gate["captures"] == 8
    assert gate["passed"] is True

    for field, failing_value in (
        ("obstacle_contact_count", 1),
        ("collision_data_complete", False),
        ("localization_data_complete", False),
        ("ego_position_rmse", 0.21),
        ("ego_yaw_rmse", 0.16),
        ("ego_final_position_error", 0.31),
        ("localization_availability", 0.94),
    ):
        failing = [dict(result) for result in results]
        failing[-1][field] = failing_value
        assert evaluate_v2_gate(failing)["passed"] is False


def test_v2_gate_rejects_missing_localization_metrics():
    results = [_passing_result() for _ in range(10)]
    results[-1]["ego_position_rmse"] = None
    assert evaluate_v2_gate(results)["passed"] is False


def test_v2_gate_rejects_partial_trial_sets():
    gate = evaluate_v2_gate([_passing_result() for _ in range(8)])
    assert gate["captures"] == 8
    assert gate["complete_trial_set"] is False
    assert gate["passed"] is False


def test_runner_persists_process_timeout(tmp_path):
    arguments = SimpleNamespace(
        output_dir=tmp_path,
        trials=1,
        timeout=150.0,
        trial_timeout=45.0,
        mode_topic="/navigation/cmd_vel_owner",
    )
    with patch("run_v1_trials.run_trial", return_value=124):
        result = run_scenarios(arguments, version=2)[0]

    assert result["reason"] == "process_timeout"
    assert result["return_code"] == 124
    assert json.loads((tmp_path / "trial_01.json").read_text()) == result
