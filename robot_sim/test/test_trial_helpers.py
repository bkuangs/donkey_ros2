import json
import math
from pathlib import Path
import signal
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import Mock, patch


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from trial_evaluation import CaptureDwell, TrackingErrors, write_result
from trial_process import run_process


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


def test_capture_dwell_requires_continuous_capture():
    capture = CaptureDwell(radius=0.45, duration=0.2)
    assert not capture.update(0.4, 1.0)
    assert not capture.update(0.5, 1.3)
    assert not capture.update(0.4, 2.0)
    assert not capture.update(0.4, 2.19)
    assert capture.update(0.4, 2.2)


def test_tracking_rmse_rotates_truth_velocity_into_world_frame():
    assert TrackingErrors().rmses() == (None, None)
    errors = TrackingErrors()
    target = _odometry(1.0, 2.0, math.pi / 2.0, velocity_x=2.0)
    estimate = _odometry(4.0, 6.0, 0.0, velocity_x=0.0, velocity_y=2.0)
    errors.add(estimate, target)
    position_rmse, velocity_rmse = errors.rmses()
    assert position_rmse == 5.0
    assert math.isclose(velocity_rmse, 0.0, abs_tol=1e-12)
    assert errors.samples == 1


def test_result_write_is_atomic_and_preserves_json(tmp_path):
    output = tmp_path / "nested" / "result.json"
    result = {"success": True, "value": None}
    write_result(output, result)
    assert json.loads(output.read_text()) == result
    assert not output.with_suffix(".json.tmp").exists()


def test_process_timeout_escalates_and_cleans_up(tmp_path):
    process = Mock(pid=42)
    timeout = subprocess.TimeoutExpired("trial", 1)
    process.wait.side_effect = timeout, timeout, 0
    with (
        patch("trial_process.subprocess.Popen", return_value=process) as launch,
        patch("trial_process.os.killpg") as kill_group,
    ):
        return_code = run_process(["trial"], 1.0, tmp_path / "trial.log", {})
    assert return_code == 124
    assert launch.call_args.kwargs["start_new_session"] is True
    assert [call.args[1] for call in kill_group.call_args_list] == [
        signal.SIGTERM,
        signal.SIGKILL,
        signal.SIGKILL,
    ]
