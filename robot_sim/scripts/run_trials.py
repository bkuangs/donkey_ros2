#!/usr/bin/env python3

import argparse
import json
import math
import os
import random
import signal
import subprocess
import sys
from pathlib import Path


def trial_geometry(random_source):
    target_phase = random_source.uniform(-math.pi, math.pi)
    target_x = 3.0 * math.cos(target_phase)
    target_y = 3.0 * math.sin(target_phase)
    target_yaw = target_phase + 0.5 * math.pi
    robot_x = random_source.uniform(-0.75, 0.75)
    robot_y = random_source.uniform(-0.75, 0.75)
    robot_yaw = math.atan2(target_y - robot_y, target_x - robot_x)
    robot_yaw += random_source.uniform(-0.08, 0.08)
    return robot_x, robot_y, robot_yaw, target_x, target_y, target_yaw


def run_trial(command, timeout, log_path, environment):
    with log_path.open("w") as log:
        process = subprocess.Popen(
            command,
            env=environment,
            start_new_session=True,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            return_code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            return_code = 124
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return return_code


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--required-successes", type=int, default=8)
    parser.add_argument("--seed", type=int, default=20260810)
    parser.add_argument("--timeout", type=float, default=50.0)
    parser.add_argument("--output-dir", type=Path, default=Path("trial_results"))
    arguments = parser.parse_args()

    if not 0 < arguments.required_successes <= arguments.trials:
        parser.error("required successes must be between one and the trial count")

    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    random_source = random.Random(arguments.seed)
    results = []
    for index in range(arguments.trials):
        trial_seed = arguments.seed + index
        geometry = trial_geometry(random_source)
        result_path = (arguments.output_dir / f"trial_{index + 1:02d}.json").resolve()
        log_path = (arguments.output_dir / f"trial_{index + 1:02d}.log").resolve()
        result_path.unlink(missing_ok=True)
        log_path.unlink(missing_ok=True)
        command = [
            "ros2",
            "launch",
            "robot_sim",
            "trial.launch.py",
            f"seed:={trial_seed}",
            f"output_path:={result_path}",
            f"robot_x:={geometry[0]}",
            f"robot_y:={geometry[1]}",
            f"robot_yaw:={geometry[2]}",
            f"target_x:={geometry[3]}",
            f"target_y:={geometry[4]}",
            f"target_yaw:={geometry[5]}",
        ]
        environment = os.environ.copy()
        environment["GZ_PARTITION"] = f"target_intercept_{trial_seed}"
        environment["ROS_DOMAIN_ID"] = str(index + 1)
        return_code = run_trial(
            command,
            arguments.timeout,
            log_path,
            environment,
        )
        if result_path.exists():
            result = json.loads(result_path.read_text())
        else:
            result = {
                "seed": trial_seed,
                "success": False,
                "reason": "launch_failure",
                "return_code": return_code,
            }
        result["initial_conditions"] = {
            "robot_x": geometry[0],
            "robot_y": geometry[1],
            "robot_yaw": geometry[2],
            "target_x": geometry[3],
            "target_y": geometry[4],
            "target_yaw": geometry[5],
        }
        results.append(result)
        print(
            f"[{index + 1}/{arguments.trials}] seed={trial_seed} "
            f"success={result['success']} reason={result['reason']}"
        )

    successes = sum(result["success"] for result in results)
    summary = {
        "seed": arguments.seed,
        "trials": arguments.trials,
        "required_successes": arguments.required_successes,
        "successes": successes,
        "passed": successes >= arguments.required_successes,
        "results": results,
    }
    summary_path = arguments.output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(
        f"Capture gate: {successes}/{arguments.trials}; "
        f"required {arguments.required_successes}; passed={summary['passed']}"
    )
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
