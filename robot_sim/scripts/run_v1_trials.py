#!/usr/bin/env python3

import argparse
import json
import math
import os
import sys
from pathlib import Path

from trial_evaluation import write_result
from trial_process import run_process as run_trial


TARGET_RADIUS = 3.0
V1_SCENARIOS = (
    ("v1_chicane_01", 2026081901, -2.40, 0.00, -0.42),
    ("v1_chicane_02", 2026081902, -2.35, 0.25, -0.32),
    ("v1_chicane_03", 2026081903, -2.45, -0.25, -0.22),
    ("v1_chicane_04", 2026081904, -2.30, 0.50, -0.12),
    ("v1_chicane_05", 2026081905, -2.50, -0.50, -0.04),
    ("v1_chicane_06", 2026081906, -2.50, 0.35, 0.04),
    ("v1_chicane_07", 2026081907, -2.30, -0.35, 0.12),
    ("v1_chicane_08", 2026081908, -2.45, 0.15, 0.22),
    ("v1_chicane_09", 2026081909, -2.35, -0.15, 0.32),
    ("v1_chicane_10", 2026081910, -2.40, 0.00, 0.42),
)


def scenario_geometry(scenario):
    name, seed, robot_x, robot_y, target_phase = scenario
    target_x = TARGET_RADIUS * math.cos(target_phase)
    target_y = TARGET_RADIUS * math.sin(target_phase)
    target_yaw = target_phase + 0.5 * math.pi
    robot_yaw = math.atan2(target_y - robot_y, target_x - robot_x)
    return {
        "name": name,
        "seed": seed,
        "robot_x": robot_x,
        "robot_y": robot_y,
        "robot_yaw": robot_yaw,
        "target_x": target_x,
        "target_y": target_y,
        "target_yaw": target_yaw,
    }


def evaluate_gate(results, required_successes):
    successes = sum(bool(result.get("captured")) for result in results)
    complete_trial_set = len(results) == len(V1_SCENARIOS)
    contact_data_complete = all(
        result.get("collision_data_complete") is True for result in results
    )
    obstacle_contacts = sum(
        result.get("obstacle_contact_count", 0) for result in results
    )
    passed = (
        complete_trial_set
        and successes >= required_successes
        and obstacle_contacts == 0
        and contact_data_complete
    )
    return (
        successes,
        obstacle_contacts,
        complete_trial_set,
        contact_data_complete,
        passed,
    )


def run_scenarios(
    arguments,
    launch_file="v1_trial.launch.py",
    version=1,
    partition_prefix="target_intercept_v1",
):
    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for index, scenario in enumerate(V1_SCENARIOS[:arguments.trials]):
        geometry = scenario_geometry(scenario)
        result_path = (
            arguments.output_dir / f"trial_{index + 1:02d}.json"
        ).resolve()
        log_path = (
            arguments.output_dir / f"trial_{index + 1:02d}.log"
        ).resolve()
        result_path.unlink(missing_ok=True)
        log_path.unlink(missing_ok=True)
        command = [
            "ros2",
            "launch",
            "robot_sim",
            launch_file,
            f"seed:={geometry['seed']}",
            f"scenario:={geometry['name']}",
            f"output_path:={result_path}",
            f"robot_x:={geometry['robot_x']}",
            f"robot_y:={geometry['robot_y']}",
            f"robot_yaw:={geometry['robot_yaw']}",
            f"target_x:={geometry['target_x']}",
            f"target_y:={geometry['target_y']}",
            f"target_yaw:={geometry['target_yaw']}",
            f"trial_timeout:={arguments.trial_timeout}",
            f"mode_topic:={arguments.mode_topic}",
        ]
        environment = os.environ.copy()
        environment["GZ_PARTITION"] = f"{partition_prefix}_{geometry['seed']}"
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
            reason = "process_timeout" if return_code == 124 else "launch_failure"
            result = {
                "version": version,
                "seed": geometry["seed"],
                "scenario": geometry["name"],
                "success": False,
                "captured": False,
                "reason": reason,
                "termination_reason": reason,
                "return_code": return_code,
                "collision_data_complete": False,
            }
            if version == 2:
                result["localization_data_complete"] = False
                result["localization_availability"] = 0.0
        result["return_code"] = return_code
        result["initial_conditions"] = {
            key: value
            for key, value in geometry.items()
            if key not in ("name", "seed")
        }
        write_result(result_path, result)
        results.append(result)
        print(
            f"[{index + 1}/{arguments.trials}] "
            f"scenario={geometry['name']} success={result['success']} "
            f"reason={result['reason']} "
            f"contacts={result.get('obstacle_contact_count', 'unknown')}"
        )
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--required-successes", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=75.0)
    parser.add_argument("--trial-timeout", type=float, default=45.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("v1_trial_results"),
    )
    parser.add_argument("--mode-topic", default="/navigation/cmd_vel_owner")
    arguments = parser.parse_args()

    if not 0 < arguments.trials <= len(V1_SCENARIOS):
        parser.error(f"trials must be between one and {len(V1_SCENARIOS)}")
    if not 0 < arguments.required_successes <= arguments.trials:
        parser.error("required successes must be between one and the trial count")
    if arguments.timeout <= arguments.trial_timeout:
        parser.error("process timeout must be longer than trial timeout")

    results = run_scenarios(arguments)

    (
        successes,
        obstacle_contacts,
        complete_trial_set,
        contact_data_complete,
        passed,
    ) = evaluate_gate(
        results,
        arguments.required_successes,
    )
    summary = {
        "version": 1,
        "trials": arguments.trials,
        "required_successes": arguments.required_successes,
        "trial_timeout_seconds": arguments.trial_timeout,
        "process_timeout_seconds": arguments.timeout,
        "successes": successes,
        "obstacle_contacts": obstacle_contacts,
        "complete_trial_set": complete_trial_set,
        "contact_data_complete": contact_data_complete,
        "passed": passed,
        "results": results,
    }
    summary_path = arguments.output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(
        f"V1 gate: captures={successes}/{arguments.trials} "
        f"(required {arguments.required_successes}), "
        f"contacts={obstacle_contacts}, "
        f"contact_data_complete={contact_data_complete}, passed={passed}"
    )
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
