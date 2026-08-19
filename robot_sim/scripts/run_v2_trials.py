#!/usr/bin/env python3

import argparse
import json
import sys
from pathlib import Path

from run_v1_trials import V1_SCENARIOS, run_scenarios


def evaluate_v2_gate(
    results,
    required_successes=8,
    maximum_contacts=0,
    maximum_position_rmse=0.20,
    maximum_yaw_rmse=0.15,
    maximum_final_position_error=0.30,
    minimum_localization_availability=0.95,
):
    captures = sum(bool(result.get("captured")) for result in results)
    complete_trial_set = len(results) == len(V1_SCENARIOS)
    contacts = sum(
        result.get("obstacle_contact_count", 0) for result in results
    )
    collision_data_complete = all(
        result.get("collision_data_complete") is True for result in results
    )
    localization_data_complete = all(
        result.get("localization_data_complete") is True for result in results
    )

    def worst(field, choose=max):
        values = [result.get(field) for result in results]
        if not values or any(value is None for value in values):
            return None
        return choose(values)

    position_rmse = worst("ego_position_rmse")
    yaw_rmse = worst("ego_yaw_rmse")
    final_position_error = worst("ego_final_position_error")
    localization_availability = worst(
        "localization_availability",
        choose=min,
    )
    passed = (
        complete_trial_set
        and captures >= required_successes
        and contacts <= maximum_contacts
        and collision_data_complete
        and localization_data_complete
        and position_rmse is not None
        and position_rmse <= maximum_position_rmse
        and yaw_rmse is not None
        and yaw_rmse <= maximum_yaw_rmse
        and final_position_error is not None
        and final_position_error <= maximum_final_position_error
        and localization_availability is not None
        and localization_availability >= minimum_localization_availability
    )
    return {
        "captures": captures,
        "complete_trial_set": complete_trial_set,
        "obstacle_contacts": contacts,
        "collision_data_complete": collision_data_complete,
        "localization_data_complete": localization_data_complete,
        "maximum_ego_position_rmse": position_rmse,
        "maximum_ego_yaw_rmse": yaw_rmse,
        "maximum_ego_final_position_error": final_position_error,
        "minimum_localization_availability": localization_availability,
        "passed": passed,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--required-successes", type=int, default=8)
    parser.add_argument("--maximum-contacts", type=int, default=0)
    parser.add_argument("--max-position-rmse", type=float, default=0.20)
    parser.add_argument("--max-yaw-rmse", type=float, default=0.15)
    parser.add_argument(
        "--max-final-position-error",
        type=float,
        default=0.30,
    )
    parser.add_argument(
        "--min-localization-availability",
        type=float,
        default=0.95,
    )
    parser.add_argument("--timeout", type=float, default=75.0)
    parser.add_argument("--trial-timeout", type=float, default=45.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("v2_trial_results"),
    )
    parser.add_argument("--mode-topic", default="/navigation/cmd_vel_owner")
    arguments = parser.parse_args()

    if not 0 < arguments.trials <= len(V1_SCENARIOS):
        parser.error(f"trials must be between one and {len(V1_SCENARIOS)}")
    if not 0 < arguments.required_successes <= arguments.trials:
        parser.error("required successes must be between one and the trial count")
    if arguments.maximum_contacts < 0:
        parser.error("maximum contacts cannot be negative")
    if min(
        arguments.max_position_rmse,
        arguments.max_yaw_rmse,
        arguments.max_final_position_error,
    ) < 0.0:
        parser.error("error thresholds cannot be negative")
    if not 0.0 <= arguments.min_localization_availability <= 1.0:
        parser.error("minimum localization availability must be in [0, 1]")
    if arguments.timeout <= arguments.trial_timeout:
        parser.error("process timeout must be longer than trial timeout")

    results = run_scenarios(
        arguments,
        launch_file="v2_trial.launch.py",
        version=2,
        partition_prefix="target_intercept_v2",
    )
    gate = evaluate_v2_gate(
        results,
        required_successes=arguments.required_successes,
        maximum_contacts=arguments.maximum_contacts,
        maximum_position_rmse=arguments.max_position_rmse,
        maximum_yaw_rmse=arguments.max_yaw_rmse,
        maximum_final_position_error=(
            arguments.max_final_position_error
        ),
        minimum_localization_availability=(
            arguments.min_localization_availability
        ),
    )
    summary = {
        "version": 2,
        "trials": arguments.trials,
        "thresholds": {
            "required_successes": arguments.required_successes,
            "maximum_contacts": arguments.maximum_contacts,
            "maximum_position_rmse": arguments.max_position_rmse,
            "maximum_yaw_rmse": arguments.max_yaw_rmse,
            "maximum_final_position_error": (
                arguments.max_final_position_error
            ),
            "minimum_localization_availability": (
                arguments.min_localization_availability
            ),
        },
        "trial_timeout_seconds": arguments.trial_timeout,
        "process_timeout_seconds": arguments.timeout,
        **gate,
        "results": results,
    }
    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = arguments.output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(
        f"V2 gate: captures={gate['captures']}/{arguments.trials} "
        f"(required {arguments.required_successes}), "
        f"contacts={gate['obstacle_contacts']}, "
        f"localization_complete={gate['localization_data_complete']}, "
        f"passed={gate['passed']}"
    )
    return 0 if gate["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
