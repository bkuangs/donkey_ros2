#!/usr/bin/env python3

import math
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import UInt8

from trial_evaluation import (
    CaptureDwell,
    LocalizationErrors,
    PlanarTransform,
    TrackingErrors,
    localization_is_ready,
    stamp_seconds,
    write_result,
    yaw_from_quaternion,
)
from v1_obstacle_geometry import (
    ROBOT_FOOTPRINT_LENGTH,
    ROBOT_FOOTPRINT_WIDTH,
    V1_OBSTACLES,
    obstacle_clearances,
)


class V1TrialEvaluator(Node):
    def __init__(self):
        super().__init__("v1_trial_evaluator")
        self.declare_parameter("ego_topic", "/ground_truth/odom")
        self.declare_parameter("target_topic", "/target/ground_truth/odom")
        self.declare_parameter("estimate_topic", "/tracking/target_state")
        self.declare_parameter("mode_topic", "/navigation/cmd_vel_owner")
        self.declare_parameter("capture_radius", 0.45)
        self.declare_parameter("capture_dwell", 0.2)
        self.declare_parameter("trial_timeout", 45.0)
        self.declare_parameter("sync_tolerance", 0.1)
        self.declare_parameter("footprint_length", ROBOT_FOOTPRINT_LENGTH)
        self.declare_parameter("footprint_width", ROBOT_FOOTPRINT_WIDTH)
        self.declare_parameter("seed", 0)
        self.declare_parameter("scenario", "")
        self.declare_parameter("output_path", "v1_trial_result.json")
        self.declare_parameter("evaluate_localization", False)
        self.declare_parameter("measured_ego_topic", "/localization/odom")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("localization_discontinuity_position", 0.75)
        self.declare_parameter("localization_discontinuity_yaw", 0.75)

        self.capture_radius = self.get_parameter("capture_radius").value
        self.capture_dwell = self.get_parameter("capture_dwell").value
        self.trial_timeout = self.get_parameter("trial_timeout").value
        self.sync_tolerance = self.get_parameter("sync_tolerance").value
        self.footprint_length = self.get_parameter("footprint_length").value
        self.footprint_width = self.get_parameter("footprint_width").value
        self.seed = self.get_parameter("seed").value
        self.scenario = self.get_parameter("scenario").value
        self.output_path = Path(self.get_parameter("output_path").value)
        self.evaluate_localization = self.get_parameter(
            "evaluate_localization"
        ).value
        self.initial_transform = PlanarTransform(
            self.get_parameter("initial_x").value,
            self.get_parameter("initial_y").value,
            self.get_parameter("initial_yaw").value,
        )
        self.localization_discontinuity_position = self.get_parameter(
            "localization_discontinuity_position"
        ).value
        self.localization_discontinuity_yaw = self.get_parameter(
            "localization_discontinuity_yaw"
        ).value

        self.ego = None
        self.measured_ego = None
        self.target = None
        self.estimate = None
        self.first_stamp = None
        self.capture = CaptureDwell(self.capture_radius, self.capture_dwell)
        self.minimum_distance = math.inf
        self.minimum_clearance = {
            obstacle.name: math.inf for obstacle in V1_OBSTACLES
        }
        self.active_contacts = set()
        self.contact_events = []
        self.collision_samples = 0
        self.mode_transitions = []
        self.last_mode = None
        self.tracking_errors = TrackingErrors()
        self.localization_errors = LocalizationErrors()
        self.localization_opportunities = 0
        self.localization_available_opportunities = 0
        self.last_truth_evaluation_stamp = None
        self.last_measured_stamp = None
        self.last_measured_evaluation_stamp = None
        self.first_measured_stamp = None
        self.timestamp_regression = False
        self.finished = False
        self.success = False
        self.started_at = time.monotonic()

        self.create_subscription(
            Odometry,
            self.get_parameter("ego_topic").value,
            self.ego_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("target_topic").value,
            self.target_callback,
            10,
        )
        if self.evaluate_localization:
            self.create_subscription(
                Odometry,
                self.get_parameter("measured_ego_topic").value,
                self.measured_ego_callback,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Odometry,
            self.get_parameter("estimate_topic").value,
            self.estimate_callback,
            10,
        )
        mode_topic = self.get_parameter("mode_topic").value
        owner_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            UInt8,
            mode_topic,
            self.mode_callback,
            owner_qos,
        )
        self.create_timer(0.02, self.evaluate)

    def ego_callback(self, message):
        self.ego = message

    def target_callback(self, message):
        self.target = message

    def measured_ego_callback(self, message):
        stamp = stamp_seconds(message)
        if self.last_measured_stamp is not None and stamp < self.last_measured_stamp:
            self.timestamp_regression = True
        self.last_measured_stamp = stamp
        if self.first_measured_stamp is None:
            self.first_measured_stamp = stamp
        self.measured_ego = message

    def estimate_callback(self, message):
        self.estimate = message

    def mode_callback(self, message):
        mode_names = {0: "stop", 1: "nav2", 2: "terminal"}
        mode = mode_names.get(message.data, str(message.data))
        if mode == self.last_mode:
            return
        elapsed = None
        if self.first_stamp is not None and self.ego is not None:
            elapsed = stamp_seconds(self.ego) - self.first_stamp
        self.mode_transitions.append({"mode": mode, "elapsed_seconds": elapsed})
        self.last_mode = mode

    def evaluate_estimate(self):
        if self.estimate is None:
            return
        if abs(stamp_seconds(self.target) - stamp_seconds(self.estimate)) > (
            self.sync_tolerance
        ):
            return

        estimate_transform = (
            self.initial_transform if self.evaluate_localization else None
        )
        self.tracking_errors.add(
            self.estimate,
            self.target,
            estimate_transform=estimate_transform,
        )
        self.estimate = None

    def evaluate_localization_sample(self, ego_stamp):
        if not self.evaluate_localization:
            return
        if (
            self.last_truth_evaluation_stamp is not None
            and ego_stamp < self.last_truth_evaluation_stamp
        ):
            self.timestamp_regression = True
        if ego_stamp == self.last_truth_evaluation_stamp:
            return
        self.last_truth_evaluation_stamp = ego_stamp
        self.localization_opportunities += 1
        if self.measured_ego is None:
            return
        measured_stamp = stamp_seconds(self.measured_ego)
        if abs(ego_stamp - measured_stamp) > self.sync_tolerance:
            return
        self.localization_available_opportunities += 1
        if measured_stamp == self.last_measured_evaluation_stamp:
            return
        self.last_measured_evaluation_stamp = measured_stamp
        measured_position = self.measured_ego.pose.pose.position
        measured_yaw = yaw_from_quaternion(
            self.measured_ego.pose.pose.orientation
        )
        measured_pose = self.initial_transform.pose(
            measured_position.x,
            measured_position.y,
            measured_yaw,
        )
        truth_position = self.ego.pose.pose.position
        truth_pose = (
            truth_position.x,
            truth_position.y,
            yaw_from_quaternion(self.ego.pose.pose.orientation),
        )
        self.localization_errors.add(
            measured_pose,
            truth_pose,
            self.localization_discontinuity_position,
            self.localization_discontinuity_yaw,
        )

    def evaluate_obstacles(self, elapsed):
        self.collision_samples += 1
        position = self.ego.pose.pose.position
        yaw = yaw_from_quaternion(self.ego.pose.pose.orientation)
        clearances = obstacle_clearances(
            position.x,
            position.y,
            yaw,
            length=self.footprint_length,
            width=self.footprint_width,
        )
        current_contacts = {
            name for name, clearance in clearances.items() if clearance <= 0.0
        }
        for name, clearance in clearances.items():
            self.minimum_clearance[name] = min(
                self.minimum_clearance[name], clearance
            )
        for name in sorted(current_contacts - self.active_contacts):
            self.contact_events.append(
                {"obstacle": name, "elapsed_seconds": elapsed}
            )
        self.active_contacts = current_contacts
        return bool(current_contacts)

    def evaluate(self):
        if self.finished:
            return
        if time.monotonic() - self.started_at > 30.0 and self.first_stamp is None:
            reason = (
                "localization_startup_timeout"
                if self.evaluate_localization
                and self.ego is not None
                and self.target is not None
                else "truth_timeout"
            )
            self.finish(False, reason, 0.0)
            return
        if self.ego is None or self.target is None:
            return

        ego_stamp = stamp_seconds(self.ego)
        target_stamp = stamp_seconds(self.target)
        if abs(ego_stamp - target_stamp) > self.sync_tolerance:
            return
        current_stamp = max(ego_stamp, target_stamp)
        if self.first_stamp is None:
            if self.evaluate_localization and not localization_is_ready(
                ego_stamp,
                self.measured_ego,
                self.sync_tolerance,
            ):
                return
            self.first_stamp = current_stamp

        elapsed = current_stamp - self.first_stamp
        self.evaluate_localization_sample(ego_stamp)
        if self.evaluate_localization and self.timestamp_regression:
            self.finish(False, "localization_timestamp_regression", elapsed)
            return
        distance = math.hypot(
            self.ego.pose.pose.position.x - self.target.pose.pose.position.x,
            self.ego.pose.pose.position.y - self.target.pose.pose.position.y,
        )
        self.minimum_distance = min(self.minimum_distance, distance)
        self.evaluate_estimate()

        if self.evaluate_obstacles(elapsed):
            self.finish(False, "obstacle_collision", elapsed)
            return

        if self.capture.update(distance, current_stamp):
            self.finish(True, "captured", elapsed)
            return

        if elapsed >= self.trial_timeout:
            self.finish(False, "trial_timeout", elapsed)

    def finish(self, success, reason, elapsed):
        if (
            self.evaluate_localization
            and success
            and self.localization_errors.samples == 0
        ):
            success = False
            reason = "localization_unavailable"
        position_rmse, velocity_rmse = self.tracking_errors.rmses()
        finite_clearances = {
            name: clearance if math.isfinite(clearance) else None
            for name, clearance in self.minimum_clearance.items()
        }
        observed_clearances = [
            clearance
            for clearance in finite_clearances.values()
            if clearance is not None
        ]
        result = {
            "version": 2 if self.evaluate_localization else 1,
            "seed": self.seed,
            "scenario": self.scenario,
            "success": success,
            "captured": success and reason == "captured",
            "reason": reason,
            "termination_reason": reason,
            "elapsed_seconds": elapsed,
            "minimum_distance": (
                self.minimum_distance
                if math.isfinite(self.minimum_distance)
                else None
            ),
            "capture_radius": self.capture_radius,
            "capture_dwell": self.capture_dwell,
            "obstacle_collision": bool(self.contact_events),
            "obstacle_contact_count": len(self.contact_events),
            "obstacle_contacts": self.contact_events,
            "collision_samples": self.collision_samples,
            "collision_data_complete": (
                self.collision_samples > 0
                and all(
                    clearance is not None
                    for clearance in finite_clearances.values()
                )
            ),
            "minimum_obstacle_clearance": (
                min(observed_clearances) if observed_clearances else None
            ),
            "minimum_obstacle_clearance_by_name": finite_clearances,
            "mode_topic_available": bool(self.mode_transitions),
            "mode_transitions": self.mode_transitions,
            "target_position_rmse": position_rmse,
            "target_velocity_rmse": velocity_rmse,
            "estimate_samples": self.tracking_errors.samples,
        }
        if self.evaluate_localization:
            localization = self.localization_errors.metrics()
            availability = (
                self.localization_available_opportunities
                / self.localization_opportunities
                if self.localization_opportunities
                else 0.0
            )
            result.update({
                "ego_position_rmse": localization["position_rmse"],
                "ego_yaw_rmse": localization["yaw_rmse"],
                "ego_max_position_error": localization[
                    "maximum_position_error"
                ],
                "ego_max_yaw_error": localization["maximum_yaw_error"],
                "ego_final_position_error": localization[
                    "final_position_error"
                ],
                "ego_final_yaw_error": localization["final_yaw_error"],
                "localization_samples": self.localization_errors.samples,
                "localization_expected_samples": (
                    self.localization_opportunities
                ),
                "localization_availability": availability,
                "localization_discontinuity_count": (
                    self.localization_errors.discontinuities
                ),
                "localization_timestamp_regression": (
                    self.timestamp_regression
                ),
                "localization_data_complete": (
                    self.localization_errors.samples > 0
                    and self.localization_opportunities > 0
                    and not self.timestamp_regression
                ),
                "evaluation_reset": {
                    "policy": "fresh_process",
                    "first_truth_stamp": self.first_stamp,
                    "first_localization_stamp": self.first_measured_stamp,
                },
            })
        write_result(self.output_path, result)
        self.success = success
        self.finished = True
        self.get_logger().info(
            f"V{result['version']} trial {self.seed} finished: {reason}, "
            f"minimum distance={result['minimum_distance']}, "
            f"minimum clearance={result['minimum_obstacle_clearance']}"
        )


def main():
    rclpy.init()
    evaluator = V1TrialEvaluator()
    while rclpy.ok() and not evaluator.finished:
        rclpy.spin_once(evaluator, timeout_sec=0.1)
    evaluator.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
