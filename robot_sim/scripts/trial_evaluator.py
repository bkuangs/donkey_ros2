#!/usr/bin/env python3

import math
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from trial_evaluation import (
    CaptureDwell,
    TrackingErrors,
    stamp_seconds,
    write_result,
)


class TrialEvaluator(Node):
    def __init__(self):
        super().__init__("trial_evaluator")
        self.declare_parameter("ego_topic", "/ground_truth/odom")
        self.declare_parameter("target_topic", "/target/ground_truth/odom")
        self.declare_parameter("estimate_topic", "/tracking/target_state")
        self.declare_parameter("capture_radius", 0.45)
        self.declare_parameter("capture_dwell", 0.2)
        self.declare_parameter("trial_timeout", 20.0)
        self.declare_parameter("sync_tolerance", 0.1)
        self.declare_parameter("seed", 0)
        self.declare_parameter("output_path", "trial_result.json")

        self.capture_radius = self.get_parameter("capture_radius").value
        self.capture_dwell = self.get_parameter("capture_dwell").value
        self.trial_timeout = self.get_parameter("trial_timeout").value
        self.sync_tolerance = self.get_parameter("sync_tolerance").value
        self.seed = self.get_parameter("seed").value
        self.output_path = Path(self.get_parameter("output_path").value)

        self.ego = None
        self.target = None
        self.estimate = None
        self.first_stamp = None
        self.capture = CaptureDwell(self.capture_radius, self.capture_dwell)
        self.minimum_distance = math.inf
        self.tracking_errors = TrackingErrors()
        self.finished = False
        self.success = False
        self.started_at = time.monotonic()

        self.ego_subscription = self.create_subscription(
            Odometry,
            self.get_parameter("ego_topic").value,
            self.ego_callback,
            10,
        )
        self.target_subscription = self.create_subscription(
            Odometry,
            self.get_parameter("target_topic").value,
            self.target_callback,
            10,
        )
        self.estimate_subscription = self.create_subscription(
            Odometry,
            self.get_parameter("estimate_topic").value,
            self.estimate_callback,
            10,
        )
        self.create_timer(0.02, self.evaluate)

    def ego_callback(self, message):
        self.ego = message

    def target_callback(self, message):
        self.target = message

    def estimate_callback(self, message):
        self.estimate = message

    def evaluate_estimate(self):
        if self.estimate is None:
            return
        if abs(stamp_seconds(self.target) - stamp_seconds(self.estimate)) > (
            self.sync_tolerance
        ):
            return

        self.tracking_errors.add(self.estimate, self.target)
        self.estimate = None

    def evaluate(self):
        if self.finished:
            return
        if time.monotonic() - self.started_at > 30.0 and self.first_stamp is None:
            self.finish(False, "truth_timeout", 0.0)
            return
        if self.ego is None or self.target is None:
            return

        ego_stamp = stamp_seconds(self.ego)
        target_stamp = stamp_seconds(self.target)
        if abs(ego_stamp - target_stamp) > self.sync_tolerance:
            return
        current_stamp = max(ego_stamp, target_stamp)
        if self.first_stamp is None:
            self.first_stamp = current_stamp

        elapsed = current_stamp - self.first_stamp
        distance = math.hypot(
            self.ego.pose.pose.position.x - self.target.pose.pose.position.x,
            self.ego.pose.pose.position.y - self.target.pose.pose.position.y,
        )
        self.minimum_distance = min(self.minimum_distance, distance)
        self.evaluate_estimate()

        if self.capture.update(distance, current_stamp):
            self.finish(True, "captured", elapsed)
            return

        if elapsed >= self.trial_timeout:
            self.finish(False, "trial_timeout", elapsed)

    def finish(self, success, reason, elapsed):
        position_rmse, velocity_rmse = self.tracking_errors.rmses()
        result = {
            "seed": self.seed,
            "success": success,
            "reason": reason,
            "elapsed_seconds": elapsed,
            "minimum_distance": (
                self.minimum_distance
                if math.isfinite(self.minimum_distance)
                else None
            ),
            "capture_radius": self.capture_radius,
            "capture_dwell": self.capture_dwell,
            "target_position_rmse": position_rmse,
            "target_velocity_rmse": velocity_rmse,
            "estimate_samples": self.tracking_errors.samples,
        }
        write_result(self.output_path, result)
        self.success = success
        self.finished = True
        self.get_logger().info(
            f"Trial {self.seed} finished: {reason}, "
            f"minimum distance={result['minimum_distance']}"
        )


def main():
    rclpy.init()
    evaluator = TrialEvaluator()
    while rclpy.ok() and not evaluator.finished:
        rclpy.spin_once(evaluator, timeout_sec=0.1)
    evaluator.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
