#!/usr/bin/env python3

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8

from v1_obstacle_geometry import (
    ROBOT_FOOTPRINT_LENGTH,
    ROBOT_FOOTPRINT_WIDTH,
    V1_OBSTACLES,
    obstacle_clearances,
)


def stamp_seconds(message):
    return message.header.stamp.sec + 1.0e-9 * message.header.stamp.nanosec


def yaw_from_quaternion(orientation):
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


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

        self.capture_radius = self.get_parameter("capture_radius").value
        self.capture_dwell = self.get_parameter("capture_dwell").value
        self.trial_timeout = self.get_parameter("trial_timeout").value
        self.sync_tolerance = self.get_parameter("sync_tolerance").value
        self.footprint_length = self.get_parameter("footprint_length").value
        self.footprint_width = self.get_parameter("footprint_width").value
        self.seed = self.get_parameter("seed").value
        self.scenario = self.get_parameter("scenario").value
        self.output_path = Path(self.get_parameter("output_path").value)

        self.ego = None
        self.target = None
        self.estimate = None
        self.first_stamp = None
        self.capture_start = None
        self.minimum_distance = math.inf
        self.minimum_clearance = {
            obstacle.name: math.inf for obstacle in V1_OBSTACLES
        }
        self.active_contacts = set()
        self.contact_events = []
        self.collision_samples = 0
        self.mode_transitions = []
        self.last_mode = None
        self.position_squared_error = 0.0
        self.velocity_squared_error = 0.0
        self.estimate_samples = 0
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

        position_error = math.hypot(
            self.estimate.pose.pose.position.x
            - self.target.pose.pose.position.x,
            self.estimate.pose.pose.position.y
            - self.target.pose.pose.position.y,
        )
        target_yaw = yaw_from_quaternion(
            self.target.pose.pose.orientation
        )
        body_velocity = self.target.twist.twist.linear
        target_velocity_x = (
            math.cos(target_yaw) * body_velocity.x
            - math.sin(target_yaw) * body_velocity.y
        )
        target_velocity_y = (
            math.sin(target_yaw) * body_velocity.x
            + math.cos(target_yaw) * body_velocity.y
        )
        velocity_error = math.hypot(
            self.estimate.twist.twist.linear.x - target_velocity_x,
            self.estimate.twist.twist.linear.y - target_velocity_y,
        )
        self.position_squared_error += position_error * position_error
        self.velocity_squared_error += velocity_error * velocity_error
        self.estimate_samples += 1
        self.estimate = None

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

        if self.evaluate_obstacles(elapsed):
            self.finish(False, "obstacle_collision", elapsed)
            return

        if distance <= self.capture_radius:
            if self.capture_start is None:
                self.capture_start = current_stamp
            elif current_stamp - self.capture_start >= self.capture_dwell:
                self.finish(True, "captured", elapsed)
                return
        else:
            self.capture_start = None

        if elapsed >= self.trial_timeout:
            self.finish(False, "trial_timeout", elapsed)

    def finish(self, success, reason, elapsed):
        position_rmse = None
        velocity_rmse = None
        if self.estimate_samples:
            position_rmse = math.sqrt(
                self.position_squared_error / self.estimate_samples
            )
            velocity_rmse = math.sqrt(
                self.velocity_squared_error / self.estimate_samples
            )
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
            "version": 1,
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
            "estimate_samples": self.estimate_samples,
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.output_path.with_suffix(
            self.output_path.suffix + ".tmp"
        )
        temporary_path.write_text(json.dumps(result, indent=2) + "\n")
        temporary_path.replace(self.output_path)
        self.success = success
        self.finished = True
        self.get_logger().info(
            f"V1 trial {self.seed} finished: {reason}, "
            f"minimum distance={result['minimum_distance']}, "
            f"minimum clearance={result['minimum_obstacle_clearance']}"
        )


def main():
    rclpy.init()
    evaluator = V1TrialEvaluator()
    while rclpy.ok() and not evaluator.finished:
        rclpy.spin_once(evaluator, timeout_sec=0.1)
    evaluator.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
