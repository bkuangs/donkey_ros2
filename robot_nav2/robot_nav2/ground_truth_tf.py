#!/usr/bin/env python3

import math
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from robot_nav2.tf_utils import GroundTruthGate, planar_quaternion


class GroundTruthTf(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_tf")
        self.declare_parameter("max_odom_age", 0.5)
        self.declare_parameter("future_tolerance", 0.1)

        max_age = self.get_parameter("max_odom_age").get_parameter_value().double_value
        future_tolerance = (
            self.get_parameter("future_tolerance").get_parameter_value().double_value
        )
        self._gate = GroundTruthGate(
            max_age_ns=int(max_age * 1.0e9),
            future_tolerance_ns=int(future_tolerance * 1.0e9),
        )
        self._last_warning_reason = ""
        self._last_warning_time = 0.0
        self._broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_map_to_odom()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self._on_odometry,
            qos,
        )

    def _publish_map_to_odom(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.rotation.w = 1.0
        self._static_broadcaster.sendTransform(transform)

    def _warn(self, reason: str) -> None:
        now = time.monotonic()
        if reason != self._last_warning_reason or now - self._last_warning_time >= 5.0:
            self.get_logger().warning(f"Dropping /ground_truth/odom: {reason}")
            self._last_warning_reason = reason
            self._last_warning_time = now

    def _on_odometry(self, message: Odometry) -> None:
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        accepted, reason = self._gate.accept(
            now_ns,
            stamp_ns,
            message.header.frame_id,
            message.child_frame_id,
        )
        if not accepted:
            self._warn(reason)
            return

        position = message.pose.pose.position
        if not all(math.isfinite(value) for value in (position.x, position.y)):
            self._warn("non-finite position")
            return
        orientation = message.pose.pose.orientation
        planar = planar_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )
        if planar is None:
            self._warn("invalid orientation")
            return

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = 0.0
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = planar
        self._broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
