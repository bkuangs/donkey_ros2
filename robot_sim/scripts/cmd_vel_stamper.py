#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelStamper(Node):
    def __init__(self):
        super().__init__("cmd_vel_stamper")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter(
            "output_topic",
            "/ackermann_steering_controller/reference",
        )

        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )

        self.publisher = self.create_publisher(TwistStamped, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.cmd_vel_callback,
            10,
        )

    def cmd_vel_callback(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.publisher.publish(stamped)


def main():
    rclpy.init()
    node = CmdVelStamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()