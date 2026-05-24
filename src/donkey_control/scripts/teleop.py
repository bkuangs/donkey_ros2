#!/usr/bin/env python3

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            "/cmd/teleop",
            10,
        )

        self.speed = 0.0
        self.steering_angle = 0.0

        self.speed_step = 0.05
        self.steering_step = 0.05

        self.max_speed = 0.5
        self.max_steering = 0.5

        self.get_logger().info("Keyboard teleop started")
        self.get_logger().info("W/S: throttle, A/D: steering, SPACE: stop, Q: quit")

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def publish_command(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle

        self.publisher.publish(msg)

    def handle_key(self, key):
        if key == "w":
            self.speed += self.speed_step
        elif key == "s":
            self.speed -= self.speed_step
        elif key == "a":
            self.steering_angle += self.steering_step
        elif key == "d":
            self.steering_angle -= self.steering_step
        elif key == " ":
            self.speed = 0.0
            self.steering_angle = 0.0

        self.speed = self.clamp(self.speed, -self.max_speed, self.max_speed)
        self.steering_angle = self.clamp(
            self.steering_angle,
            -self.max_steering,
            self.max_steering,
        )

        self.publish_command()

        self.get_logger().info(
            f"speed={self.speed:.2f}, steering={self.steering_angle:.2f}"
        )


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return key


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    try:
        while rclpy.ok():
            key = get_key()

            if key == "q":
                break

            node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass

    finally:
        node.speed = 0.0
        node.steering_angle = 0.0
        node.publish_command()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()