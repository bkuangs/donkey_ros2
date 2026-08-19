import json
import math
from dataclasses import dataclass


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


@dataclass
class CaptureDwell:
    radius: float
    duration: float
    started_at: float | None = None

    def update(self, distance, current_stamp):
        if distance > self.radius:
            self.started_at = None
        elif self.started_at is None:
            self.started_at = current_stamp
        elif current_stamp - self.started_at >= self.duration:
            return True
        return False


@dataclass
class TrackingErrors:
    position_squared: float = 0.0
    velocity_squared: float = 0.0
    samples: int = 0

    def add(self, estimate, target):
        position_error = math.hypot(
            estimate.pose.pose.position.x - target.pose.pose.position.x,
            estimate.pose.pose.position.y - target.pose.pose.position.y,
        )
        target_yaw = yaw_from_quaternion(target.pose.pose.orientation)
        body_velocity = target.twist.twist.linear
        target_velocity_x = (
            math.cos(target_yaw) * body_velocity.x
            - math.sin(target_yaw) * body_velocity.y
        )
        target_velocity_y = (
            math.sin(target_yaw) * body_velocity.x
            + math.cos(target_yaw) * body_velocity.y
        )
        velocity_error = math.hypot(
            estimate.twist.twist.linear.x - target_velocity_x,
            estimate.twist.twist.linear.y - target_velocity_y,
        )
        self.position_squared += position_error * position_error
        self.velocity_squared += velocity_error * velocity_error
        self.samples += 1

    def rmses(self):
        if not self.samples:
            return None, None
        return (
            math.sqrt(self.position_squared / self.samples),
            math.sqrt(self.velocity_squared / self.samples),
        )


def write_result(path, result):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    temporary_path.write_text(json.dumps(result, indent=2) + "\n")
    temporary_path.replace(path)
