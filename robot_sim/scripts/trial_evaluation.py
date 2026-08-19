import json
import math
from dataclasses import dataclass


def normalize_yaw(yaw):
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class PlanarTransform:
    x: float
    y: float
    yaw: float

    def pose(self, x, y, yaw):
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        return (
            self.x + cosine * x - sine * y,
            self.y + sine * x + cosine * y,
            normalize_yaw(self.yaw + yaw),
        )

    def velocity(self, velocity_x, velocity_y):
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        return (
            cosine * velocity_x - sine * velocity_y,
            sine * velocity_x + cosine * velocity_y,
        )


def stamp_seconds(message):
    return message.header.stamp.sec + 1.0e-9 * message.header.stamp.nanosec


def localization_is_ready(truth_stamp, measured, tolerance):
    return measured is not None and abs(
        truth_stamp - stamp_seconds(measured)
    ) <= tolerance


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

    def add(self, estimate, target, estimate_transform=None):
        estimate_x = estimate.pose.pose.position.x
        estimate_y = estimate.pose.pose.position.y
        estimate_velocity_x = estimate.twist.twist.linear.x
        estimate_velocity_y = estimate.twist.twist.linear.y
        if estimate_transform is not None:
            estimate_x, estimate_y, _ = estimate_transform.pose(
                estimate_x,
                estimate_y,
                0.0,
            )
            estimate_velocity_x, estimate_velocity_y = (
                estimate_transform.velocity(
                    estimate_velocity_x,
                    estimate_velocity_y,
                )
            )
        position_error = math.hypot(
            estimate_x - target.pose.pose.position.x,
            estimate_y - target.pose.pose.position.y,
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
            estimate_velocity_x - target_velocity_x,
            estimate_velocity_y - target_velocity_y,
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


@dataclass
class LocalizationErrors:
    position_squared: float = 0.0
    yaw_squared: float = 0.0
    samples: int = 0
    maximum_position_error: float = 0.0
    maximum_yaw_error: float = 0.0
    final_position_error: float | None = None
    final_yaw_error: float | None = None
    discontinuities: int = 0
    previous_position: tuple[float, float] | None = None
    previous_yaw: float | None = None

    def add(
        self,
        measured_pose,
        truth_pose,
        discontinuity_position=0.75,
        discontinuity_yaw=0.75,
    ):
        measured_x, measured_y, measured_yaw = measured_pose
        truth_x, truth_y, truth_yaw = truth_pose
        position_error = math.hypot(
            measured_x - truth_x,
            measured_y - truth_y,
        )
        yaw_error = abs(normalize_yaw(measured_yaw - truth_yaw))
        if self.previous_position is not None:
            position_jump = math.hypot(
                measured_x - self.previous_position[0],
                measured_y - self.previous_position[1],
            )
            yaw_jump = abs(normalize_yaw(measured_yaw - self.previous_yaw))
            if (
                position_jump > discontinuity_position
                or yaw_jump > discontinuity_yaw
            ):
                self.discontinuities += 1
        self.previous_position = (measured_x, measured_y)
        self.previous_yaw = measured_yaw
        self.position_squared += position_error * position_error
        self.yaw_squared += yaw_error * yaw_error
        self.samples += 1
        self.maximum_position_error = max(
            self.maximum_position_error,
            position_error,
        )
        self.maximum_yaw_error = max(self.maximum_yaw_error, yaw_error)
        self.final_position_error = position_error
        self.final_yaw_error = yaw_error

    def metrics(self):
        if not self.samples:
            return {
                "position_rmse": None,
                "yaw_rmse": None,
                "maximum_position_error": None,
                "maximum_yaw_error": None,
                "final_position_error": None,
                "final_yaw_error": None,
            }
        return {
            "position_rmse": math.sqrt(
                self.position_squared / self.samples
            ),
            "yaw_rmse": math.sqrt(self.yaw_squared / self.samples),
            "maximum_position_error": self.maximum_position_error,
            "maximum_yaw_error": self.maximum_yaw_error,
            "final_position_error": self.final_position_error,
            "final_yaw_error": self.final_yaw_error,
        }


def write_result(path, result):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    temporary_path.write_text(json.dumps(result, indent=2) + "\n")
    temporary_path.replace(path)
