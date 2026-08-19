"""Pure helpers for validating ground-truth odometry before publishing TF."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


def canonical_frame(frame: str) -> str:
    return frame.lstrip("/")


def planar_quaternion(
    quaternion: Sequence[float],
) -> Optional[Tuple[float, float, float, float]]:
    if len(quaternion) != 4 or not all(math.isfinite(value) for value in quaternion):
        return None
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-9:
        return None
    x, y, z, w = (value / norm for value in (x, y, z, w))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


@dataclass
class GroundTruthGate:
    max_age_ns: int
    future_tolerance_ns: int
    parent_frame: str = "odom"
    child_frame: str = "base_footprint"
    last_stamp_ns: Optional[int] = None
    last_now_ns: Optional[int] = None

    def accept(
        self,
        now_ns: int,
        stamp_ns: int,
        parent_frame: str,
        child_frame: str,
    ) -> Tuple[bool, str]:
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            self.last_stamp_ns = None
        self.last_now_ns = now_ns

        if canonical_frame(parent_frame) != self.parent_frame:
            return False, "unexpected parent frame"
        if canonical_frame(child_frame) != self.child_frame:
            return False, "unexpected child frame"
        if stamp_ns <= 0:
            return False, "zero or negative timestamp"
        if stamp_ns > now_ns + self.future_tolerance_ns:
            return False, "timestamp is in the future"
        if now_ns - stamp_ns > self.max_age_ns:
            return False, "stale odometry"
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return False, "duplicate or out-of-order odometry"

        self.last_stamp_ns = stamp_ns
        return True, ""
