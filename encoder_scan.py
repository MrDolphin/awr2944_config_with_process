"""Pure encoder-count planning for a cable-safe radar sweep.

Hardware adapters own GPIO and motor-driver details.  This module only turns a
measured encoder count into the next safe motor command, making the scan policy
testable without a Raspberry Pi or connected motor.
"""

from dataclasses import dataclass
from typing import Literal


CommandMode = Literal["forward", "reverse", "hold"]


@dataclass(frozen=True)
class SweepCommand:
    mode: CommandMode
    duty: float
    reason: str


class EncoderSweepPlan:
    """Plan a stop-settle-reverse sweep from absolute encoder counts.

    ``count=0`` corresponds to the calibrated minimum angle.  The caller must
    restore that reference with a homing sensor after power-up or when a fault
    is detected.
    """

    def __init__(
        self,
        *,
        counts_per_rev: int,
        min_angle_deg: float,
        max_angle_deg: float,
        cruise_duty: float = 0.45,
        approach_duty: float = 0.18,
        approach_window_deg: float = 15.0,
        settle_s: float = 0.25,
        require_capture_release: bool = False,
    ) -> None:
        if counts_per_rev <= 0:
            raise ValueError("counts_per_rev must be positive")
        if max_angle_deg <= min_angle_deg:
            raise ValueError("max_angle_deg must be greater than min_angle_deg")
        if not 0.0 < approach_duty <= cruise_duty <= 1.0:
            raise ValueError("duty values must satisfy 0 < approach <= cruise <= 1")
        if approach_window_deg < 0 or settle_s < 0:
            raise ValueError("approach_window_deg and settle_s must be non-negative")

        self.counts_per_rev = counts_per_rev
        self.min_count = round(min_angle_deg * counts_per_rev / 360.0)
        self.max_count = round(max_angle_deg * counts_per_rev / 360.0)
        self.approach_count = round(approach_window_deg * counts_per_rev / 360.0)
        self.cruise_duty = cruise_duty
        self.approach_duty = approach_duty
        self.settle_s = settle_s
        self.require_capture_release = require_capture_release
        self._direction: Literal["forward", "reverse"] = "forward"
        self._endpoint_since_s: float | None = None
        self._capture_ready = False

    def angle_for_count(self, count: int) -> float:
        """Return the physical scan angle, clamped to the configured sweep."""
        bounded_count = min(self.max_count, max(self.min_count, count))
        return bounded_count * 360.0 / self.counts_per_rev

    def release_capture(self) -> bool:
        """Allow travel away from a settled endpoint after a static frame is saved."""
        if not self.require_capture_release or not self._capture_ready:
            return False
        self._direction = "reverse" if self._direction == "forward" else "forward"
        self._endpoint_since_s = None
        self._capture_ready = False
        return True

    def command_for_count(self, count: int, *, now_s: float = 0.0) -> SweepCommand:
        """Return the safe command for the latest measured encoder count."""
        at_max = count >= self.max_count
        at_min = count <= self.min_count

        if (self._direction == "forward" and at_max) or (self._direction == "reverse" and at_min):
            endpoint = "max" if self._direction == "forward" else "min"
            if self._endpoint_since_s is None:
                self._endpoint_since_s = now_s
                return SweepCommand("hold", 0.0, f"{endpoint}_endpoint")
            if now_s - self._endpoint_since_s < self.settle_s:
                return SweepCommand("hold", 0.0, f"settling_at_{endpoint}")
            if self.require_capture_release:
                self._capture_ready = True
                return SweepCommand("hold", 0.0, f"capture_ready_at_{endpoint}")
            self._direction = "reverse" if self._direction == "forward" else "forward"
            self._endpoint_since_s = None
            return SweepCommand(self._direction, self.cruise_duty, f"settled_at_{endpoint}")

        self._endpoint_since_s = None
        if self._direction == "forward":
            duty = self.approach_duty if count >= self.max_count - self.approach_count else self.cruise_duty
            reason = "approaching_max" if duty == self.approach_duty else "scanning_forward"
        else:
            duty = self.approach_duty if count <= self.min_count + self.approach_count else self.cruise_duty
            reason = "approaching_min" if duty == self.approach_duty else "scanning_reverse"
        return SweepCommand(self._direction, duty, reason)
