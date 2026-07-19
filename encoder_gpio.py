"""GPIO adapters for the existing AT8236 motor and quadrature encoder wiring.

The module deliberately has no GPIO side effects at import time.  Hardware is
created only by a caller after a real-device acceptance check.
"""

from dataclasses import dataclass

from encoder_scan import EncoderSweepPlan, SweepCommand


class EncoderCounter:
    """Count quadrature encoder movement from both edges of channel A."""

    def __init__(self, initial_count: int = 0) -> None:
        self.count = initial_count

    def on_a_edge(self, *, a_value: bool, b_value: bool) -> int:
        if a_value == b_value:
            self.count += 1
        else:
            self.count -= 1
        return self.count


class GpioMotorDriver:
    """Apply sweep commands using the AT8236 polarity from the proven probe."""

    def __init__(self, pwm_device, direction_device) -> None:
        self._pwm = pwm_device
        self._direction = direction_device

    def apply(self, command: SweepCommand) -> None:
        if command.mode == "forward":
            self._direction.off()
            self._pwm.value = command.duty
        elif command.mode == "reverse":
            self._direction.on()
            self._pwm.value = 1.0 - command.duty
        else:
            self._pwm.value = 0.0
            self._direction.off()


@dataclass(frozen=True)
class EncoderSweepSnapshot:
    count: int
    angle_deg: float
    command: SweepCommand
    capture_ready: bool


class EncoderSweepSession:
    """Join measured encoder state, motion policy, and motor output at one seam."""

    def __init__(self, plan: EncoderSweepPlan, counter: EncoderCounter, driver: GpioMotorDriver) -> None:
        self._plan = plan
        self._counter = counter
        self._driver = driver

    def tick(self, *, now_s: float) -> EncoderSweepSnapshot:
        command = self._plan.command_for_count(self._counter.count, now_s=now_s)
        self._driver.apply(command)
        return EncoderSweepSnapshot(
            count=self._counter.count,
            angle_deg=self._plan.angle_for_count(self._counter.count),
            command=command,
            capture_ready=command.reason.startswith("capture_ready_"),
        )

    def release_capture(self) -> bool:
        return self._plan.release_capture()
