"""GPIO adapters for the existing AT8236 motor and quadrature encoder wiring.

The module deliberately has no GPIO side effects at import time.  Hardware is
created only by a caller after a real-device acceptance check.
"""

from encoder_scan import SweepCommand


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
