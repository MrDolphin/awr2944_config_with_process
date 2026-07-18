import unittest

from encoder_gpio import EncoderCounter, GpioMotorDriver
from encoder_scan import SweepCommand


class FakePwm:
    def __init__(self):
        self.value = None


class FakeDirection:
    def __init__(self):
        self.state = None

    def on(self):
        self.state = "on"

    def off(self):
        self.state = "off"


class EncoderCounterTests(unittest.TestCase):
    def test_a_edge_uses_b_phase_to_track_direction(self):
        counter = EncoderCounter()

        counter.on_a_edge(a_value=True, b_value=True)
        counter.on_a_edge(a_value=False, b_value=True)

        self.assertEqual(0, counter.count)


class GpioMotorDriverTests(unittest.TestCase):
    def setUp(self):
        self.pwm = FakePwm()
        self.direction = FakeDirection()
        self.driver = GpioMotorDriver(self.pwm, self.direction)

    def test_applies_at8236_forward_reverse_and_hold_commands(self):
        self.driver.apply(SweepCommand("forward", 0.45, "scan"))
        self.assertEqual("off", self.direction.state)
        self.assertEqual(0.45, self.pwm.value)

        self.driver.apply(SweepCommand("reverse", 0.45, "scan"))
        self.assertEqual("on", self.direction.state)
        self.assertEqual(0.55, self.pwm.value)

        self.driver.apply(SweepCommand("hold", 0.0, "capture_ready"))
        self.assertEqual("off", self.direction.state)
        self.assertEqual(0.0, self.pwm.value)


if __name__ == "__main__":
    unittest.main()
