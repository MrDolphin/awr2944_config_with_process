import unittest

from encoder_gpio import (
    EncoderCounter,
    EncoderSweepSession,
    GpioMotorDriver,
    open_encoder_sweep_session,
)
from encoder_scan import EncoderSweepPlan
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


class FakeInput:
    def __init__(self, pin):
        self.pin = pin
        self.value = False
        self.when_activated = None
        self.when_deactivated = None


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


class EncoderSweepSessionTests(unittest.TestCase):
    def test_exposes_measured_angle_and_capture_ready_before_reversing(self):
        counter = EncoderCounter(initial_count=180)
        pwm = FakePwm()
        direction = FakeDirection()
        session = EncoderSweepSession(
            EncoderSweepPlan(
                counts_per_rev=360,
                min_angle_deg=0,
                max_angle_deg=180,
                settle_s=0.25,
                require_capture_release=True,
            ),
            counter,
            GpioMotorDriver(pwm, direction),
        )

        session.tick(now_s=10.0)
        snapshot = session.tick(now_s=10.25)

        self.assertEqual(180.0, snapshot.angle_deg)
        self.assertTrue(snapshot.capture_ready)
        self.assertEqual("hold", snapshot.command.mode)
        self.assertTrue(session.release_capture())
        self.assertEqual("reverse", session.tick(now_s=10.26).command.mode)

    def test_gpio_session_wires_encoder_a_edges_without_importing_gpiozero(self):
        inputs = {}

        def input_factory(pin):
            device = FakeInput(pin)
            inputs[pin] = device
            return device

        plan = EncoderSweepPlan(counts_per_rev=360, min_angle_deg=0, max_angle_deg=180)
        session, resources = open_encoder_sweep_session(
            plan,
            pwm_gpio=12,
            direction_gpio=17,
            encoder_a_gpio=22,
            encoder_b_gpio=23,
            pwm_factory=lambda _pin, **_kwargs: FakePwm(),
            direction_factory=lambda _pin: FakeDirection(),
            input_factory=input_factory,
        )

        inputs[22].value = True
        inputs[23].value = True
        inputs[22].when_activated()

        self.assertEqual(1, resources.counter.count)
        self.assertEqual(1.0, session.tick(now_s=1.0).angle_deg)


if __name__ == "__main__":
    unittest.main()
