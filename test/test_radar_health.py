import unittest

from radar_health import RadarHealthMonitor


class RadarHealthMonitorTests(unittest.TestCase):
    def test_reports_offline_waiting_streaming_and_stale_states(self):
        now = [100.0]
        monitor = RadarHealthMonitor(clock=lambda: now[0], monotonic=lambda: now[0])
        self.assertEqual(monitor.snapshot()["data_status"], "offline")
        monitor.mark_serial_open("/dev/ttyACM1")
        self.assertEqual(monitor.snapshot()["data_status"], "waiting_for_frame")
        monitor.mark_bytes(64)
        monitor.mark_frame(12)
        self.assertEqual(monitor.snapshot()["data_status"], "streaming")
        now[0] += 2.1
        self.assertEqual(monitor.snapshot()["data_status"], "stale")
        monitor.mark_serial_open("/dev/ttyACM2")
        self.assertEqual(monitor.snapshot()["data_status"], "waiting_for_frame")

    def test_records_error_and_preserves_recording_quality(self):
        monitor = RadarHealthMonitor(clock=lambda: 10.0, monotonic=lambda: 10.0)
        monitor.mark_serial_open("/dev/ttyACM1")
        monitor.mark_error("Input/output error", disconnected=True)
        snapshot = monitor.snapshot(recording={"dropped_frames": 3, "writer_error": "disk full"})
        self.assertEqual(snapshot["data_status"], "offline")
        self.assertEqual(snapshot["parser_errors"], 1)
        self.assertEqual(snapshot["recording"]["dropped_frames"], 3)
        monitor.mark_serial_open("/dev/ttyACM1")
        monitor.mark_frame(9)
        self.assertIsNone(monitor.snapshot()["last_error"])


if __name__ == "__main__":
    unittest.main()
