import importlib
import logging
import sys
import threading
import types
import unittest
from unittest.mock import patch


class FakeSerialPort:
    def __init__(self):
        self.writes = []
        self.closed = False

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.closed = True


class Qd4310ProtocolTests(unittest.TestCase):
    def test_auto_resolves_unique_qdrive_on_each_start(self):
        from qd4310_motor import Qd4310Motor

        qdrive = lambda device, serial_number: types.SimpleNamespace(
            device=device, vid=0x0483, pid=0x5741,
            product="QDrive ComPort", description="QDrive ComPort",
            serial_number=serial_number,
        )
        xds110 = types.SimpleNamespace(
            device="/dev/ttyACM0", vid=0x0451, pid=0xBEF3,
            product="XDS110", description="XDS110", serial_number="RE450055",
        )
        motor = Qd4310Motor("auto")
        with patch("qd4310_motor._comports", side_effect=[
            [xds110, qdrive("/dev/ttyACM1", "old")],
            [xds110, qdrive("/dev/ttyACM2", "new")],
        ]), patch("qd4310_motor.serial.Serial", side_effect=[FakeSerialPort(), FakeSerialPort()]) as open_port:
            motor.start(1)
            motor.stop()
            motor.start(1)
            motor.stop()
        self.assertEqual([call.args[0] for call in open_port.call_args_list],
                         ["/dev/ttyACM1", "/dev/ttyACM2"])

    def test_auto_fails_closed_for_zero_or_multiple_qdrives(self):
        from qd4310_motor import Qd4310Motor

        qdrive = lambda device: types.SimpleNamespace(
            device=device, vid=0x0483, pid=0x5741,
            product="QDrive ComPort", description="QDrive ComPort",
            serial_number=device,
        )
        with patch("qd4310_motor._comports", side_effect=[
            [], [qdrive("/dev/ttyACM1"), qdrive("/dev/ttyACM2")],
        ]), patch("qd4310_motor.serial.Serial") as open_port:
            with self.assertRaisesRegex(RuntimeError, "No QD4310"):
                Qd4310Motor("auto").start(1)
            with self.assertRaisesRegex(RuntimeError, "Multiple QD4310"):
                Qd4310Motor("auto").start(1)
        open_port.assert_not_called()

    def test_auto_prefers_identity_bound_linux_by_id_path(self):
        from qd4310_motor import resolve_qd4310_port

        device = types.SimpleNamespace(
            device="/dev/ttyACM2", vid=0x0483, pid=0x5741,
            product="QDrive ComPort", description="QDrive ComPort",
            serial_number="NEW",
        )
        alias = "/dev/serial/by-id/usb-STMicroelectronics_QDrive_ComPort_NEW-if00"
        with patch("qd4310_motor._comports", return_value=[device]), patch(
            "qd4310_motor.os.path.isdir", return_value=True
        ), patch("qd4310_motor.os.listdir", return_value=[alias.rsplit("/", 1)[-1]]), patch(
            "qd4310_motor.os.path.realpath", side_effect=lambda path: "/dev/ttyACM2" if path == alias else path
        ):
            self.assertEqual(resolve_qd4310_port("auto"), alias)

    def test_start_update_and_stop_use_bounded_ascii_commands(self):
        from qd4310_motor import Qd4310Motor

        port = FakeSerialPort()
        with patch("qd4310_motor.serial.Serial", return_value=port) as open_port:
            motor = Qd4310Motor("/dev/serial/by-id/qd4310-test")
            motor.start(10)
            motor.set_speed(5.5)
            motor.stop()

        open_port.assert_called_once_with(
            "/dev/serial/by-id/qd4310-test", baudrate=115200,
            timeout=0.005, write_timeout=0.5,
        )
        self.assertEqual(
            port.writes,
            [b"enable\n", b"ctrl speed 10\n", b"ctrl speed 5.5\n",
             b"ctrl speed 0\n", b"disable\n"],
        )
        self.assertTrue(port.closed)

    def test_invalid_speed_never_opens_port(self):
        from qd4310_motor import Qd4310Motor

        with patch("qd4310_motor.serial.Serial") as open_port:
            motor = Qd4310Motor("/dev/ttyUSB9")
            for value in (0, -1, 31, float("nan"), "1\nreset"):
                with self.assertRaises(ValueError):
                    motor.start(value)
            open_port.assert_not_called()

    def test_short_serial_write_is_not_reported_as_success(self):
        from qd4310_motor import Qd4310Motor

        port = FakeSerialPort()
        port.write = lambda data: len(data) - 1
        with patch("qd4310_motor.serial.Serial", return_value=port):
            with self.assertRaises(OSError):
                Qd4310Motor("/dev/ttyUSB9").start(1)
        self.assertTrue(port.closed)

    def test_failed_speed_command_still_attempts_both_stop_commands(self):
        from qd4310_motor import Qd4310Motor

        port = FakeSerialPort()

        def write(data):
            port.writes.append(data)
            if data in (b"ctrl speed 1\n", b"ctrl speed 0\n"):
                raise OSError("serial failure")
            return len(data)

        port.write = write
        with patch("qd4310_motor.serial.Serial", return_value=port):
            with self.assertRaises(OSError):
                Qd4310Motor("/dev/ttyUSB9").start(1)
        self.assertEqual(port.writes, [b"enable\n", b"ctrl speed 1\n",
                                       b"ctrl speed 0\n", b"disable\n"])
        self.assertTrue(port.closed)


class Qd4310ServerModeTests(unittest.TestCase):
    def _import_server(self):
        previous_serial = sys.modules.get("serial")
        previous_websockets = sys.modules.get("websockets")
        sys.modules["serial"] = types.SimpleNamespace(Serial=object, SerialException=OSError)
        sys.modules["websockets"] = types.SimpleNamespace(
            exceptions=types.SimpleNamespace(ConnectionClosed=RuntimeError)
        )
        sys.modules.pop("radar_server", None)
        with patch("logging.FileHandler", return_value=logging.NullHandler()), patch("logging.basicConfig"):
            server = importlib.import_module("radar_server")
        return server, previous_serial, previous_websockets

    def _restore_modules(self, previous_serial, previous_websockets):
        sys.modules.pop("radar_server", None)
        if previous_serial is None:
            sys.modules.pop("serial", None)
        else:
            sys.modules["serial"] = previous_serial
        if previous_websockets is None:
            sys.modules.pop("websockets", None)
        else:
            sys.modules["websockets"] = previous_websockets

    def test_usb_mode_requires_server_configured_port(self):
        server, previous_serial, previous_websockets = self._import_server()
        try:
            ok, message = server.start_gimbal_scan({
                "mode": "qd4310", "rpm": 10, "qd4310_port": "/dev/ttyACM0"
            })
            self.assertFalse(ok)
            self.assertIn("port", message.lower())
            self.assertFalse(server.gimbal_scan["enabled"])
        finally:
            self._restore_modules(previous_serial, previous_websockets)

    def test_invalid_usb_rpm_returns_error_without_starting_thread(self):
        server, previous_serial, previous_websockets = self._import_server()
        try:
            server.gimbal_scan["config"]["qd4310_port"] = "/dev/ttyUSB9"
            with patch.object(server.threading, "Thread") as thread:
                ok, message = server.start_gimbal_scan({"mode": "qd4310", "rpm": "bad"})
            self.assertFalse(ok)
            self.assertIn("rpm", message.lower())
            thread.assert_not_called()
        finally:
            self._restore_modules(previous_serial, previous_websockets)

    def test_configured_usb_mode_selects_driver_and_updates_speed(self):
        server, previous_serial, previous_websockets = self._import_server()
        try:
            server.gimbal_scan["config"]["qd4310_port"] = "/dev/serial/by-id/qd4310-test"
            created = []

            class FakeThread:
                def __init__(self, *, target, args, daemon):
                    created.append((target, args, daemon))

                def start(self):
                    pass

            with patch.object(server.threading, "Thread", FakeThread):
                ok, _message = server.start_gimbal_scan({"mode": "qd4310", "rpm": 10})
            self.assertTrue(ok)
            self.assertIs(created[0][0], server.qd4310_scan_loop)
            self.assertEqual(created[0][1][0]["qd4310_port"], "/dev/serial/by-id/qd4310-test")
            fake_motor = types.SimpleNamespace(set_speed=lambda rpm: created.append(rpm))
            server.gimbal_scan["motor"] = fake_motor
            self.assertEqual(server.set_qd4310_speed(5.5), (True, "QD4310 speed updated"))
            self.assertEqual(created[-1], 5.5)
            self.assertEqual(server.gimbal_scan["rpm"], 5.5)
            self.assertFalse(server.set_qd4310_speed(31)[0])
            self.assertEqual(created[-1], 5.5)
        finally:
            self._restore_modules(previous_serial, previous_websockets)

    def test_usb_worker_stops_motor_when_stop_requested(self):
        server, previous_serial, previous_websockets = self._import_server()
        try:
            stop_event = threading.Event()
            calls = []

            class FakeMotor:
                def __init__(self, port):
                    calls.append(("port", port))

                def start(self, rpm):
                    calls.append(("start", rpm))
                    stop_event.set()

                def stop(self):
                    calls.append(("stop",))

            server.gimbal_scan["enabled"] = True
            with patch.object(server, "Qd4310Motor", FakeMotor):
                server.qd4310_scan_loop({"qd4310_port": "/dev/ttyUSB9", "rpm": 1}, stop_event)
            self.assertEqual(calls, [("port", "/dev/ttyUSB9"), ("start", 1), ("stop",)])
            self.assertFalse(server.gimbal_scan["enabled"])
            self.assertIsNone(server.gimbal_scan["motor"])
        finally:
            self._restore_modules(previous_serial, previous_websockets)

    def test_radar_port_inventory_excludes_qdrive_usb_device(self):
        server, previous_serial, previous_websockets = self._import_server()
        try:
            qdrive = "/dev/serial/by-id/qdrive-test"
            paths = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]
            with patch.object(server, "list_qdrive_ports", return_value=[]), patch("glob.glob", return_value=paths), patch.object(
                server.os.path, "realpath", side_effect=lambda path: "/dev/ttyACM0" if path == qdrive else path
            ):
                self.assertEqual(server.list_radar_acm_ports(qdrive), paths[1:])
        finally:
            self._restore_modules(previous_serial, previous_websockets)

    def test_auto_radar_port_inventory_excludes_detected_qdrive(self):
        server, previous_serial, previous_websockets = self._import_server()
        try:
            paths = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]
            with patch("glob.glob", return_value=paths), patch.object(
                server, "list_qdrive_ports", return_value=["/dev/ttyACM2"]
            ):
                self.assertEqual(server.list_radar_acm_ports("auto"), paths[:2])
                args = types.SimpleNamespace(qd4310_port="auto", cfg_port="/dev/ttyACM2", data_port="")
                with self.assertRaises(ValueError):
                    server.validate_radar_ports_not_motor(args)
        finally:
            self._restore_modules(previous_serial, previous_websockets)

    def test_explicit_radar_port_cannot_use_motor_device(self):
        server, previous_serial, previous_websockets = self._import_server()
        try:
            qdrive = "/dev/serial/by-id/qdrive-test"
            args = types.SimpleNamespace(qd4310_port=qdrive, cfg_port="/dev/ttyACM0", data_port="")
            with patch.object(server, "list_qdrive_ports", return_value=[]), patch.object(
                server.os.path, "realpath", side_effect=lambda path: "/dev/ttyACM0" if path == qdrive else path
            ):
                with self.assertRaises(ValueError):
                    server.validate_radar_ports_not_motor(args)
        finally:
            self._restore_modules(previous_serial, previous_websockets)

    def test_motor_only_server_does_not_probe_radar_ports(self):
        server, previous_serial, previous_websockets = self._import_server()
        try:
            args = types.SimpleNamespace(
                qd4310_port="/dev/serial/by-id/qdrive-test", enable_camera=True,
                camera_config="camera.cfg", camera_http_host="0.0.0.0",
                camera_http_port=8081, camera_public_base_url="http://pi:8081",
                ws_port=8765,
            )
            with patch.object(server, "auto_detect_ports") as probe, patch.object(
                server, "start_camera_services"
            ) as camera, patch.object(server, "run_websocket_server") as websocket:
                server.run_motor_only_server(args)
            probe.assert_not_called()
            camera.assert_called_once_with("camera.cfg", "0.0.0.0", 8081, "http://pi:8081")
            websocket.assert_called_once_with(8765)
            args.qd4310_port = ""
            with self.assertRaises(ValueError):
                server.run_motor_only_server(args)
        finally:
            self._restore_modules(previous_serial, previous_websockets)


if __name__ == "__main__":
    unittest.main()
