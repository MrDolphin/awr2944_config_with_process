import importlib
import logging
import sys
import types
import unittest
from unittest.mock import patch


class EncoderServerModeTests(unittest.TestCase):
    def _import_server(self):
        previous_serial = sys.modules.get("serial")
        previous_websockets = sys.modules.get("websockets")
        sys.modules["serial"] = types.SimpleNamespace(Serial=object)
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

    def test_encoder_mode_is_rejected_until_explicitly_enabled(self):
        server, previous_serial, previous_websockets = self._import_server()
        try:
            ok, message = server.start_gimbal_scan(
                {"mode": "encoder", "counts_per_rev": 360}
            )

            self.assertFalse(ok)
            self.assertIn("disabled", message)
        finally:
            self._restore_modules(previous_serial, previous_websockets)

    def test_enabled_encoder_mode_selects_encoder_loop(self):
        server, previous_serial, previous_websockets = self._import_server()
        try:
            created = []

            class FakeThread:
                def __init__(self, *, target, args, daemon):
                    created.append((target, args, daemon))

                def start(self):
                    pass

            with patch.object(server.threading, "Thread", FakeThread):
                ok, _message = server.start_gimbal_scan(
                    {"mode": "encoder", "encoder_enabled": True, "counts_per_rev": 360}
                )

            self.assertTrue(ok)
            self.assertIs(created[0][0], server.encoder_scan_loop)
            self.assertEqual("encoder", server.gimbal_scan["config"]["mode"])
            self.assertTrue(server.gimbal_scan["config"]["encoder_enabled"])
        finally:
            self._restore_modules(previous_serial, previous_websockets)


if __name__ == "__main__":
    unittest.main()
