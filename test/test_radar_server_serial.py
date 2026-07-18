import importlib
import struct
import sys
import threading
import types
import unittest


class RadarSerialIntegrationTests(unittest.TestCase):
    def test_empty_packet_is_forwarded_with_header_time_and_raw_bytes(self):
        stop_event = threading.Event()
        packet = struct.pack(
            "<8sIIIIIIII",
            b"\x02\x01\x04\x03\x06\x05\x08\x07",
            0, 40, 0, 42, 1234, 0, 0, 0,
        )

        class FakeSerial:
            def __init__(self, *args, **kwargs):
                self.buffer = bytearray(packet)

            @property
            def in_waiting(self):
                return len(self.buffer)

            def read(self, _count):
                data = bytes(self.buffer)
                self.buffer.clear()
                stop_event.set()
                return data

            def reset_input_buffer(self):
                pass

        previous_serial = sys.modules.get("serial")
        previous_websockets = sys.modules.get("websockets")
        sys.modules["serial"] = types.SimpleNamespace(Serial=FakeSerial)
        sys.modules["websockets"] = types.SimpleNamespace(
            exceptions=types.SimpleNamespace(ConnectionClosed=RuntimeError)
        )
        sys.modules.pop("radar_server", None)
        try:
            server = importlib.import_module("radar_server")
            captured = []
            server.record_pointcloud_frame = lambda frame, raw_packet: captured.append((frame, raw_packet))
            previous_disabled = server.logger.disabled
            server.logger.disabled = True
            try:
                server.radar_serial_thread("fake", 3125000, stop_event=stop_event)
            finally:
                server.logger.disabled = previous_disabled
        finally:
            sys.modules.pop("radar_server", None)
            if previous_serial is None:
                sys.modules.pop("serial", None)
            else:
                sys.modules["serial"] = previous_serial
            if previous_websockets is None:
                sys.modules.pop("websockets", None)
            else:
                sys.modules["websockets"] = previous_websockets

        self.assertEqual(len(captured), 1)
        frame, raw_packet = captured[0]
        self.assertEqual(raw_packet, packet)
        self.assertEqual(frame["frame_num"], 42)
        self.assertEqual(frame["device_time_cpu_cycles"], 1234)
        self.assertEqual(frame["detected_object_count"], 0)
        self.assertEqual(frame["points"], [])


if __name__ == "__main__":
    unittest.main()
