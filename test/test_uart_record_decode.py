import struct
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_99_custom_record_audit import MAGIC
from simulation.run_v04_100_uart_record_decode import decode


class UartRecordDecodeTest(unittest.TestCase):
    def test_decodes_type1_point_from_uart_frame(self):
        with tempfile.TemporaryDirectory() as temp:
            body = struct.pack("<IIffff", 1, 16, 1.0, 2.0, 3.0, -0.5)
            packet_length = 40 + len(body)
            header = struct.pack("<8sIIIIIIII", MAGIC, 0x04070001, packet_length, 0x2944, 7, 0, 1, 1, 0)
            path = Path(temp) / "record.bin"; path.write_bytes(header + body)
            frames, points = decode(path)
            self.assertEqual(len(frames), 1)
            self.assertEqual(len(points), 1)
            self.assertEqual(points[0]["frame_num"], 7)
            self.assertAlmostEqual(points[0]["velocity_mps"], -0.5)


if __name__ == "__main__":
    unittest.main()
