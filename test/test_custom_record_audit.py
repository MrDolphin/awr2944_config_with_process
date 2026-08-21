import struct
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_99_custom_record_audit import HEADER_BYTES, MAGIC, audit


class CustomRecordAuditTest(unittest.TestCase):
    def test_detects_custom_headers_and_sequence_gaps(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "record.bin"
            packets = []
            for sequence in (5, 6, 8):
                header = struct.pack("<8sIIIIIIII", MAGIC, 0x04070001, HEADER_BYTES + 4, 0x2944, sequence, 0, 8, 6, 0)
                packets.append(header + b"DATA")
            path.write_bytes(b"".join(packets))
            result = audit(path)
            self.assertEqual(result["packet_count"], 3)
            self.assertEqual(result["sequence_gap_count"], 1)
            self.assertEqual(result["header_bytes_assumed"], 40)


if __name__ == "__main__":
    unittest.main()
