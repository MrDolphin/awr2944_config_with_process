import json
from pathlib import Path
import struct
import tempfile
import unittest

from tools.render_tlv2_range_time import render_capture


MAGIC = b"\x02\x01\x04\x03\x06\x05\x08\x07"


def _packet(frame_num, bins):
    payload = struct.pack(f"<{len(bins)}H", *bins)
    size = 40 + 8 + len(payload)
    return (struct.pack("<8sIIIIIIII", MAGIC, 0, size, 0, frame_num, 0, 0, 1, 0)
            + struct.pack("<II", 2, len(payload)) + payload)


class RenderTlv2RangeTimeTests(unittest.TestCase):
    def test_stationary_capture_produces_contrast_heatmap_not_a_ppi(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = root / "capture"
            capture.mkdir()
            packets = [_packet(10, [2500, 2500, 4500, 4500]),
                       _packet(11, [2500, 2500, 4500, 4500])]
            (capture / "frames.tlv").write_bytes(b"".join(packets))
            offset = 0
            frames = []
            for index, packet in enumerate(packets):
                frames.append({"record_index": index, "frame_num": index + 10,
                               "host_monotonic_s": 100 + index * 0.1,
                               "raw_offset": offset, "raw_length": len(packet)})
                offset += len(packet)
            (capture / "frames.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in frames), encoding="utf-8")
            (capture / "motor_samples.jsonl").write_text("", encoding="utf-8")
            summary = render_capture(capture, root / "analysis")
            svg = (root / "analysis" / "range_time_contrast.svg").read_text(encoding="utf-8")
            self.assertEqual(summary["profile_frames"], 2)
            self.assertEqual(summary["range_bin_count"], 4)
            self.assertEqual(summary["display_lower_raw"], 2500)
            self.assertEqual(summary["display_upper_raw"], 4500)
            self.assertFalse(summary["ppi_generated"])
            self.assertFalse(summary["intensity_filter_applied"])
            self.assertIn('fill="#001800"', svg)
            self.assertIn('fill="#00ff55"', svg)
            with self.assertRaises(FileExistsError):
                render_capture(capture, root / "analysis")


if __name__ == "__main__":
    unittest.main()
