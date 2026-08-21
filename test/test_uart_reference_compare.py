import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_101_uart_reference_compare import run


class UartReferenceCompareTest(unittest.TestCase):
    def test_compares_matching_point_count_without_claiming_adc(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frames = root / "frames.csv"; points = root / "points.csv"; reference = root / "reference.jsonl"
            with frames.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=["source_file", "frame_num", "decoded_point_count"]); writer.writeheader(); writer.writerow({"source_file": "record_01.bin", "frame_num": 4, "decoded_point_count": 1})
            with points.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=["source_file", "frame_num", "x_m", "y_m", "z_m", "velocity_mps"]); writer.writeheader(); writer.writerow({"source_file": "record_01.bin", "frame_num": 4, "x_m": 1, "y_m": 2, "z_m": 3, "velocity_mps": 0})
            reference.write_text(json.dumps({"frame_num": 4, "points": [{"x": 1, "y": 2, "z": 3, "v": 0}]}) + "\n", encoding="utf-8")
            summary = run(frames, points, reference, root / "out")
            self.assertEqual(summary["point_count_match_frames"], 1)
            self.assertFalse(summary["hardware_aoa_validated"])


if __name__ == "__main__":
    unittest.main()
