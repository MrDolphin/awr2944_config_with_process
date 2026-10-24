import csv
import json
from pathlib import Path
import subprocess
import struct
import sys
import tempfile
import unittest

from tools.register_scan_capture import register_capture


class RegisterScanCaptureTests(unittest.TestCase):
    def _make_capture(self, root: Path) -> tuple[Path, Path]:
        capture = root / "capture"
        capture.mkdir()
        frames = [
            {"record_index": 0, "frame_num": 10, "host_monotonic_s": 0.02},
            {"record_index": 1, "frame_num": 11, "host_monotonic_s": 0.06},
            {"record_index": 2, "frame_num": 12, "host_monotonic_s": 0.12},
        ]
        (capture / "frames.jsonl").write_text(
            "".join(json.dumps(frame) + "\n" for frame in frames), encoding="utf-8"
        )
        with (capture / "points.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=[
                "record_index", "host_time_s", "frame_num", "point_index",
                "x_m", "y_m", "z_m", "v_mps", "snr_db", "noise_db",
            ])
            writer.writeheader()
            writer.writerow({"record_index": 0, "host_time_s": 1, "frame_num": 10,
                             "point_index": 0, "x_m": 1, "y_m": 2, "z_m": 3,
                             "v_mps": 0, "snr_db": 12, "noise_db": 4})
            writer.writerow({"record_index": 1, "host_time_s": 1.04, "frame_num": 11,
                             "point_index": 0, "x_m": 0, "y_m": 1, "z_m": 0,
                             "v_mps": 0, "snr_db": 14, "noise_db": 5})
        samples = root / "motor_samples.jsonl"
        samples.write_text("".join(json.dumps(sample) + "\n" for sample in [
            {"query_started_monotonic_s": 0.0, "query_completed_monotonic_s": 0.0,
             "angle_deg": 0.0},
            {"query_started_monotonic_s": 0.04, "query_completed_monotonic_s": 0.04,
             "angle_deg": 6.0},
            {"query_started_monotonic_s": 0.08, "query_completed_monotonic_s": 0.08,
             "angle_deg": 12.0},
        ]), encoding="utf-8")
        return capture, samples

    def test_writes_registered_frames_and_points_without_filling_missing_sector(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture, samples = self._make_capture(root)
            output = root / "registered"
            summary = register_capture(
                capture, samples, output, direction=1, zero_deg=0,
                mounting_yaw_deg=0, mounting_roll_deg=90,
            )
            self.assertEqual(summary["frame_count"], 3)
            self.assertEqual(summary["registered_frames"], 2)
            self.assertEqual(summary["unregistered_frames"], 1)
            self.assertEqual(summary["registered_points"], 2)
            self.assertFalse(summary["rf_time_calibrated"])
            self.assertEqual(summary["time_basis"], "host_packet_parse_monotonic")
            self.assertEqual(summary["statuses"], {"registered": 2, "unbracketed": 1})
            frame_rows = [json.loads(line) for line in (output / "registered_frames.jsonl")
                          .read_text(encoding="utf-8").splitlines()]
            self.assertAlmostEqual(frame_rows[0]["yaw_deg"], 3.0)
            self.assertEqual(frame_rows[2]["status"], "unbracketed")
            self.assertIsNone(frame_rows[2]["yaw_deg"])
            with (output / "registered_points.csv").open(newline="", encoding="utf-8") as handle:
                point_rows = list(csv.DictReader(handle))
            self.assertEqual([row["record_index"] for row in point_rows], ["0", "1"])
            self.assertEqual(point_rows[0]["snr_db"], "12")
            self.assertAlmostEqual(float(point_rows[0]["z_m"]), -1.0)
            self.assertEqual((capture / "frames.jsonl").read_text(encoding="utf-8").count("\n"), 3)

    def test_direct_cli_accepts_measured_angle_direction(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture, samples = self._make_capture(root)
            output = root / "registered"
            script = Path(__file__).resolve().parents[1] / "tools" / "register_scan_capture.py"
            subprocess.run([
                sys.executable, str(script), str(capture), str(samples), str(output),
                "--angle-direction", "increasing", "--zero-deg", "0",
                "--mounting-yaw-deg", "0", "--mounting-roll-deg", "90",
            ], check=True, capture_output=True, text=True)
            self.assertTrue((output / "summary.json").exists())

    def test_extracts_uncalibrated_range_profile_from_recorded_raw_tlv(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture, samples = self._make_capture(root)
            profile = struct.pack("<HHHH", 100, 200, 300, 400)
            packet = struct.pack(
                "<8sIIIIIIII", b"\x02\x01\x04\x03\x06\x05\x08\x07",
                0, 40 + 8 + len(profile), 0, 10, 0, 0, 1, 0,
            ) + struct.pack("<II", 2, len(profile)) + profile
            (capture / "frames.tlv").write_bytes(packet)
            frames_path = capture / "frames.jsonl"
            frames = [json.loads(line) for line in frames_path.read_text(encoding="utf-8").splitlines()]
            frames[0]["raw_offset"] = 0
            frames[0]["raw_length"] = len(packet)
            frames_path.write_text("".join(json.dumps(row) + "\n" for row in frames), encoding="utf-8")
            output = root / "registered"
            summary = register_capture(capture, samples, output, direction=1,
                                       zero_deg=0, mounting_yaw_deg=0, mounting_roll_deg=0)
            profiles = [json.loads(line) for line in (output / "polar_range_profiles.jsonl")
                        .read_text(encoding="utf-8").splitlines()]
            self.assertEqual(summary["range_profile_frames"], 1)
            self.assertEqual(profiles[0]["range_bins_raw"], [100, 200, 300, 400])
            self.assertAlmostEqual(profiles[0]["yaw_deg"], 3.0)
            self.assertEqual(profiles[0]["range_axis"], "bin_index_uncalibrated")

    def test_refuses_existing_output_and_mismatched_point_record(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture, samples = self._make_capture(root)
            output = root / "registered"
            output.mkdir()
            with self.assertRaises(FileExistsError):
                register_capture(capture, samples, output, direction=1,
                                 zero_deg=0, mounting_yaw_deg=0, mounting_roll_deg=0)
            output.rmdir()
            points = capture / "points.csv"
            points.write_text(points.read_text(encoding="utf-8").replace("0,1,10,0,", "99,1,10,0,"),
                              encoding="utf-8")
            with self.assertRaises(ValueError):
                register_capture(capture, samples, output, direction=1,
                                 zero_deg=0, mounting_yaw_deg=0, mounting_roll_deg=0)

    def test_rejects_invalid_direction_even_when_no_frame_is_bracketed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture, samples = self._make_capture(root)
            samples.write_text(
                '{"query_started_monotonic_s":1,"query_completed_monotonic_s":1,"angle_deg":0}\n'
                '{"query_started_monotonic_s":2,"query_completed_monotonic_s":2,"angle_deg":1}\n',
                encoding="utf-8",
            )
            output = root / "registered"
            with self.assertRaises(ValueError):
                register_capture(capture, samples, output, direction=0,
                                 zero_deg=0, mounting_yaw_deg=0, mounting_roll_deg=0)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
