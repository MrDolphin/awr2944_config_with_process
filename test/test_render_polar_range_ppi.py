import json
import tempfile
import unittest
from pathlib import Path

from tools.render_polar_range_ppi import render_ppi


class RenderPolarRangePpiTests(unittest.TestCase):
    @staticmethod
    def _row(index, yaw, bins=(0, 10, 100, 20)):
        return {
            "record_index": index,
            "frame_num": index + 1,
            "host_monotonic_s": 100.0 + index * 0.1,
            "yaw_deg": yaw,
            "range_bins_raw": list(bins),
            "range_axis": "bin_index_uncalibrated",
            "value_units": "firmware_raw_uint16",
            "time_basis": "host_packet_parse_monotonic",
        }

    @staticmethod
    def _write(path, rows):
        path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")

    def test_observed_spokes_and_missing_coverage_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "polar_range_profiles.jsonl"
            self._write(source, [self._row(0, 0), self._row(1, 90)])
            result = render_ppi(source, root / "ppi", skip_bins=1)
            svg = (root / "ppi" / "intensity_ppi.svg").read_text(encoding="utf-8")
            self.assertEqual(result["spoke_count"], 2)
            self.assertEqual(result["largest_angular_gap_deg"], 270.0)
            self.assertEqual(result["observed_10deg_bins"], [0, 9])
            self.assertFalse(result["unobserved_bearings_filled"])
            self.assertFalse(result["rf_time_calibrated"])
            self.assertIn('data-spoke="0"', svg)
            self.assertIn('data-spoke="1"', svg)
            self.assertNotIn('data-spoke="2"', svg)

    def test_bad_input_does_not_create_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "polar_range_profiles.jsonl"
            bad = self._row(0, 0)
            bad["range_axis"] = "metres"
            for rows in ([bad], [self._row(0, 0), self._row(0, 90)],
                         [self._row(0, 0), self._row(1, 90, bins=(1, 2))]):
                self._write(source, rows)
                with self.assertRaises(ValueError):
                    render_ppi(source, root / "ppi")
                self.assertFalse((root / "ppi").exists())

    def test_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "polar_range_profiles.jsonl"
            self._write(source, [self._row(0, 0)])
            (root / "ppi").mkdir()
            with self.assertRaises(FileExistsError):
                render_ppi(source, root / "ppi")


if __name__ == "__main__":
    unittest.main()
