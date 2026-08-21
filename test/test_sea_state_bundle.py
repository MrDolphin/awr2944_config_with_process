import json
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_sea_state_bundle import run


class SeaStateBundleTest(unittest.TestCase):
    def _write_cloud(self, path: Path, scale: float) -> None:
        values = {
            "range_m": np.array([10.0, 20.0, 30.0]) * scale,
            "velocity_mps": np.array([-0.2, 0.0, 0.3]) * scale,
            "azimuth_deg": np.array([-2.0, 0.0, 3.0]) * scale,
            "elevation_deg": np.array([-0.5, 0.0, 0.7]) * scale,
            "power_linear": np.array([1.0, 2.0, 3.0]) * scale,
            "x_m": np.array([1.0, 2.0, 3.0]) * scale,
            "y_m": np.array([0.5, 1.0, 1.5]) * scale,
            "z_m": np.array([0.1, 0.2, 0.3]) * scale,
        }
        with h5py.File(path, "w") as handle:
            group = handle.create_group("point_cloud")
            for name, data in values.items():
                group.create_dataset(name, data=data)

    def test_bundle_writes_all_reports_and_figures(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline = root / "ss0.h5"
            rough = root / "ss3.h5"
            self._write_cloud(baseline, 1.0)
            self._write_cloud(rough, 2.0)
            output = root / "bundle"

            result = run([("ss0", baseline), ("ss3", rough)], output)

            self.assertEqual(result["status"], "completed_descriptive_sea_state_bundle")
            self.assertTrue((output / "bundle_summary.json").is_file())
            self.assertTrue((output / "README.md").is_file())
            self.assertTrue((output / "stats" / "point_cloud_stats.csv").is_file())
            self.assertTrue((output / "stats" / "output_analysis.md").is_file())
            self.assertTrue((output / "visual_report" / "sea_state_comparison.md").is_file())
            self.assertTrue((output / "visual_report" / "figures" / "sea_state_comparison.png").is_file())
            self.assertTrue((output / "visual_report" / "figures" / "ss3_point_cloud_3d.png").is_file())
            self.assertTrue((output / "interpretation" / "sea_state_interpretation.md").is_file())
            payload = json.loads((output / "interpretation" / "interpretation.json").read_text(encoding="utf-8"))
            self.assertEqual(len(payload["runs"]), 2)
            self.assertIn("wider_velocity_spread_than_baseline", payload["runs"][1]["descriptive_flags"])


if __name__ == "__main__":
    unittest.main()
