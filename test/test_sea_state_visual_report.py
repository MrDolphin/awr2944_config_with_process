import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_sea_state_visual_report import run


class SeaStateVisualReportTests(unittest.TestCase):
    def test_generates_per_run_and_comparison_figures(self):
        fields = ("range_m", "velocity_mps", "azimuth_deg", "elevation_deg", "power_linear", "x_m", "y_m", "z_m")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); inputs = []
            for label in ("ss0", "ss1"):
                path = root / f"{label}.h5"
                with h5py.File(path, "w") as handle:
                    values = np.arange(3, dtype=float)
                    for field in fields:
                        handle.create_dataset(f"/point_cloud/{field}", data=values)
                inputs.append((label, path))
            output = root / "report"
            report = run(inputs, output)
            self.assertTrue(report.exists())
            self.assertTrue((output / "figures" / "ss0_range_velocity.png").exists())
            self.assertTrue((output / "figures" / "sea_state_comparison.png").exists())


if __name__ == "__main__":
    unittest.main()
