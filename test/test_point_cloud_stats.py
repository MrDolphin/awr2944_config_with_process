import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_point_cloud_stats import run


class PointCloudStatsTests(unittest.TestCase):
    def test_compares_two_labeled_runs(self):
        fields = ("range_m", "velocity_mps", "azimuth_deg", "elevation_deg", "power_linear", "x_m", "y_m", "z_m")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); paths = []
            for index, count in enumerate((2, 3)):
                path = root / f"run{index}.h5"
                with h5py.File(path, "w") as handle:
                    for field in fields:
                        handle.create_dataset(f"/point_cloud/{field}", data=np.arange(count, dtype=float))
                paths.append(path)
            summaries = run([("ss0", paths[0]), ("ss1", paths[1])], root / "out")
            self.assertEqual([item["point_count"] for item in summaries], [2, 3])
            self.assertEqual((root / "out" / "point_cloud_stats.csv").exists(), True)


if __name__ == "__main__":
    unittest.main()
