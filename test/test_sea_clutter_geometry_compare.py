import csv
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_87_sea_clutter_geometry_compare import run


class SeaClutterGeometryCompareTest(unittest.TestCase):
    def test_two_models_are_reported(self):
        candidate = Path("simulation/hardware/awr2944pev/v04_85_pcb_array_candidate/virtual_array_candidates.csv")
        if not candidate.is_file():
            self.skipTest("V0.4.85 candidate package is not present")
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "data"
            root.mkdir()
            with h5py.File(root / "ss0_flat_seed101.h5", "w") as handle:
                handle.create_dataset("/axes/time_s", data=np.asarray([0.0, 0.1]))
                handle.create_dataset("/axes/x_m", data=np.asarray([1.0, 2.0]))
                handle.create_dataset("/axes/y_m", data=np.asarray([1.0, 2.0]))
                handle.create_dataset("/truth/height_m", data=np.zeros((2, 2, 2)))
                handle.create_dataset("/installation/height_m", data=np.asarray([1.0]))
                handle.attrs["case_id"] = "ss0_flat"
            output = Path(tmp) / "output"
            result = run(root, candidate, output)
            self.assertEqual(result["case_count"], 1)
            self.assertEqual(len(result["models"]), 2)
            with (output / "sea_clutter_geometry_summary.csv").open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)
            self.assertTrue((output / "sea_clutter_geometry_comparison.png").is_file())


if __name__ == "__main__":
    unittest.main()
