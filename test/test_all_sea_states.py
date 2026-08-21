import csv
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_67_all_sea_states import run


class AllSeaStatesTest(unittest.TestCase):
    def test_zero_detection_cases_are_reported_not_dropped(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            summary = root / "case_summary.csv"
            h5_zero = root / "ss0.h5"; h5_nonzero = root / "ss3.h5"
            with h5py.File(h5_zero, "w") as handle:
                group = handle.create_group("point_cloud")
                for key in ("frame", "range_m", "velocity_mps", "azimuth_deg", "elevation_deg"):
                    group.create_dataset(key, data=np.empty((0,)))
            with h5py.File(h5_nonzero, "w") as handle:
                group = handle.create_group("point_cloud")
                group.create_dataset("frame", data=np.array([1, 1]))
                group.create_dataset("range_m", data=np.array([5.0, 6.0]))
                group.create_dataset("velocity_mps", data=np.array([0.0, 0.1]))
                group.create_dataset("azimuth_deg", data=np.array([1.0, 2.0]))
                group.create_dataset("elevation_deg", data=np.array([0.5, 0.6]))
            summary.write_text("case_id,output_h5,frames\nss0_flat," + str(h5_zero) + ",2\nss3_nominal," + str(h5_nonzero) + ",2\n", encoding="utf-8")
            result = run(root, root / "output")
            self.assertEqual(result["case_count"], 2)
            self.assertEqual(result["zero_detection_case_count"], 1)
            with (root / "output" / "all_sea_state_coverage.csv").open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)


if __name__ == "__main__":
    unittest.main()
