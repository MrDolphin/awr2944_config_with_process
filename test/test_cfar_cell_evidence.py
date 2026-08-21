import csv
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_69_cfar_cell_evidence import run


class CfarCellEvidenceTest(unittest.TestCase):
    def test_reports_boundary_and_threshold_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            h5 = root / "ss0_flat_range_doppler.h5"
            with h5py.File(h5, "w") as handle:
                axes = handle.create_group("axes"); axes.create_dataset("range_m", data=np.arange(16.0)); axes.create_dataset("velocity_mps", data=np.arange(16.0))
                rd = handle.create_group("range_doppler")
                power = np.ones((2, 16, 16)); power[0, 8, 8] = 1000
                rd.create_dataset("power_linear", data=power); rd.create_dataset("spectrum_complex", data=np.ones((2, 16, 16, 4, 4), dtype=np.complex64))
            output = root / "output"; summary = run(root, output, top_n=1)
            self.assertEqual(summary["case_count"], 1)
            with (output / "cfar_cell_evidence.csv").open(encoding="utf-8", newline="") as handle:
                row = next(csv.DictReader(handle))
            self.assertIn(row["valid_for_cfar"], {"True", "False"})
            self.assertIn("threshold_linear", row)


if __name__ == "__main__":
    unittest.main()
