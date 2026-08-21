import csv
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_68_cfar_energy_matrix import run


class CfarEnergyMatrixTest(unittest.TestCase):
    def test_exports_pre_cfar_energy_and_sweep_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            h5 = root / "ss0_flat_range_doppler.h5"
            with h5py.File(h5, "w") as handle:
                axes = handle.create_group("axes")
                axes.create_dataset("range_m", data=np.arange(16, dtype=float))
                axes.create_dataset("velocity_mps", data=np.arange(16, dtype=float))
                rd = handle.create_group("range_doppler")
                rd.create_dataset("power_linear", data=np.ones((2, 16, 16)))
                rd.create_dataset("spectrum_complex", data=np.ones((2, 16, 16, 16), dtype=np.complex64))
            output = root / "output"
            summary = run(root, output)
            self.assertEqual(summary["case_count"], 1)
            with (output / "pre_cfar_energy.csv").open(encoding="utf-8", newline="") as handle:
                row = next(csv.DictReader(handle))
            self.assertAlmostEqual(float(row["power_p99_linear"]), 1.0)
            self.assertTrue((output / "cfar_energy_matrix.csv").exists())


if __name__ == "__main__":
    unittest.main()
