import csv
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_71_detector_comparison import run


class DetectorComparisonTest(unittest.TestCase):
    def test_exports_all_detector_variants(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); h5 = root / "ss0_flat_range_doppler.h5"
            with h5py.File(h5, "w") as handle:
                axes = handle.create_group("axes"); axes.create_dataset("range_m", data=np.arange(16.0)); axes.create_dataset("velocity_mps", data=np.arange(16.0))
                rd = handle.create_group("range_doppler"); power = np.ones((2, 16, 16)); power[0, 8, 8] = 1000; rd.create_dataset("power_linear", data=power); rd.create_dataset("spectrum_complex", data=np.ones((2, 16, 16, 4, 4), dtype=np.complex64))
            output = root / "output"; summary = run(root, output)
            self.assertEqual(summary["case_count"], 1)
            with (output / "detector_comparison.csv").open(encoding="utf-8", newline="") as handle:
                detectors = {row["detector"] for row in csv.DictReader(handle)}
            self.assertEqual(detectors, {"ca_cfar_local_peak", "ca_threshold_only", "os_cfar_local_peak", "fixed_energy_local_peak"})


if __name__ == "__main__":
    unittest.main()
