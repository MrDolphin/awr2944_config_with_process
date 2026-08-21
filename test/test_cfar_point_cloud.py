import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_cfar_point_cloud import run


class CfarPointCloudTests(unittest.TestCase):
    def test_detects_local_peak_and_writes_point_fields(self):
        geometry = Path(__file__).parents[1] / "simulation" / "hardware" / "awr2944pev" / "antgeometry_mapping.csv"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); input_h5 = root / "rd.h5"; output_h5 = root / "points.h5"
            spectrum = np.ones((16, 16, 4, 4), dtype=complex)
            spectrum[8, 8] = 100.0 + 0j
            power = np.mean(np.abs(spectrum) ** 2, axis=(2, 3))
            with h5py.File(input_h5, "w") as handle:
                handle.create_dataset("/range_doppler/power_linear", data=power)
                handle.create_dataset("/range_doppler/spectrum_complex", data=spectrum)
                handle.create_dataset("/axes/range_m", data=np.arange(16, dtype=float))
                handle.create_dataset("/axes/velocity_mps", data=np.arange(16, dtype=float))
            summary = run(input_h5, geometry, output_h5, training=(2, 2), guard=(1, 1), pfa=1e-2)
            self.assertGreaterEqual(summary["point_count"], 1)
            with h5py.File(output_h5, "r") as handle:
                self.assertIn("/point_cloud/range_m", handle)
                self.assertEqual(handle["/point_cloud/range_m"].shape[0], summary["point_count"])


if __name__ == "__main__":
    unittest.main()
