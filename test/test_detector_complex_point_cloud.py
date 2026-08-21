import tempfile
import unittest
from pathlib import Path
import h5py
import numpy as np

from simulation.run_v04_83_detector_complex_point_cloud import run


class DetectorComplexPointCloudTest(unittest.TestCase):
    def test_stage_contract(self):
        root = Path(__file__).resolve().parents[1]
        mapping = root / "simulation/hardware/awr2944pev/antgeometry_mapping.csv"
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name); input_root = temp / "input"; input_root.mkdir()
            with h5py.File(input_root / "case_range_doppler.h5", "w") as handle:
                power = np.ones((1, 16, 16), dtype=np.float32)
                spectrum = np.ones((1, 16, 16, 4, 4), dtype=np.complex64)
                handle.create_dataset("/range_doppler/power_linear", data=power)
                handle.create_dataset("/range_doppler/spectrum_complex", data=spectrum)
                handle.create_dataset("/axes/range_m", data=np.arange(16, dtype=float))
                handle.create_dataset("/axes/velocity_mps", data=np.arange(16, dtype=float))
            result = run(input_root, mapping, temp / "output", max_points_per_group=2)
            self.assertEqual(result["status"], "completed_detector_complex_bin_point_cloud")
            self.assertEqual(result["aoa_source"], "per_detection_complex_range_doppler_bin")
            self.assertTrue((temp / "output/detector_point_cloud_summary.csv").exists())


if __name__ == "__main__":
    unittest.main()
