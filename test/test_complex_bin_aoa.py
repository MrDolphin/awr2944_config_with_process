import tempfile
import unittest
from pathlib import Path
import h5py

from simulation.run_v04_80_end_to_end_replay import run as make_replay
from simulation.run_v04_82_complex_bin_aoa import run


class ComplexBinAoaTest(unittest.TestCase):
    def test_peak_bin_aoa_is_close_to_known_angle(self):
        root = Path(__file__).resolve().parents[1]
        mapping = root / "simulation/hardware/awr2944pev/antgeometry_mapping.csv"
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name)
            make_replay(mapping, temp / "input", azimuth_deg=5.0, elevation_deg=2.0)
            result = run(temp / "input/end_to_end_replay.h5", mapping, temp / "output")
            self.assertLess(abs(result["estimated_azimuth_deg"] - 5.0), 0.1)
            self.assertLess(abs(result["estimated_elevation_deg"] - 2.0), 0.1)
            with h5py.File(temp / "output/complex_bin_aoa.h5", "r") as handle:
                self.assertEqual(handle["/range_doppler/spectrum_complex"].shape, (4, 32, 4, 4))


if __name__ == "__main__":
    unittest.main()
