import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_33_known_angle_fixture import run
from simulation.run_v04_53_hdf5_channel_order_validation import validate


class Hdf5ChannelOrderValidationTests(unittest.TestCase):
    def test_known_angle_fixture_is_loaded_and_ranked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); input_path = root / "known.h5"; output = root / "out"
            run(input_path, azimuth_deg=10.0, elevation_deg=2.0, slant_range_m=20.0, radial_velocity_mps=0.0, frames=8, samples=64)
            summary = validate(input_path, Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), output)
            self.assertEqual(summary["candidate_count"], 576)
            self.assertFalse(summary["source_is_hardware_measurement"])
            self.assertTrue((output / "hdf5_channel_order_candidates.csv").is_file())


if __name__ == "__main__":
    unittest.main()
