import tempfile
import unittest
from pathlib import Path
import h5py

from simulation.run_v04_80_end_to_end_replay import run


class EndToEndReplayTest(unittest.TestCase):
    def test_round_trip_and_aoa_contract(self):
        root = Path(__file__).resolve().parents[1]
        mapping = root / "simulation/hardware/awr2944pev/antgeometry_mapping.csv"
        with tempfile.TemporaryDirectory() as temp_name:
            result = run(mapping, Path(temp_name), azimuth_deg=5.0, elevation_deg=2.0)
            self.assertEqual(result["raw_iq_contract_status"], "valid_contract")
            self.assertLess(abs(result["azimuth_error_deg"]), 0.1)
            self.assertLess(abs(result["elevation_error_deg"]), 0.1)
            with h5py.File(Path(temp_name) / "end_to_end_replay.h5", "r") as handle:
                self.assertEqual(handle["/radar/raw_iq"].shape, (16, 64, 4))
                self.assertEqual(handle["/radar/virtual_iq"].shape, (4, 64, 4, 4))


if __name__ == "__main__":
    unittest.main()
