import tempfile
import unittest
from pathlib import Path


class KnownAngleBatchGateV136Test(unittest.TestCase):
    def test_template_batch_is_blocked_by_missing_raw_iq(self):
        from simulation.run_v04_136_known_angle_batch_gate import run

        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("simulation/hardware/awr2944pev/v04_135_known_angle_manifest_batch"), Path("simulation/hardware/awr2944pev/virtual_array_coordinates.csv"), Path(directory) / "out")
        self.assertEqual(result["manifest_count"], 5)
        self.assertEqual(result["processed_capture_count"], 0)
        self.assertEqual(result["failure_count"], 5)
        self.assertFalse(result["hardware_aoa_validated"])


if __name__ == "__main__":
    unittest.main()
