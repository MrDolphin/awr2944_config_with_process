import tempfile
import unittest
from pathlib import Path
import numpy as np

from simulation.run_v04_81_batch_replay import run


class BatchReplayTest(unittest.TestCase):
    def test_npz_batch_replay(self):
        root = Path(__file__).resolve().parents[1]
        mapping = root / "simulation/hardware/awr2944pev/antgeometry_mapping.csv"
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name); input_dir = temp / "input"; input_dir.mkdir()
            np.savez(input_dir / "sample.npz", iq=np.ones((8, 4, 4), dtype=np.complex64))
            result = run(input_dir, temp / "output", mapping)
            self.assertEqual(result["success_count"], 1)
            self.assertEqual(result["error_count"], 0)
            self.assertTrue((temp / "output/sample/replay.h5").exists())


if __name__ == "__main__":
    unittest.main()
