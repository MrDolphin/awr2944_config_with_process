import csv
import unittest
from pathlib import Path
import numpy as np

from simulation.run_v04_75_coordinate_transform_validation import _load_cad, candidates
from simulation.v03 import FmcwConfig


class CoordinateTransformValidationTest(unittest.TestCase):
    def test_candidate_count_and_shape(self):
        root = Path(__file__).resolve().parents[1]
        cad = root / "simulation/hardware/awr2944pev/v04_73_new_pcb_package/cad_virtual_array_coordinates.csv"
        cfg = FmcwConfig()
        x, y = _load_cad(cad, cfg)
        result = candidates(x, y, x, y)
        self.assertEqual(len(result), 16)
        self.assertTrue(all(xx.shape == (4, 4) and yy.shape == (4, 4) for xx, yy in result.values()))
        self.assertTrue(np.isfinite(np.stack([v[0] for v in result.values()])).all())


if __name__ == "__main__": unittest.main()
