import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_38_pcb_array_comparison import load_pcb_virtual
from simulation.v03 import FmcwConfig


class PcbArrayComparisonTests(unittest.TestCase):
    def test_pcb_candidate_has_expected_shape_and_origin(self):
        config = FmcwConfig()
        path = Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv")
        x, y = load_pcb_virtual(path, config)
        self.assertEqual(x.shape, (4, 4))
        self.assertEqual(y.shape, (4, 4))
        self.assertAlmostEqual(float(x[0, 0]), 0.0, places=9)
        self.assertAlmostEqual(float(y[0, 0]), 0.0, places=9)
        self.assertGreater(float(x.max()), 0.1)


if __name__ == "__main__":
    unittest.main()
