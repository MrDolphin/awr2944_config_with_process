import unittest

import numpy as np

from simulation.run_v04_phase_sensitivity import scan
from simulation.v03 import FmcwConfig
from simulation.v04 import virtual_array_positions


class PhaseSensitivityTests(unittest.TestCase):
    def test_unity_effective_permittivity_has_no_route_phase(self):
        config = FmcwConfig()
        x, y = virtual_array_positions(config)
        result = scan(config, x, y, np.zeros(4), np.zeros(4), 1.0)
        self.assertAlmostEqual(result["phase_rms_deg"], 0.0, places=12)


if __name__ == "__main__":
    unittest.main()
