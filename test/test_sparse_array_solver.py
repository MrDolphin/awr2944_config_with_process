import unittest
from pathlib import Path

import numpy as np

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.run_v04_39_sparse_array_solver import estimate_sparse_grid
from simulation.v03 import FmcwConfig
from simulation.v04 import generate_aoa_iq_with_positions


class SparseArraySolverTests(unittest.TestCase):
    def test_ideal_array_recovers_known_grid_angle(self):
        config = FmcwConfig()
        models = model_positions(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), config)
        x, y = models["ideal_half_lambda"]
        iq = generate_aoa_iq_with_positions(config, slant_range_m=30.0, radial_velocity_mps=0.0, azimuth_deg=20.0, elevation_deg=10.0, x_positions_m=x, y_positions_m=y)
        estimate = estimate_sparse_grid(config, np.mean(iq, axis=(0, 1)), x, y, np.arange(-60.0, 61.0, 1.0), np.arange(-20.0, 21.0, 1.0))
        self.assertEqual(estimate[:2], (20.0, 10.0))


if __name__ == "__main__":
    unittest.main()
