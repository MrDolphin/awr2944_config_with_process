import unittest
from pathlib import Path

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.run_v04_40_calibration_noise_peaks import apply_impairments, estimate_with_peaks
from simulation.v03 import FmcwConfig
from simulation.v04 import generate_aoa_iq_with_positions
import numpy as np


class CalibrationNoisePeakTests(unittest.TestCase):
    def test_clean_known_angle_has_nonzero_peak_margin(self):
        config = FmcwConfig()
        models = model_positions(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), config)
        x, y = models["ideal_half_lambda"]
        iq = generate_aoa_iq_with_positions(config, slant_range_m=30.0, radial_velocity_mps=0.0, azimuth_deg=20.0, elevation_deg=10.0, x_positions_m=x, y_positions_m=y)
        channel = apply_impairments(np.mean(iq, axis=(0, 1)), np.random.default_rng(1), 0.0, 0.0, None)
        result = estimate_with_peaks(config, channel, x, y, np.arange(-60.0, 61.0, 1.0), np.arange(-20.0, 21.0, 1.0))
        self.assertEqual((result["estimated_azimuth_deg"], result["estimated_elevation_deg"]), (20.0, 10.0))
        self.assertGreater(result["peak_to_second_db"], 0.0)


if __name__ == "__main__":
    unittest.main()
