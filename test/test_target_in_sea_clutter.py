import unittest
from pathlib import Path

from simulation.run_v04_46_target_in_sea_clutter import SCENARIOS, run_case
from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.v03 import FmcwConfig


class TargetInSeaClutterTests(unittest.TestCase):
    def test_target_sweep_returns_detection_probability_field(self):
        config = FmcwConfig(samples_per_chirp=128, chirps_per_frame=64)
        positions = model_positions(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), config)["pcb_centroid_candidate"]
        result = run_case(Path("simulation/hardware/awr2944pev/v04_43_sea_range_doppler/ss3_upper_range_doppler.h5"), SCENARIOS[0], positions, config, 10.0, 2.0, 20.0, 1.0)
        self.assertGreaterEqual(result["detection_probability"], 0.0)
        self.assertLessEqual(result["detection_probability"], 1.0)

    def test_controlled_high_snr_target_is_detectable(self):
        config = FmcwConfig(samples_per_chirp=128, chirps_per_frame=64)
        positions = model_positions(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), config)["pcb_centroid_candidate"]
        result = run_case(Path("simulation/hardware/awr2944pev/v04_43_sea_range_doppler/ss0_flat_range_doppler.h5"), SCENARIOS[-1], positions, config, 10.0, 2.0, 20.0, 1.0)
        self.assertGreater(result["detection_probability"], 0.0)


if __name__ == "__main__":
    unittest.main()
