import unittest
from pathlib import Path

from simulation.hardware_cfg import parse_cfg


class Awr2944CfgTests(unittest.TestCase):
    def test_project_cfg_snapshot_matches_fov_and_uncalibrated_state(self):
        cfg = Path("Config/profile_3d_3Azim_1ElevTx_awr2944P.cfg")
        snapshot = parse_cfg(cfg)
        self.assertEqual(snapshot.platform, "AWR2944P")
        self.assertEqual(snapshot.tx_mask, 15)
        self.assertEqual(snapshot.rx_mask, 15)
        self.assertEqual(snapshot.azimuth_fov_deg, (-90.0, 90.0))
        self.assertEqual(snapshot.elevation_fov_deg, (-90.0, 90.0))
        self.assertFalse(snapshot.calibration_measure_enabled)
        self.assertTrue(snapshot.calibration_compensation_is_identity)
