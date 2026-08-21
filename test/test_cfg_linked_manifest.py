import unittest
from pathlib import Path


class CfgLinkedManifestTest(unittest.TestCase):
    def test_reads_current_profile_geometry(self):
        from simulation.run_v04_111_cfg_linked_manifest import parse_cfg
        cfg = parse_cfg(Path("Config/profile_3d_3Azim_1ElevTx_awr2944P.cfg"))
        self.assertEqual(cfg["adc_samples"], 656)
        self.assertEqual(cfg["chirps_per_frame"], 64)
        self.assertEqual(cfg["tx_order_bit_index"], [0, 2, 3, 1])


if __name__ == "__main__":
    unittest.main()
