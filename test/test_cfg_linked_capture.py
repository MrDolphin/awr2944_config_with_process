import unittest

from tools.dca1000_capture import parse_radar_cfg
from simulation.run_v04_51_cfg_linked_capture import derive_capture_contract


class CfgLinkedCaptureTests(unittest.TestCase):
    def test_project_awr2944p_cfg_derives_tdm_contract(self):
        parsed = parse_radar_cfg("Config/profile_3d_3Azim_1ElevTx_awr2944P.cfg")
        contract = derive_capture_contract(parsed)
        self.assertEqual(contract["rx_count"], 4)
        self.assertEqual(contract["samples_per_chirp"], 656)
        self.assertEqual(contract["chirps_per_frame"], 64)
        self.assertEqual(contract["tx_sequence"], (0, 2, 3, 1))
        self.assertEqual(contract["bytes_per_frame"], 656 * 4 * 64 * 4)


if __name__ == "__main__":
    unittest.main()
