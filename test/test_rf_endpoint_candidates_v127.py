import tempfile
import unittest
from pathlib import Path


class RfEndpointCandidatesV127Test(unittest.TestCase):
    def test_all_eight_rf_networks_get_candidates(self):
        from simulation.run_v04_127_rf_endpoint_candidates import extract

        with tempfile.TemporaryDirectory() as directory:
            result = extract(Path("simulation/hardware/awr2944pev/v04_103_rf_network_trace/rf_network_primitives.csv"), Path("simulation/hardware/awr2944pev/v04_102_pcb_asset_audit/tx_rx_package_pads.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), Path(directory) / "out")
        self.assertEqual(result["row_count"], 8)
        self.assertFalse(result["phase_center_ready"])
        self.assertTrue(all(item["coordinate_status"].endswith("not_phase_center") for item in result["candidates"]))


if __name__ == "__main__":
    unittest.main()
