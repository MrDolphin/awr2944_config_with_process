import csv
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_76_rf_channel_mapping import run


class RfChannelMappingTest(unittest.TestCase):
    def test_mapping_has_all_virtual_inputs(self):
        root = Path(__file__).resolve().parents[1]
        pads = root / "simulation/hardware/awr2944pev/v04_73_new_pcb_package/rf_ports/rf_net_pads.csv"
        cfg = root / "Config/profile_3d_3Azim_1ElevTx_awr2944P.cfg"
        with tempfile.TemporaryDirectory() as temp:
            result = run(pads, cfg, Path(temp))
            self.assertEqual(result["mapping_count"], 16)
            self.assertEqual(result["unique_virtual_inputs"], 16)
            with (Path(temp) / "rf_channel_mapping_candidate.csv").open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([row["tx_enable_bit"] for row in rows[0:4]], ["1"] * 4)
            self.assertEqual([row["tx_enable_bit"] for row in rows[4:8]], ["4"] * 4)
            self.assertTrue(all(row["mapping_status"].endswith("not_channel_calibrated") for row in rows))


if __name__ == "__main__":
    unittest.main()
