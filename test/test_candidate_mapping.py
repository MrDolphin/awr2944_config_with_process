import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_29_candidate_mapping import run


class CandidateMappingTest(unittest.TestCase):
    def test_maps_named_rf_regions_to_virtual_channels_and_keeps_candidate_status(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mapping = root / "mapping.csv"
            mapping.write_text(
                "virtual_input_index,tx_index,tx_name,rx_index,rx_name,row,column,azimuth_spacing_lambda,elevation_spacing_lambda\n"
                "0,0,TX1,0,RX1,0,0,0.5,0.8\n"
                "1,0,TX1,1,RX2,0,1,0.5,0.8\n",
                encoding="utf-8",
            )
            regions = root / "regions.csv"
            regions.write_text(
                "antenna,center_x_mm,center_y_mm\nTX1,10,20\nRX1,0,20\nRX2,2,20\n",
                encoding="utf-8",
            )
            output = root / "output"
            summary = run(mapping, regions, output)
            self.assertEqual(summary["channel_count"], 2)
            self.assertTrue(summary["all_channels_have_named_regions"])
            self.assertFalse(summary["phase_center_confirmed"])
            with (output / "channel_mapping_candidates.csv").open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["mapping_status"], "candidate_network_and_region_geometry")
            payload = json.loads((output / "mapping_summary.json").read_text(encoding="utf-8"))
            self.assertIn(payload["best_shape_transform"], {"identity", "mirror_x", "mirror_y", "rotate_180"})


if __name__ == "__main__":
    unittest.main()
