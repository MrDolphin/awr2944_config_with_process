import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_34_geometry_semantics_scan import run


class GeometrySemanticsScanTest(unittest.TestCase):
    def test_pairwise_scan_records_models_without_selecting_hardware_truth(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mapping = root / "mapping.csv"
            rows = ["rx_index,tx_index,row,column,azimuth_spacing_lambda,elevation_spacing_lambda"]
            for rx in range(4):
                for tx in range(4):
                    rows.append(f"{rx},{tx},{rx},{tx},0.5,0.5")
            mapping.write_text("\n".join(rows), encoding="utf-8")
            output = root / "output"
            summary = run(mapping, output)
            self.assertEqual(len(summary["model_names"]), 6)
            self.assertFalse(summary["hardware_geometry_confirmed"])
            with (output / "geometry_semantics_pairwise.csv").open(encoding="utf-8", newline="") as handle:
                self.assertEqual(len(list(csv.DictReader(handle))), 36)
            payload = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            self.assertIn("tx_rx_regular", payload["best_assumed_by_actual"])


if __name__ == "__main__":
    unittest.main()
