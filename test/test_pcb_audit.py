import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_28_pcb_audit import run


class PcbAuditTest(unittest.TestCase):
    def test_ascii_pcb_records_are_audited_without_claiming_phase_centres(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "sources"
            root.mkdir()
            (root / "PROC113D_ASCII.PcbDoc").write_text(
                "\n".join(
                    [
                        "|RECORD=Board|FILENAME=test|KIND=Protel_Advanced_PCB|VERSION=5.01|V9_STACK_LAYER0_NAME=Top Layer|V9_STACK_LAYER0_LAYERID=16777217|V9_STACK_LAYER1_NAME=Dielectric 1|V9_STACK_LAYER1_DIELCONST=3.0|V9_STACK_LAYER1_DIELHEIGHT=5mil",
                        "|RECORD=Net|ID=1|NAME=TX1_P",
                        "|RECORD=Net|ID=2|NAME=LVDS_TX0_P",
                        "|RECORD=Component|ID=0|X=10mil|Y=20mil|PATTERN=U1|SOURCEDESIGNATOR=U1",
                        "|RECORD=Pad|INDEXFORSAVE=0|LAYER=TOP|NAME=1|X=10mil|Y=20mil|XSIZE=5mil|YSIZE=5mil|SHAPE=ROUND|UNIQUEID=P1",
                    ]
                ),
                encoding="utf-8",
            )
            (root / "board.step").write_text("step", encoding="utf-8")
            output = Path(directory) / "audit"
            summary = run([root], output)
            self.assertEqual(summary["component_count"], 1)
            self.assertEqual(summary["net_count"], 2)
            self.assertFalse(summary["phase_center_coordinates_confirmed"])
            self.assertTrue((output / "source_manifest.csv").is_file())
            self.assertTrue((output / "layer_stack.csv").is_file())
            self.assertTrue((output / "output_analysis.md").is_file())
            with (output / "rf_net_inventory.csv").open(encoding="utf-8", newline="") as handle:
                self.assertEqual(len(list(csv.DictReader(handle))), 2)
            payload = json.loads((output / "audit_summary.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["record_counts"]["Pad"], 1)


if __name__ == "__main__":
    unittest.main()
