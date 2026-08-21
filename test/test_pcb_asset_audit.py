import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_102_pcb_asset_audit import audit


class PcbAssetAuditTest(unittest.TestCase):
    def test_extracts_chip_tx_rx_package_pads_without_claiming_phase_centers(self):
        text = (
            "|RECORD=Board|ORIGINX=0mil|ORIGINY=0mil|VX0=0mil|VY0=0mil|VX1=100mil|VY1=0mil|"
            "|RECORD=Component|ID=7|SOURCEDESIGNATOR=U1|SOURCEDESCRIPTION=XA2944BGALT|X=10mil|Y=20mil|"
            "|RECORD=Net|ID=1|NAME=TX1|"
            "|RECORD=Net|ID=2|NAME=RX1|"
            "|RECORD=Pad|COMPONENT=7|NAME=A1|NET=1|X=11mil|Y=21mil|LAYER=TOP|"
            "|RECORD=Pad|COMPONENT=7|NAME=A2|NET=2|X=12mil|Y=22mil|LAYER=TOP|"
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            pcb = root / "board.PcbDoc"
            out = root / "out"
            pcb.write_text(text, encoding="utf-8")
            result = audit(pcb, out)
            self.assertEqual(result["tx_rx_package_pad_count"], 2)
            self.assertIn("TX/RX antenna phase-center coordinates", result["not_yet_proven"])
            self.assertEqual(json.loads((out / "summary.json").read_text())["chip"]["SOURCEDESCRIPTION"], "XA2944BGALT")


if __name__ == "__main__":
    unittest.main()
