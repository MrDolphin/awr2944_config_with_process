import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_104_antenna_class_audit import audit


class AntennaClassAuditTest(unittest.TestCase):
    def test_extracts_members_and_enabled_width_rule(self):
        text = (
            "|RECORD=Class|NAME=Antenna|M0=TX4|M1=TX3|M2=TX2|M3=TX1|M4=RX4|M5=RX3|M6=RX2|M7=RX1|"
            "|RECORD=DXPRule|NAME=Antenna|RULEKIND=Width|ENABLED=TRUE|MINLIMIT=8.4mil|MAXLIMIT=10mil|"
            "PREFEREDWIDTH=10mil|TOPLAYER_PREFWIDTH=8.4mil|SCOPE1EXPRESSION=InNetClass('Antenna')|"
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            pcb = root / "board.PcbDoc"
            out = root / "out"
            pcb.write_text(text, encoding="utf-8")
            result = audit(pcb, out)
            self.assertTrue(result["class_members_match_expected"])
            self.assertEqual(result["enabled_width_rule"]["MINLIMIT"], "8.4mil")


if __name__ == "__main__":
    unittest.main()
