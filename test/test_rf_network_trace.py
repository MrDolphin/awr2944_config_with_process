import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_103_rf_network_trace import trace


class RfNetworkTraceTest(unittest.TestCase):
    def test_traces_pad_track_and_arc_but_not_phase_center(self):
        text = (
            "|RECORD=Net|ID=1|NAME=TX1|"
            "|RECORD=Pad|NET=1|COMPONENT=7|NAME=A1|X=10mil|Y=20mil|LAYER=TOP|"
            "|RECORD=Track|NET=1|X1=10mil|Y1=20mil|X2=100mil|Y2=20mil|WIDTH=8mil|LAYER=TOP|"
            "|RECORD=Arc|NET=1|LOCATION.X=100mil|LOCATION.Y=20mil|RADIUS=10mil|STARTANGLE=0|ENDANGLE=90|WIDTH=8mil|LAYER=TOP|"
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            pcb = root / "board.PcbDoc"
            out = root / "out"
            pcb.write_text(text, encoding="utf-8")
            result = trace(pcb, out)
            self.assertEqual(result["primitive_count"], 3)
            self.assertEqual(result["primitive_types_found"], ["Arc", "Pad", "Track"])
            self.assertFalse(result["phase_center_ready"])


if __name__ == "__main__":
    unittest.main()
