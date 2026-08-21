import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_85_pcb_array_extract import extract


def _region(net: int, x: float, y: float) -> str:
    vertices = "|".join([
        f"VX0={x}mil", f"VY0={y}mil", f"VX1={x + 10}mil", f"VY1={y}mil",
        f"VX2={x + 10}mil", f"VY2={y + 20}mil", f"VX3={x}mil", f"VY3={y + 20}mil",
    ])
    return f"|RECORD=Region|NET={net}|LAYER=TOP|MAINCONTOURVERTEXCOUNT=4|{vertices}\n"


class PcbArrayExtractTest(unittest.TestCase):
    def test_extracts_all_rf_channels(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "fixture.PcbDoc"
            lines = ["|RECORD=Board|ORIGINX=100mil|ORIGINY=200mil|VX0=100mil|VY0=200mil|VX1=1000mil|VY1=200mil|VX2=1000mil|VY2=1000mil|VX3=100mil|VY3=1000mil\n"]
            for net, name in [(107, "TX4"), (108, "TX3"), (109, "TX2"), (110, "TX1"),
                              (117, "RX4"), (118, "RX3"), (119, "RX2"), (120, "RX1")]:
                lines.append(f"|RECORD=Pad|NET={net}|COMPONENT=727|NAME={name}|X=110mil|Y=210mil|LAYER=TOP\n")
                lines.append(_region(net, 200 + net, 300))
            source.write_text("".join(lines), encoding="utf-8")
            output = root / "output"
            result = extract(source, output)
            self.assertEqual(result["channel_count"], 8)
            self.assertEqual(result["tx_count"], 4)
            self.assertEqual(result["rx_count"], 4)
            self.assertFalse(result["hardware_validated"])
            rows = (output / "rf_array_candidates.csv").read_text(encoding="utf-8")
            self.assertIn("TX1", rows)
            self.assertIn("RX4", rows)
            virtual = (output / "virtual_array_candidates.csv").read_text(encoding="utf-8")
            self.assertEqual(virtual.count("\n") - 1, 16)
            summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["status"], "completed_pcb_rf_array_candidate_extraction")


if __name__ == "__main__":
    unittest.main()
