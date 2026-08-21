import csv
import tempfile
import unittest
from pathlib import Path

from simulation.extract_pcb_rf_ports import extract


class PcbRfPortExtractionTests(unittest.TestCase):
    def test_extracts_pad_coordinates_without_matching_pad_offsets(self):
        text = (
            "|RECORD=Pad|NET=110|COMPONENT=727|LAYER=TOP|NAME=B3|X=5778.4656mil|Y=4565.6692mil|"
            "PADXOFFSET0=0mil|PADYOFFSET0=0mil\n"
            "|RECORD=Track|NET=110|X1=5778.4656mil|Y1=4565.6692mil|X2=6000mil|Y2=4565mil|\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pcb = root / "sample.PcbDoc"
            output = root / "out"
            pcb.write_text(text, encoding="utf-8")
            summary = extract(pcb, output)
            self.assertEqual(summary["rf_pad_count"], 1)
            with (output / "rf_net_pads.csv").open(encoding="utf-8", newline="") as handle:
                row = next(csv.DictReader(handle))
            self.assertAlmostEqual(float(row["x_mil"]), 5778.4656)
            self.assertAlmostEqual(float(row["y_mil"]), 4565.6692)


if __name__ == "__main__":
    unittest.main()
