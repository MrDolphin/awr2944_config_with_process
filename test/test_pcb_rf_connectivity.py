import csv
import tempfile
import unittest
from pathlib import Path

from simulation.build_pcb_rf_connectivity import run


class PcbRfConnectivityTests(unittest.TestCase):
    def test_track_endpoint_inside_region_is_connected(self):
        text = (
            "|RECORD=Pad|NET=110|X=0mil|Y=0mil|NAME=B3|\n"
            "|RECORD=Track|NET=110|X1=0mil|Y1=0mil|X2=5mil|Y2=5mil|\n"
            "|RECORD=Region|NET=110|VX1=4mil|VY1=4mil|VX2=8mil|VY2=4mil|VX3=8mil|VY3=8mil|VX4=4mil|VY4=8mil|\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pcb = root / "sample.PcbDoc"
            output = root / "out"
            pcb.write_text(text, encoding="utf-8")
            run(pcb, output)
            with (output / "rf_connectivity_summary.csv").open(encoding="utf-8", newline="") as handle:
                rows = {row["antenna"]: row for row in csv.DictReader(handle)}
            self.assertEqual(rows["TX1"]["pad_to_region_connected"], "True")
            self.assertAlmostEqual(float(rows["TX1"]["shortest_pad_to_region_mil"]), 7.0710678119, places=6)


if __name__ == "__main__":
    unittest.main()
