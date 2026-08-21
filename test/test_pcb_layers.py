import csv
import tempfile
import unittest
from pathlib import Path

from simulation.analyze_pcb_layers import run


class PcbLayerAnalysisTests(unittest.TestCase):
    def test_extracts_stack_and_rf_layers(self):
        text = (
            "|RECORD=Board|V9_STACK_LAYER0_NAME=Top Layer|V9_STACK_LAYER0_LAYERID=16777217|"
            "V9_STACK_LAYER0_COPTHICK=1.6mil|V9_STACK_LAYER1_NAME=Dielectric 1|"
            "V9_STACK_LAYER1_DIELCONST=3.000|V9_STACK_LAYER1_DIELHEIGHT=5mil|"
            "V9_STACK_LAYER1_DIELMATERIAL=RO3003\n"
            "|RECORD=Track|NET=110|LAYER=TOP|X1=0mil|Y1=0mil|X2=1mil|Y2=1mil|\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pcb = root / "sample.PcbDoc"
            output = root / "out"
            pcb.write_text(text, encoding="utf-8")
            run(pcb, output)
            with (output / "layer_stack.csv").open(encoding="utf-8", newline="") as handle:
                layers = list(csv.DictReader(handle))
            with (output / "rf_object_layers.csv").open(encoding="utf-8", newline="") as handle:
                rf = list(csv.DictReader(handle))
            self.assertEqual(layers[1]["dielmaterial"], "RO3003")
            self.assertEqual(rf[0]["layer"], "TOP")


if __name__ == "__main__":
    unittest.main()
