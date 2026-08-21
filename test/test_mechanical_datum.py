import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_35_mechanical_datum import run


class MechanicalDatumTest(unittest.TestCase):
    def test_extracts_board_datum_and_marks_rf_coordinates_non_phase_center(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pcb = root / "board.PcbDoc"
            pcb.write_text(
                "\n".join([
                    "|RECORD=Board|ORIGINX=100mil|ORIGINY=200mil|VX0=100mil|VY0=200mil|VX1=1100mil|VY1=200mil|VX2=1100mil|VY2=1200mil|VX3=100mil|VY3=1200mil",
                    "|RECORD=Component|SOURCEDESIGNATOR=H1|PATTERN=MountHole|X=150mil|Y=250mil|SOURCEDESCRIPTION=Mounting hole",
                ]), encoding="utf-8")
            regions = root / "regions.csv"
            regions.write_text("antenna,center_x_mil,center_y_mil\nTX1,300,400\n", encoding="utf-8")
            output = root / "output"
            summary = run(pcb, regions, output)
            self.assertAlmostEqual(summary["board_width_mm"], 25.4)
            self.assertAlmostEqual(summary["board_height_mm"], 25.4)
            self.assertFalse(summary["phase_center_confirmed"])
            with (output / "rf_regions_board_relative.csv").open(encoding="utf-8", newline="") as handle:
                row = next(csv.DictReader(handle))
            self.assertEqual(row["status"], "mechanical_board_relative_not_phase_center")
            payload = json.loads((output / "mechanical_datum.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["mechanical_reference_candidate_count"], 1)


if __name__ == "__main__":
    unittest.main()
