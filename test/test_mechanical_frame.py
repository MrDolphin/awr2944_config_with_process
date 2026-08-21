from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from pypdf import PdfWriter

from simulation.run_v04_88_mechanical_frame import run


class MechanicalFrameTest(unittest.TestCase):
    def test_step_bbox_and_traceable_outputs(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            step = root / "board.step"
            step.write_text(
                "#2810=MANIFOLD_SOLID_BREP('',#2811);"
                "#2811=CLOSED_SHELL('',(#2812,#2813,#2814,#2815));"
                "#2812=CARTESIAN_POINT('',(0,0,-1.4));"
                "#2813=CARTESIAN_POINT('',(85,0,0));"
                "#2814=CARTESIAN_POINT('',(0,125,0));"
                "#2815=CARTESIAN_POINT('',(85,125,-1.4));",
                encoding="ascii",
            )
            assembly = root / "assembly.pdf"
            schematic = root / "schematic.pdf"
            for pdf in (assembly, schematic):
                writer = PdfWriter()
                writer.add_blank_page(width=100, height=100)
                with pdf.open("wb") as handle:
                    writer.write(handle)

            output = root / "output"
            result = run(step, assembly, schematic, output)

            self.assertEqual(result["status"], "completed_sprr440_mechanical_frame_extraction")
            self.assertAlmostEqual(result["step_board"]["bbox_mm"]["x"]["size"], 85.0)
            self.assertAlmostEqual(result["step_board"]["bbox_mm"]["y"]["size"], 125.0)
            self.assertAlmostEqual(result["step_board"]["bbox_mm"]["z"]["size"], 1.4)
            self.assertEqual(result["coordinate_status"], "pcb_local_frame_confirmed_only")
            self.assertTrue((output / "mechanical_frame.json").is_file())
            self.assertTrue((output / "source_traceability.md").is_file())
            self.assertTrue((output / "output_analysis.md").is_file())


if __name__ == "__main__":
    unittest.main()
