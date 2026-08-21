import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_86_geometry_comparison import run


class GeometryComparisonTest(unittest.TestCase):
    def test_candidate_vs_ideal_contract(self):
        candidate = Path("simulation/hardware/awr2944pev/v04_85_pcb_array_candidate/virtual_array_candidates.csv")
        if not candidate.is_file():
            self.skipTest("V0.4.85 candidate package is not present")
        with tempfile.TemporaryDirectory() as tmp:
            result = run(candidate, Path(tmp))
            self.assertEqual(result["candidate_virtual_channels"], 16)
            self.assertIn("ideal_v04", result["beam_3db_width_az_deg"])
            self.assertIn("pcb_candidate", result["beam_3db_width_az_deg"])
            self.assertEqual(len(result["known_angle_metrics"]), 2)
            self.assertTrue((Path(tmp) / "beam_comparison.png").is_file())
            self.assertEqual(json.loads((Path(tmp) / "summary.json").read_text(encoding="utf-8"))["status"], "completed_pcb_candidate_geometry_comparison")


if __name__ == "__main__":
    unittest.main()
