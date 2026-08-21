import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_105_rf_endpoint_candidates import trace


class RfEndpointCandidatesTest(unittest.TestCase):
    def test_ranks_degree_one_endpoint_and_marks_not_phase_center(self):
        text = (
            "|RECORD=Net|ID=1|NAME=TX1|"
            "|RECORD=Pad|NET=1|COMPONENT=727|X=10mil|Y=20mil|"
            "|RECORD=Track|NET=1|X1=10mil|Y1=20mil|X2=100mil|Y2=20mil|"
        )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            pcb = root / "board.PcbDoc"
            out = root / "out"
            pcb.write_text(text, encoding="utf-8")
            result = trace(pcb, out)
            self.assertEqual(result["candidate_count"], 2)
            self.assertFalse(result["phase_centers_validated"])
            self.assertEqual(result["candidates_by_net"]["TX1"][0]["x_mil"], 100.0)


if __name__ == "__main__":
    unittest.main()
