import tempfile
import unittest
from pathlib import Path


class CaptureCompletenessV111Test(unittest.TestCase):
    def test_empty_capture_scaffold_is_zero_ready(self):
        from simulation.run_v04_114_capture_completeness_v111 import run
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan = root / "plan.csv"
            plan.write_text("case_id\naz001\n", encoding="utf-8")
            result = run(plan, root / "captures", root / "out")
            self.assertEqual(result["planned_count"], 1)
            self.assertEqual(result["ready_for_v0112_count"], 0)


if __name__ == "__main__":
    unittest.main()
