import tempfile
import unittest
from pathlib import Path


class RealCaptureHandoffV134Test(unittest.TestCase):
    def test_first_five_cases_are_generated(self):
        from simulation.run_v04_134_real_capture_handoff import run

        with tempfile.TemporaryDirectory() as directory:
            result = run(Path(directory) / "out")
        self.assertEqual(result["case_count"], 5)
        self.assertFalse(result["real_capture_present"])


if __name__ == "__main__":
    unittest.main()
