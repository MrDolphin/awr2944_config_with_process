import tempfile
import unittest
from pathlib import Path


class ProjectStatusReportV123Test(unittest.TestCase):
    def test_report_requires_existing_stage_evidence(self):
        from simulation.run_v04_123_project_status_report import run
        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("."), Path(directory) / "out")
            self.assertEqual(result["status"], "completed_project_status_report")
            self.assertFalse(result["hardware_aoa_validated"])
            self.assertTrue((Path(directory) / "out" / "output_analysis.md").exists())


if __name__ == "__main__":
    unittest.main()
