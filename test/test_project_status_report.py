import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_98_project_status_report import run


class ProjectStatusReportTest(unittest.TestCase):
    def test_report_preserves_unvalidated_hardware_boundary(self):
        with tempfile.TemporaryDirectory() as temp:
            result = run(Path(temp), Path(temp) / "output")
            self.assertEqual(result["status"], "completed_project_status_report")
            self.assertFalse(result["hardware_aoa_validated"])
            self.assertTrue((Path(temp) / "output" / "output_analysis.md").is_file())


if __name__ == "__main__":
    unittest.main()
