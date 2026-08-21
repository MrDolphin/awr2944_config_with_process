import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_97_data_source_audit import run


class DataSourceAuditTest(unittest.TestCase):
    def test_empty_roots_are_reported_without_hardware_claim(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            summary = run([root / "missing"], root / "output")
            self.assertEqual(summary["file_count"], 0)
            self.assertFalse(summary["hardware_aoa_validated"])
            self.assertTrue((root / "output" / "data_source_audit.csv").is_file())


if __name__ == "__main__":
    unittest.main()
