import tempfile
import unittest
from pathlib import Path


class PcbSourceReconciliationTest(unittest.TestCase):
    def test_supplied_package_has_ascii_pcb_and_rf_nets(self):
        from simulation.run_v04_126_pcb_source_reconciliation import reconcile

        sprr440 = Path(r"C:\Users\56461\Downloads\2944p资料\sprr440a (1)")
        sprr441 = Path(r"C:\Users\56461\Downloads\2944p资料\sprr441a")
        if not sprr440.exists() or not sprr441.exists():
            self.skipTest("supplied external PCB package is not available")
        with tempfile.TemporaryDirectory() as directory:
            result = reconcile(sprr440, sprr441, Path(directory) / "out")
        self.assertGreater(result["file_count"], 0)
        self.assertTrue(result["ascii_pcb_candidates"])
        self.assertEqual(result["ascii_pcb_candidates"][0]["ascii_pcb"]["target_rf_pad_count"], 8)
        self.assertFalse(result["phase_center_ready"])


if __name__ == "__main__":
    unittest.main()
