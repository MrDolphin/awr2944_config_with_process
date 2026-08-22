import tempfile
import unittest
from pathlib import Path


class CaptureArrivalScanV151Test(unittest.TestCase):
    def test_empty_known_angle_cases_are_scanned_without_decode(self):
        from simulation.run_v04_140_lvds_candidate_manifest_batch import run as make_batch
        from simulation.run_v04_151_capture_arrival_scan import run

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); cfg = root / "candidate.cfg"
            cfg.write_text("profileCfg 0 77 1 2 3 4 5 6 7 4 9 10 11\nchirpCfg 0 0 0 0 0 0 0 1\nframeCfg 0 0 1 4 100 1 0\nsensorStart\n", encoding="utf-8")
            cases = root / "cases"; make_batch(cfg, cases)
            result = run(cases, root / "scan")
            self.assertEqual(result["case_count"], 5)
            self.assertEqual(result["geometry_ready_count"], 0)
            self.assertFalse(result["decode_requested"])
            self.assertFalse(result["hardware_commands_executed"])


if __name__ == "__main__":
    unittest.main()
