import tempfile
import unittest
from pathlib import Path


class LvdsCandidateCfgV139Test(unittest.TestCase):
    def test_candidate_adds_only_lvds_line_and_keeps_source_unchanged(self):
        from simulation.run_v04_139_lvds_candidate_cfg import run

        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "source.cfg"; source.write_text("profileCfg 0 77 1 2 3 4 5 6 7 656 8 9 10 11\nsensorStart\n", encoding="utf-8")
            original = source.read_text(encoding="utf-8")
            result = run(source, Path(directory) / "out")
            candidate = (Path(directory) / "out" / "source.cfg").read_text(encoding="utf-8")
            source_after = source.read_text(encoding="utf-8")
        self.assertTrue(result["inserted"])
        self.assertEqual(source_after, original)
        self.assertIn("lvdsStreamCfg -1 0 1 0", candidate)
        self.assertFalse(result["authoritative_hardware_cfg"])


if __name__ == "__main__":
    unittest.main()
