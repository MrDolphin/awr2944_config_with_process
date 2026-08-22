import json
import tempfile
import unittest
from pathlib import Path


class CenterIqDecodeGateV142Test(unittest.TestCase):
    def test_gate_does_not_decode_template(self):
        from simulation.run_v04_140_lvds_candidate_manifest_batch import run as make_batch
        from simulation.run_v04_142_center_iq_decode_gate import run

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cfg = root / "candidate.cfg"
            cfg.write_text("profileCfg 0 77 1 2 3 4 5 6 7 8 656 9 10 11\nchirpCfg 0 0 0 0 0 0 0 1\nframeCfg 0 0 1 0 656 100 1 0\nsensorStart\n", encoding="utf-8")
            make_batch(cfg, root / "batch")
            result = run(root / "batch" / "lv003_az+00_el-10_r10", root / "analysis")
            self.assertFalse(result["decode_started"])
            self.assertFalse(result["decoded"])
            self.assertEqual(result["status"], "decode_not_started_readiness_gate_failed")
            summary = json.loads((root / "analysis" / "summary.json").read_text(encoding="utf-8"))
            self.assertFalse(summary["decode_started"])


if __name__ == "__main__":
    unittest.main()
