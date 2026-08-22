import json
import tempfile
import unittest
from pathlib import Path


class LvdsCandidateManifestBatchV140Test(unittest.TestCase):
    def test_generates_five_review_only_cases(self):
        from simulation.run_v04_140_lvds_candidate_manifest_batch import run

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cfg = root / "candidate.cfg"
            cfg.write_text(
                "profileCfg 0 77 1 2 3 4 5 6 7 8 656 9 10 11\n"
                "chirpCfg 0 0 0 0 0 0 0 1\n"
                "frameCfg 0 0 1 0 656 100 1 0\n"
                "sensorStart\n",
                encoding="utf-8",
            )
            output = root / "out"
            result = run(cfg, output)
            self.assertEqual(result["case_count"], 5)
            self.assertFalse(result["authoritative_hardware_cfg"])
            for case in output.glob("lv*"):
                manifest = json.loads((case / "manifest.json").read_text(encoding="utf-8"))
                self.assertEqual(manifest["evidence_status"], "candidate_cfg_awaiting_operator_review")
                self.assertTrue((case / "candidate.cfg").exists())


if __name__ == "__main__":
    unittest.main()
