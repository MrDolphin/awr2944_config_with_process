import json
import tempfile
import unittest
from pathlib import Path


class CenterCaptureReadinessV141Test(unittest.TestCase):
    def test_template_is_not_admitted_without_capture(self):
        from simulation.run_v04_140_lvds_candidate_manifest_batch import run as make_batch
        from simulation.run_v04_141_center_capture_readiness import run

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cfg = root / "candidate.cfg"
            cfg.write_text("profileCfg 0 77 1 2 3 4 5 6 7 8 656 9 10 11\nchirpCfg 0 0 0 0 0 0 0 1\nframeCfg 0 0 1 0 656 100 1 0\nsensorStart\n", encoding="utf-8")
            batch = root / "batch"
            make_batch(cfg, batch)
            result = run(batch / "lv003_az+00_el-10_r10", root / "analysis")
            self.assertFalse(result["ready_for_decode"])
            self.assertIn("capture_missing", result["issues"])

    def test_complete_metadata_and_nonempty_capture_are_admitted_for_decode(self):
        from simulation.run_v04_140_lvds_candidate_manifest_batch import run as make_batch
        from simulation.run_v04_141_center_capture_readiness import run

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cfg = root / "candidate.cfg"
            cfg.write_text("profileCfg 0 77 1 2 3 4 5 6 7 8 656 9 10 11\nchirpCfg 0 0 0 0 0 0 0 1\nframeCfg 0 0 1 0 656 100 1 0\nsensorStart\n", encoding="utf-8")
            case = root / "batch" / "lv003_az+00_el-10_r10"
            make_batch(cfg, root / "batch")
            manifest_path = case / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["capture"]["capture_timestamp"] = "2026-08-22T00:00:00+08:00"
            manifest["capture"]["imu_or_platform_pose_reference"] = "imu.csv"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            (case / "capture.bin").write_bytes(b"synthetic-not-hardware")
            result = run(case, root / "analysis")
            self.assertTrue(result["ready_for_decode"])
            self.assertFalse(result["hardware_aoa_validated"])


if __name__ == "__main__":
    unittest.main()
