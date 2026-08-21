import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from simulation.dca1000_iq import encode_interleaved_iq
from simulation.run_v04_90_known_angle_capture_contract import template
from simulation.run_v04_92_known_angle_batch import run


class KnownAngleBatchTest(unittest.TestCase):
    def test_batch_processes_multiple_manifests_and_aggregates(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            for index in range(2):
                capture_dir = root / f"capture_{index:02d}"
                capture_dir.mkdir()
                chirps, samples = 64, 32
                n = np.arange(samples)[None, :, None]
                raw = np.tile(1000.0 * np.exp(2j * np.pi * 3 * n / samples), (chirps, 1, 4))
                (capture_dir / "capture.bin").write_bytes(encode_interleaved_iq(raw))
                (capture_dir / "profile.cfg").write_text("sensorStart\n", encoding="utf-8")
                manifest = template()
                manifest["capture_file"] = "capture.bin"
                manifest["capture"].update({"cfg_file": "profile.cfg", "chirps": chirps, "samples_per_chirp": samples, "rx_count": 4})
                manifest["target"]["range_m"] = 0.0
                (capture_dir / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            candidate = Path("simulation/hardware/awr2944pev/v04_85_pcb_array_candidate/virtual_array_candidates.csv")
            summary = run(root, candidate, root / "results")
            self.assertEqual(summary["status"], "completed_known_angle_batch")
            self.assertEqual(summary["processed_capture_count"], 2)
            self.assertEqual(summary["failure_count"], 0)
            self.assertEqual(summary["best_transform_by_batch_rmse"], "identity")
            self.assertTrue((root / "results" / "transform_aggregate.csv").is_file())


if __name__ == "__main__":
    unittest.main()
