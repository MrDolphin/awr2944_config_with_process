import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from simulation.dca1000_iq import encode_interleaved_iq
from simulation.run_v04_86_geometry_comparison import _load_candidate
from simulation.run_v04_90_known_angle_capture_contract import template
from simulation.run_v04_91_known_angle_capture import process


class KnownAngleCaptureTest(unittest.TestCase):
    def test_bin_capture_is_decoded_and_candidates_are_ranked(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            chirps, samples = 64, 32
            sample_index = np.arange(samples)[None, :, None]
            raw = np.tile((1000.0 * np.exp(2j * np.pi * 3 * sample_index / samples)), (chirps, 1, 4))
            (root / "capture.bin").write_bytes(encode_interleaved_iq(raw))
            (root / "profile.cfg").write_text("sensorStart\n", encoding="utf-8")
            candidate = Path("simulation/hardware/awr2944pev/v04_85_pcb_array_candidate/virtual_array_candidates.csv")
            manifest = template()
            manifest["capture_file"] = "capture.bin"
            manifest["target"]["range_m"] = 0.0
            manifest["capture"].update({"cfg_file": "profile.cfg", "chirps": chirps, "samples_per_chirp": samples, "rx_count": 4})
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            result = process(manifest_path, candidate, root / "output")
            self.assertEqual(result["status"], "completed_known_angle_capture_processing")
            self.assertEqual(result["decoded_iq_shape"], [chirps, samples, 4])
            self.assertEqual(len(result["results"]), 4)
            self.assertEqual(result["best_transform_by_known_angle"], "identity")


if __name__ == "__main__":
    unittest.main()
