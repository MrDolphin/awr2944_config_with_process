import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from simulation.dca1000_iq import encode_interleaved_iq
from simulation.run_v04_capture_ingest import run


class CaptureIngestTests(unittest.TestCase):
    def test_ingests_synthetic_bin_and_applies_optional_calibration(self):
        chirps, samples, rx = 8, 4, 4
        iq = np.ones((chirps, samples, rx), dtype=complex) * (10 + 2j)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw_path = root / "capture.bin"
            cfg_path = root / "capture.cfg"
            calibration_path = root / "calibration.json"
            output_path = root / "capture.h5"
            raw_path.write_bytes(encode_interleaved_iq(iq))
            cfg_path.write_text("sensorStart\n", encoding="utf-8")
            calibration_path.write_text(json.dumps({
                "amplitude": np.full((4, 4), 2.0).tolist(),
                "phase_deg": np.zeros((4, 4)).tolist(),
                "calibration_status": "synthetic",
            }), encoding="utf-8")
            summary = run(raw_path, cfg_path, output_path, chirps=chirps,
                          samples_per_chirp=samples, rx_count=rx,
                          tx_sequence=(0, 1, 2, 3), calibration_path=calibration_path)
            self.assertEqual(summary["decoded_shape"], [chirps, samples, rx])
            self.assertEqual(summary["virtual_shape"], [2, samples, rx, 4])
            self.assertFalse(summary["channel_order_verified"])
            self.assertTrue(summary["calibration"]["applied"])


if __name__ == "__main__":
    unittest.main()
