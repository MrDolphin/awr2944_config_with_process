import json
import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_32_calibration_readiness import run


class CalibrationReadinessTest(unittest.TestCase):
    def test_synthetic_calibration_never_counts_as_measured_ready(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            calibration = root / "calibration.json"
            calibration.write_text(json.dumps({"amplitude": [[1.0] * 4] * 4, "phase_deg": [[0.0] * 4] * 4, "calibration_status": "synthetic_injected_error_not_measured"}), encoding="utf-8")
            cfg = root / "cfg.json"
            cfg.write_text("{}", encoding="utf-8")
            capture = root / "capture.h5"
            with h5py.File(capture, "w") as handle:
                handle.create_dataset("recovered/virtual_iq", data=np.zeros((1, 1, 4, 4), dtype=np.complex64))
                handle.attrs["channel_order_verified"] = False
            summary = run(capture, calibration, cfg, root / "output")
            self.assertFalse(summary["real_aoa_ready"])
            self.assertTrue(summary["checks"]["calibration_shape_4x4"])
            self.assertFalse(summary["checks"]["measured_calibration"])
            self.assertFalse(summary["checks"]["capture_channel_order_verified"])


if __name__ == "__main__":
    unittest.main()
