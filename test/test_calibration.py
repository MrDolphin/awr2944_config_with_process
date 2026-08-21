import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from simulation.calibration import apply_channel_correction, complex_matrix, load_calibration


class CalibrationTests(unittest.TestCase):
    def test_inverse_correction_recovers_channel(self):
        error = complex_matrix(np.full((4, 4), 1.1), np.full((4, 4), 12.0))
        raw = np.ones((4, 4), dtype=complex) * error
        corrected = apply_channel_correction(raw, 1.0 / error)
        np.testing.assert_allclose(corrected, np.ones((4, 4)), atol=1e-12)

    def test_json_loader_preserves_metadata(self):
        payload = {"amplitude": np.ones((4, 4)).tolist(), "phase_deg": np.zeros((4, 4)).tolist(),
                   "calibration_status": "synthetic"}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            correction, metadata = load_calibration(path)
        np.testing.assert_allclose(correction, np.ones((4, 4)))
        self.assertEqual(metadata["calibration_status"], "synthetic")


if __name__ == "__main__":
    unittest.main()
