import tempfile
import unittest
from pathlib import Path

import numpy as np

from simulation.dca1000_iq import encode_interleaved_iq
from simulation.run_v04_50_dca_reference_calibration import process_capture


class DcaReferenceCalibrationTests(unittest.TestCase):
    def test_known_capture_produces_reference_power_and_ratio(self):
        chirps, samples, rx = 16, 64, 4
        iq = np.zeros((chirps, samples, rx), dtype=complex)
        iq[:, 8, :] = 100.0 + 25.0j
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); raw = root / "reference.bin"; output = root / "reference.h5"
            raw.write_bytes(encode_interleaved_iq(iq))
            result = process_capture(raw, output, chirps=chirps, samples_per_chirp=samples, rx_count=rx, expected_range_m=8 * (25e6 / 64 * 299792458.0 / (2 * (1e9 / 60e-6))) / 25e6, expected_velocity_mps=0.0, tx_sequence=(0,), synthetic_reference_power_linear=1.0)
            self.assertTrue(output.is_file())
            self.assertGreater(result["reference_power_linear"], 0.0)
            self.assertGreater(result["synthetic_to_measured_power_ratio"], 0.0)
            self.assertFalse(result["channel_order_verified"])


if __name__ == "__main__":
    unittest.main()
