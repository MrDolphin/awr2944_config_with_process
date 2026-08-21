import tempfile
import unittest
from pathlib import Path

from simulation.parse_cfg_calibration import parse


class CfgCalibrationTests(unittest.TestCase):
    def test_tx_major_pairs_become_rx_by_tx_matrix(self):
        pairs = " ".join(f"{i + 1} 0" for i in range(16))
        text = f"compRangeBiasAndRxChanPhase 0.25 {pairs}\n"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.cfg"
            path.write_text(text, encoding="utf-8")
            payload = parse(path)
        self.assertEqual(payload["range_bias_m"], 0.25)
        self.assertEqual(payload["amplitude"][0][0], 1.0)
        self.assertEqual(payload["amplitude"][0][1], 5.0)
        self.assertEqual(payload["amplitude"][3][3], 16.0)
        self.assertEqual(payload["calibration_status"], "cfg_values_present_measurement_provenance_required")


if __name__ == "__main__":
    unittest.main()
