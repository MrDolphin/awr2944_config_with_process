import unittest

from simulation.run_v04_49_calibration_doppler_overlap import DEFAULT_CALIBRATION, load_calibration


class CalibrationDopplerOverlapTests(unittest.TestCase):
    def test_default_calibration_is_explicitly_unmeasured(self):
        calibration = load_calibration(None)
        self.assertEqual(calibration["status"], "awaiting_dca1000_reference_capture")
        self.assertIsNone(calibration["reference_power_linear"])
        self.assertIn("raw_adc_iq_file", calibration["required_evidence"])


if __name__ == "__main__":
    unittest.main()
