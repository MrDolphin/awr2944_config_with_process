import tempfile
import unittest
from pathlib import Path


class DcaIqIntakeAuditTest(unittest.TestCase):
    def test_classifies_uart_and_size_compatible_binary_separately(self):
        from simulation.run_v04_110_dca_iq_intake_audit import classify

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            uart = root / "legacy.bin"
            uart.write_bytes(bytes.fromhex("02 01 04 03 06 05 08 07") + b"\0" * 100)
            raw = root / "capture.bin"
            raw.write_bytes(b"\0" * (656 * 4 * 4))
            self.assertEqual(classify(uart, adc_samples=656, rx_count=4, chirps_per_frame=240)["classification"], "uart_point_cloud_record")
            self.assertEqual(classify(raw, adc_samples=656, rx_count=4, chirps_per_frame=240)["classification"], "dca1000_raw_iq_candidate")


if __name__ == "__main__":
    unittest.main()
