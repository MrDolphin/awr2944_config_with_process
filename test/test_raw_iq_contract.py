import tempfile
import unittest
from pathlib import Path
import h5py
import numpy as np

from simulation.run_v04_79_raw_iq_contract import validate


class RawIqContractTest(unittest.TestCase):
    def test_raw_iq_shape_and_metadata_boundary(self):
        with tempfile.TemporaryDirectory() as temp_name:
            path = Path(temp_name) / "capture.h5"
            with h5py.File(path, "w") as handle:
                handle.create_dataset("/radar/iq", data=np.zeros((8, 4, 4), dtype=np.complex64))
                handle.attrs["raw_dtype"] = "little_endian_int16"
                handle.attrs["wire_order_assumption"] = "sample -> rx -> I,Q"
                handle.attrs["channel_order_verified"] = False
            result = validate(path)
            self.assertEqual(result["kind"], "raw_adc_iq")
            self.assertEqual(result["status"], "valid_shape_metadata_incomplete")
            self.assertIn("missing_metadata", result)

    def test_virtual_iq_requires_four_by_four_trailing_shape(self):
        with tempfile.TemporaryDirectory() as temp_name:
            path = Path(temp_name) / "virtual.h5"
            with h5py.File(path, "w") as handle:
                handle.create_dataset("/recovered/virtual_iq", data=np.zeros((1, 4, 4, 3), dtype=np.complex64))
            result = validate(path)
            self.assertEqual(result["status"], "invalid_contract")
            self.assertTrue(result["issues"])


if __name__ == "__main__":
    unittest.main()
