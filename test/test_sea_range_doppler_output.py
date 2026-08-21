import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np


class SeaRangeDopplerOutputTests(unittest.TestCase):
    def test_existing_v043_output_has_expected_datasets(self):
        path = Path("simulation/hardware/awr2944pev/v04_43_sea_range_doppler/ss2_normal_range_doppler.h5")
        self.assertTrue(path.exists())
        with h5py.File(path, "r") as handle:
            self.assertEqual(handle["/range_doppler/power_linear"].ndim, 3)
            self.assertEqual(handle["/range_doppler/power_linear"].shape[1:], (64, 64))
            self.assertEqual(handle["/peaks/range_m"].shape[0], 41)
            self.assertFalse(bool(handle.attrs["channel_order_verified"]))


if __name__ == "__main__":
    unittest.main()
