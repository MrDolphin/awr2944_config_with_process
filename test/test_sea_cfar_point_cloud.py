import unittest
from pathlib import Path

import h5py


class SeaCfarPointCloudTests(unittest.TestCase):
    def test_v043_complex_spectrum_is_available_for_cfar_aoa(self):
        path = Path("simulation/hardware/awr2944pev/v04_43_sea_range_doppler/ss2_normal_range_doppler.h5")
        with h5py.File(path, "r") as handle:
            self.assertEqual(handle["/range_doppler/spectrum_complex"].shape, (41, 64, 64, 4, 4))
            self.assertEqual(handle["/range_doppler/power_linear"].shape, (41, 64, 64))


if __name__ == "__main__":
    unittest.main()
