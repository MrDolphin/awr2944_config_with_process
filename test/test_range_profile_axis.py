import unittest

from range_profile_axis import range_profile_bin_size_m


class RangeProfileAxisTests(unittest.TestCase):
    CFG = "profileCfg 0 77 34 6 66 0 0 70 0 656 13349 0 0 30\n"

    def test_656_samples_and_512_profile_bins_match_recorded_target_ranges(self):
        bin_size = range_profile_bin_size_m(self.CFG, 512)
        self.assertIsNotNone(bin_size)
        self.assertAlmostEqual(38 * bin_size, 1.061, places=2)
        self.assertAlmostEqual(55 * bin_size, 1.535, places=2)
        self.assertAlmostEqual(73 * bin_size, 2.038, places=2)
        self.assertGreater(511 * bin_size, 14)

    def test_missing_or_inconsistent_cfg_does_not_invent_a_metres_axis(self):
        self.assertIsNone(range_profile_bin_size_m(None, 512))
        self.assertIsNone(range_profile_bin_size_m(self.CFG, 300))
        self.assertIsNone(range_profile_bin_size_m(self.CFG.replace("70 0 656", "nan 0 656"), 512))
        self.assertIsNone(range_profile_bin_size_m(
            self.CFG + "profileCfg 1 77 34 6 66 0 0 50 0 656 13349 0 0 30\n", 512
        ))


if __name__ == "__main__":
    unittest.main()
