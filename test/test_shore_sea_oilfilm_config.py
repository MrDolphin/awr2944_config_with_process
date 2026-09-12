import unittest
from pathlib import Path

from tools.analyze_adc_range import range_axis_m
from tools.dca1000_capture import parse_radar_cfg


class ShoreSeaOilFilmConfigTests(unittest.TestCase):
    def test_longchirp_candidate_preserves_400m_margin_and_improves_range_bins(self):
        cfg_path = Path("Config/shore_sea_400m_oilfilm_v1_longchirp.cfg")
        cfg = parse_radar_cfg(str(cfg_path))

        self.assertEqual(cfg["num_rx"], 4)
        self.assertEqual(cfg["num_adc_samples"], 1336)
        self.assertEqual(cfg["num_chirps_per_frame"], 64)
        self.assertEqual(cfg["frame_period_ms"], 100.0)
        self.assertEqual(cfg["sample_rate_ksps"], 13349.0)
        self.assertEqual(cfg["freq_slope_mhz_per_us"], 2.0)

        axis = range_axis_m(
            cfg["num_adc_samples"], cfg["sample_rate_ksps"], cfg["freq_slope_mhz_per_us"]
        )
        self.assertGreater(axis[-1], 400.0)
        self.assertAlmostEqual(axis[1] - axis[0], 0.75, delta=0.02)


if __name__ == "__main__":
    unittest.main()
