"""Static contract checks for the AWR2944P long-range raw-ADC candidate."""

from pathlib import Path
import unittest


class ShoreSea400mCfgTests(unittest.TestCase):
    def setUp(self):
        self.lines = [
            line.strip()
            for line in (
                Path(__file__).resolve().parents[1]
                / "Config"
                / "shore_sea_400m_v0_raw_adc.cfg"
            ).read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("%")
        ]

    def test_keeps_the_long_range_real_adc_waveform(self):
        self.assertIn("channelCfg 15 1 0 0 0", self.lines)
        self.assertIn("adcCfg 2 0", self.lines)
        self.assertIn("profileCfg 0 77 220 6 132 0 0 4 1 3072 25000 0 0 158", self.lines)
        self.assertIn("frameCfg 0 0 128 0 3072 500 1 0", self.lines)
        self.assertIn("lvdsStreamCfg -1 0 1 0", self.lines)
        self.assertIn("cfarFovCfg -1 1 -2.77 2.77", self.lines)

    def test_has_two_lane_lvds_chirp_transport_margin(self):
        """Keep the profile inside the TI demo's per-chirp LVDS timing rule.

        With 3072 ADC samples, four RX channels and the demo's documented
        complex-sample sizing, one chirp needs 49,408 bytes after 256-byte
        alignment.  At the conservative two-lane / 600-Mbps setting, the
        selected 352-us chirp period can transport 52,800 bytes.
        """
        required_bytes = ((3072 * 4 * 4 + 52 + 255) // 256) * 256
        available_bytes = 352 * 2 * 600 // 8
        self.assertGreaterEqual(available_bytes, required_bytes)

    def test_v1_is_a_single_variable_2048_sample_ab_profile(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "Config"
            / "shore_sea_400m_v1_raw_adc_2048.cfg"
        )
        lines = {
            line.strip()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("%")
        }
        self.assertIn("channelCfg 15 1 0 0 0", lines)
        self.assertIn("adcCfg 2 0", lines)
        self.assertIn("profileCfg 0 77 220 6 132 0 0 4 1 2048 25000 0 0 158", lines)
        self.assertIn("frameCfg 0 0 128 0 2048 500 1 0", lines)
        self.assertIn("lvdsStreamCfg -1 0 1 0", lines)
        self.assertIn("sensorStart", lines)

        required_bytes = ((2048 * 4 * 4 + 52 + 255) // 256) * 256
        available_bytes = 352 * 2 * 600 // 8
        self.assertEqual(required_bytes, 33024)
        self.assertGreaterEqual(available_bytes, required_bytes)

    def test_is_a_complete_mmw_demo_configuration_before_sensor_start(self):
        required_prefixes = (
            "dfeDataOutputMode ",
            "channelCfg ",
            "adcCfg ",
            "adcbufCfg ",
            "profileCfg ",
            "chirpCfg ",
            "frameCfg ",
            "guiMonitor ",
            "lvdsStreamCfg ",
            "cfarCfg ",
            "multiObjBeamForming ",
            "calibDcRangeSig ",
            "clutterRemoval ",
            "antGeometryCfg ",
            "compRangeBiasAndRxChanPhase ",
            "measureRangeBiasAndRxChanPhase ",
            "aoaFovCfg ",
            "cfarFovCfg ",
            "extendedMaxVelocity ",
            "calibData ",
        )
        sensor_start = self.lines.index("sensorStart")
        configured_lines = self.lines[:sensor_start]
        for prefix in required_prefixes:
            self.assertTrue(
                any(line.startswith(prefix) for line in configured_lines),
                msg=f"missing required pre-start command: {prefix}",
            )


if __name__ == "__main__":
    unittest.main()
