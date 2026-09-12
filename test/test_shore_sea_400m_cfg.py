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

    def test_v2_changes_only_to_known_good_four_tx_tdm_topology(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "Config"
            / "shore_sea_400m_v2_raw_adc_2048_4tx.cfg"
        )
        lines = {
            line.strip()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("%")
        }
        self.assertIn("channelCfg 15 15 0 0 0", lines)
        self.assertIn("profileCfg 0 77 220 6 132 0 0 4 1 2048 25000 0 0 158", lines)
        self.assertIn("frameCfg 0 3 32 0 2048 500 1 0", lines)
        self.assertIn("lvdsStreamCfg -1 0 1 0", lines)
        self.assertTrue({
            "chirpCfg 0 0 0 0 0 0 0 1",
            "chirpCfg 1 1 0 0 0 0 0 4",
            "chirpCfg 2 2 0 0 0 0 0 8",
            "chirpCfg 3 3 0 0 0 0 0 2",
        }.issubset(lines))

        required_bytes = ((2048 * 4 * 4 + 52 + 255) // 256) * 256
        available_bytes = 352 * 2 * 600 // 8
        self.assertEqual(required_bytes, 33024)
        self.assertGreaterEqual(available_bytes, required_bytes)

    def test_v3_reduces_only_total_chirps_to_the_known_good_baseline_count(self):
        """V3 keeps V2's long-range waveform but cuts 128 chirps to 64.

        The V2 start attempt had no LVDS data even after a 12-second serial
        observation.  This A/B check reduces radar-cube/slow-time pressure
        while holding the 2048 real samples, 4-TX topology, slope and sample
        rate fixed.  Four chirp types multiplied by 16 loops gives 64 total
        chirps, matching the known-good near-range baseline's frame count.
        """
        path = (
            Path(__file__).resolve().parents[1]
            / "Config"
            / "shore_sea_400m_v3_raw_adc_2048_4tx_64chirps.cfg"
        )
        lines = {
            line.strip()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("%")
        }
        self.assertIn("channelCfg 15 15 0 0 0", lines)
        self.assertIn("profileCfg 0 77 220 6 132 0 0 4 1 2048 25000 0 0 158", lines)
        self.assertIn("frameCfg 0 3 16 0 2048 500 1 0", lines)
        self.assertIn("lvdsStreamCfg -1 0 1 0", lines)
        self.assertIn("sensorStart", lines)

        chirps_per_frame = 4 * 16
        tx_count = 4
        self.assertEqual(chirps_per_frame, 64)
        self.assertEqual(chirps_per_frame % tx_count, 0)
        self.assertEqual((chirps_per_frame // tx_count) % 2, 0)

    def test_200m_v0_retains_the_known_good_baseline_data_path(self):
        """The 200-m candidate changes only slope and range FOV from test_full."""
        path = (
            Path(__file__).resolve().parents[1]
            / "Config"
            / "shore_sea_200m_v0_from_test_full.cfg"
        )
        lines = {
            line.strip()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("%")
        }
        self.assertIn("channelCfg 15 15 0 0 0", lines)
        self.assertIn("adcCfg 2 0", lines)
        self.assertIn("profileCfg 0 77 186 7 57.14 0 0 5 1 656 13349 0 0 158", lines)
        self.assertIn("frameCfg 0 3 16 0 656 100 1 0", lines)
        self.assertIn("guiMonitor -1 2 1 0 0 0 1", lines)
        self.assertIn("lvdsStreamCfg -1 0 1 0", lines)
        self.assertIn("cfarFovCfg -1 0 0 180", lines)
        self.assertIn("cfarFovCfg -1 1 -1 1.00", lines)

        # Real-only ADC: Rmax = c*Fs/(4*slope).  180 m retains margin below
        # the ~200.2 m theoretical Nyquist-limited limit of this waveform.
        sample_rate_hz = 13_349_000
        slope_hz_per_s = 5_000_000_000_000
        theoretical_limit_m = 299_792_458 * sample_rate_hz / (4 * slope_hz_per_s)
        self.assertGreater(theoretical_limit_m, 200)
        self.assertLess(180, theoretical_limit_m)

    def test_300m_v0_retains_the_200m_baseline_except_for_slope_and_range_fov(self):
        """The 300-m class profile preserves the now-proven 200-m data path."""
        path = (
            Path(__file__).resolve().parents[1]
            / "Config"
            / "shore_sea_300m_v0_from_test_full.cfg"
        )
        lines = {
            line.strip()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("%")
        }
        self.assertIn("channelCfg 15 15 0 0 0", lines)
        self.assertIn("adcCfg 2 0", lines)
        self.assertIn("profileCfg 0 77 186 7 57.14 0 0 3 1 656 13349 0 0 158", lines)
        self.assertIn("frameCfg 0 3 16 0 656 100 1 0", lines)
        self.assertIn("guiMonitor -1 2 1 0 0 0 1", lines)
        self.assertIn("lvdsStreamCfg -1 0 1 0", lines)
        self.assertIn("cfarFovCfg -1 0 0 300", lines)
        self.assertIn("cfarFovCfg -1 1 -1 1.00", lines)

        # At 300 m, the real-ADC beat frequency is 6 MHz, below the
        # 6.6745-MHz Nyquist frequency of this 13.349-MSps baseline path.
        sample_rate_hz = 13_349_000
        slope_hz_per_s = 3_000_000_000_000
        theoretical_limit_m = 299_792_458 * sample_rate_hz / (4 * slope_hz_per_s)
        self.assertGreater(theoretical_limit_m, 300)

    def test_350m_v0_retains_the_300m_baseline_with_frequency_margin(self):
        """The 350-m candidate changes only slope and range FOV from 300-m V0."""
        path = (
            Path(__file__).resolve().parents[1]
            / "Config"
            / "shore_sea_350m_v0_from_test_full.cfg"
        )
        lines = {
            line.strip()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("%")
        }
        self.assertIn("channelCfg 15 15 0 0 0", lines)
        self.assertIn("adcCfg 2 0", lines)
        self.assertIn("profileCfg 0 77 186 7 57.14 0 0 2.5 1 656 13349 0 0 158", lines)
        self.assertIn("frameCfg 0 3 16 0 656 100 1 0", lines)
        self.assertIn("guiMonitor -1 2 1 0 0 0 1", lines)
        self.assertIn("lvdsStreamCfg -1 0 1 0", lines)
        self.assertIn("cfarFovCfg -1 0 0 350", lines)
        self.assertIn("cfarFovCfg -1 1 -1 1.00", lines)

        # The 2.5-MHz/us slope gives roughly 400 m real-ADC headroom,
        # preserving margin above the requested 350-m processing region.
        sample_rate_hz = 13_349_000
        slope_hz_per_s = 2_500_000_000_000
        theoretical_limit_m = 299_792_458 * sample_rate_hz / (4 * slope_hz_per_s)
        self.assertGreater(theoretical_limit_m, 400)
        self.assertLess(350, theoretical_limit_m)

    def test_v4_halves_only_the_v0_slope_for_a_bandwidth_ab(self):
        """V4 isolates sweep-bandwidth/slope from the V0 sample-load factors."""
        path = (
            Path(__file__).resolve().parents[1]
            / "Config"
            / "shore_sea_400m_v4_half_bandwidth_raw_adc.cfg"
        )
        lines = {
            line.strip()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("%")
        }
        self.assertIn("channelCfg 15 1 0 0 0", lines)
        self.assertIn("adcCfg 2 0", lines)
        self.assertIn("profileCfg 0 77 220 6 132 0 0 2 1 3072 25000 0 0 158", lines)
        self.assertIn("frameCfg 0 0 128 0 3072 500 1 0", lines)
        self.assertIn("lvdsStreamCfg -1 0 1 0", lines)
        self.assertIn("cfarFovCfg -1 0 0 450", lines)

        # Holding sample rate and acquisition time constant halves FMCW
        # bandwidth, doubles Rmax, and doubles range-bin spacing versus V0.
        sample_rate_hz = 25_000_000
        slope_hz_per_s = 2_000_000_000_000
        theoretical_limit_m = 299_792_458 * sample_rate_hz / (4 * slope_hz_per_s)
        self.assertGreater(theoretical_limit_m, 900)
        self.assertLess(400, theoretical_limit_m)

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
