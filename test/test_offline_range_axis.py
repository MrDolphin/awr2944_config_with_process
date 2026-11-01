"""Pure, offline checks for a capture's *theoretical* TLV2 distance axis."""

import hashlib
import unittest

from tools.offline_range_axis import diagnose_capture_range_axis


CFG = "\n".join((
    "dfeDataOutputMode 1",
    "adcCfg 2 0",
    "profileCfg 0 77 186 7 57.14 0 0 70 1 656 13349 0 0 158",
    "chirpCfg 0 0 0 0 0 0 0 1",
    "chirpCfg 1 1 0 0 0 0 0 4",
    "chirpCfg 2 2 0 0 0 0 0 8",
    "chirpCfg 3 3 0 0 0 0 0 2",
    "frameCfg 0 3 16 0 656 100 1 0",
    "cfarFovCfg -1 0 0 3.5",
    "cfarFovCfg -1 1 -1 1.00",
))


def metadata(content=CFG, *, sha=None):
    return {"active_config": {"content": content, "sha256": sha or
            hashlib.sha256(content.encode("utf-8")).hexdigest()}}


class OfflineRangeAxisTests(unittest.TestCase):
    def test_real_adc_cfg_produces_theoretical_axis_not_detection_claim(self):
        result = diagnose_capture_range_axis(metadata(), 512)
        step = 299_792_458 * 13_349_000 / (2 * 70e12 * 1024)
        self.assertEqual(result["range_axis"], "metres_theoretical_uncalibrated")
        self.assertEqual(result["fft_size"], 1024)
        self.assertAlmostEqual(result["range_bin_size_m"], step)
        self.assertAlmostEqual(result["nominal_unambiguous_range_m"], step * 512)
        self.assertAlmostEqual(result["last_tlv2_bin_m"], step * 511)
        self.assertEqual(result["cfar_range_fov_m"], [0.0, 3.5])
        self.assertEqual(result["frame_period_ms"], 100.0)
        self.assertEqual(result["config_sha256"], metadata()["active_config"]["sha256"])
        self.assertIn("not a detection", result["interpretation"].lower())

    def test_missing_cfg_keeps_raw_bin_index(self):
        result = diagnose_capture_range_axis({}, 512)
        self.assertEqual(result["range_axis"], "bin_index_uncalibrated")
        self.assertIsNone(result["range_bin_size_m"])
        self.assertIn("missing", result["reason"].lower())

    def test_conflicting_profile_cfg_keeps_raw_bin_index(self):
        second = "profileCfg 1 77 186 7 57.14 0 0 35 1 656 13349 0 0 158"
        result = diagnose_capture_range_axis(metadata(CFG + "\n" + second), 512)
        self.assertEqual(result["range_axis"], "bin_index_uncalibrated")
        self.assertIn("profileCfg", result["reason"])

    def test_inconsistent_tlv2_length_keeps_raw_bin_index(self):
        result = diagnose_capture_range_axis(metadata(), 256)
        self.assertEqual(result["range_axis"], "bin_index_uncalibrated")
        self.assertIn("TLV2", result["reason"])

    def test_frame_cfg_adc_samples_must_match_profile(self):
        result = diagnose_capture_range_axis(metadata(CFG.replace("0 656 100 1 0", "0 512 100 1 0")), 512)
        self.assertEqual(result["range_axis"], "bin_index_uncalibrated")
        self.assertIn("frameCfg", result["reason"])

    def test_missing_or_wrong_active_chirp_profile_keeps_bin_index(self):
        first_chirp = "chirpCfg 0 0 0 0 0 0 0 1"
        for altered in (CFG.replace(first_chirp, ""),
                        CFG.replace(first_chirp, "chirpCfg 0 0 1 0 0 0 0 1")):
            with self.subTest(altered=altered):
                result = diagnose_capture_range_axis(metadata(altered), 512)
                self.assertEqual(result["range_axis"], "bin_index_uncalibrated")
                self.assertIn("chirpCfg", result["reason"])

    def test_malformed_cfar_fov_does_not_silently_disappear(self):
        result = diagnose_capture_range_axis(metadata(CFG.replace("cfarFovCfg -1 0 0 3.5", "cfarFovCfg -1")), 512)
        self.assertEqual(result["range_axis"], "bin_index_uncalibrated")
        self.assertIn("cfarFovCfg", result["reason"])

    def test_overlapping_chirp_with_other_profile_is_conflict(self):
        result = diagnose_capture_range_axis(metadata(CFG + "\nchirpCfg 0 0 1 0 0 0 0 1"), 512)
        self.assertEqual(result["range_axis"], "bin_index_uncalibrated")
        self.assertIn("chirpCfg", result["reason"])

    def test_invalid_or_tampered_cfg_keeps_raw_bin_index(self):
        for sample in (metadata(CFG.replace(" 70 1 656", " 0 1 656")),
                       metadata(CFG, sha="0" * 64)):
            with self.subTest(sample=sample):
                result = diagnose_capture_range_axis(sample, 512)
                self.assertEqual(result["range_axis"], "bin_index_uncalibrated")
                self.assertIsNone(result["range_bin_size_m"])

    def test_complex_adc_is_not_silently_treated_as_real(self):
        result = diagnose_capture_range_axis(metadata(CFG.replace("adcCfg 2 0", "adcCfg 2 1")), 512)
        self.assertEqual(result["range_axis"], "bin_index_uncalibrated")
        self.assertIn("adcCfg", result["reason"])

    def test_extreme_numeric_cfg_never_emits_nonfinite_metres(self):
        result = diagnose_capture_range_axis(metadata(CFG.replace(" 656 13349 ", " 656 1e308 ")), 512)
        self.assertEqual(result["range_axis"], "bin_index_uncalibrated")
        self.assertIsNone(result["range_bin_size_m"])


if __name__ == "__main__":
    unittest.main()
