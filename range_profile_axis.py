"""Physical distance scale for the range-profile TLV of the active radar CFG."""

from functools import lru_cache
from math import isfinite


SPEED_OF_LIGHT_MPS = 299_792_458.0


@lru_cache(maxsize=16)
def range_profile_bin_size_m(config_content: str | None, profile_bins: int) -> float | None:
    """Return metres per FFT bin, or None if the active CFG cannot justify an axis.

    The range FFT is padded to the next power of two of the ADC sample count.
    A profile may contain the positive half or all of those bins, depending on
    the demo firmware. Its length is checked, but it is not a display range.
    """
    if not isinstance(config_content, str) or not isinstance(profile_bins, int):
        return None
    profiles = [line.split() for line in config_content.splitlines()
                if line.strip().startswith("profileCfg ")]
    if len(profiles) != 1 or len(profiles[0]) < 12:
        return None
    try:
        profile = profiles[0]
        slope_mhz_per_us = float(profile[8])
        adc_samples = int(profile[10])
        sample_rate_ksps = float(profile[11])
    except (ValueError, OverflowError):
        return None
    if (not 1 <= adc_samples <= 16384 or profile_bins <= 0
            or not isfinite(slope_mhz_per_us) or slope_mhz_per_us <= 0
            or not isfinite(sample_rate_ksps) or sample_rate_ksps <= 0):
        return None
    fft_bins = 1 << (adc_samples - 1).bit_length()
    if profile_bins not in (fft_bins // 2, fft_bins):
        return None
    return SPEED_OF_LIGHT_MPS * sample_rate_ksps * 1e3 / (
        2 * slope_mhz_per_us * 1e12 * fft_bins
    )
