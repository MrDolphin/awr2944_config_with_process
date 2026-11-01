"""Conservative TLV2 distance-axis arithmetic from a *recorded* AWR2944 CFG.

This module never configures hardware. Its numbers assume the TI OOB real-ADC
range FFT: next-power-of-two FFT length and a positive-frequency TLV2 half.
They describe nominal bin locations, not target detection or calibration.
TI CLI field reference: https://dr-download.ti.com/software-development/
software-development-kit-sdk/MD-U4MY7aGNn5/04.06.00.01/
mmwave_mcuplus_sdk_user_guide.pdf
"""

from __future__ import annotations

import hashlib
import math


SPEED_OF_LIGHT_MPS = 299_792_458


def _fallback(reason: str, bin_count: int, config_sha256: str | None = None) -> dict:
    return {
        "range_axis": "bin_index_uncalibrated",
        "reason": reason,
        "config_sha256": config_sha256,
        "tlv2_bin_count": bin_count,
        "fft_size": None,
        "range_bin_size_m": None,
        "nominal_unambiguous_range_m": None,
        "last_tlv2_bin_m": None,
        "cfar_range_fov_m": None,
        "frame_period_ms": None,
        "interpretation": "Raw TLV2 bin index only; no distance conversion or detection claim.",
    }


def _one(commands: dict[str, list[list[str]]], name: str) -> list[str]:
    rows = commands.get(name, [])
    if len(rows) != 1:
        raise ValueError(f"{name}: expected exactly one command, found {len(rows)}")
    return rows[0]


def _positive_float(value: str, field: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f"{field}: expected positive finite value")
    return number


def diagnose_capture_range_axis(metadata: dict, tlv2_bin_count: int) -> dict:
    """Return a theoretical axis, or preserve raw bin indices if evidence is weak.

    ``metadata`` is the captured ``metadata.json`` object, not a current CFG
    file that may have changed since acquisition. No files are read or written.
    """
    if not isinstance(tlv2_bin_count, int) or isinstance(tlv2_bin_count, bool) or tlv2_bin_count <= 0:
        return _fallback("TLV2 bin count is missing or invalid", tlv2_bin_count)
    config = metadata.get("active_config") if isinstance(metadata, dict) else None
    if not isinstance(config, dict) or not isinstance(config.get("content"), str) or not config["content"].strip():
        return _fallback("missing active_config.content in capture metadata", tlv2_bin_count)

    content = config["content"]
    digest = hashlib.sha256(content.encode("utf-8")).hexdigest()
    if config.get("sha256") is not None and config["sha256"] != digest:
        return _fallback("active_config SHA-256 does not match content", tlv2_bin_count, digest)

    commands: dict[str, list[list[str]]] = {}
    for raw_line in content.splitlines():
        tokens = raw_line.split("%", 1)[0].split("#", 1)[0].split()
        if tokens:
            commands.setdefault(tokens[0], []).append(tokens)

    try:
        adc = _one(commands, "adcCfg")
        profile = _one(commands, "profileCfg")
        frame = _one(commands, "frameCfg")
        if len(adc) != 3 or adc[2] != "0":
            raise ValueError("adcCfg: only real ADC output can use this conversion")
        if len(profile) < 12 or len(frame) != 9:
            raise ValueError("profileCfg/frameCfg: unsupported or incomplete CLI layout")
        if commands.get("dfeDataOutputMode") not in (None, [["dfeDataOutputMode", "1"]]):
            raise ValueError("dfeDataOutputMode: only legacy frame mode is supported")

        adc_samples = int(profile[10])
        frame_adc_samples = int(frame[5])
        if adc_samples <= 0 or adc_samples > 1_000_000:
            raise ValueError("profileCfg: invalid ADC sample count")
        if frame_adc_samples != adc_samples:
            raise ValueError("frameCfg: ADC sample count differs from profileCfg")
        chirp_start, chirp_end = int(frame[1]), int(frame[2])
        if chirp_start < 0 or chirp_end > 511 or chirp_start > chirp_end or int(frame[3]) <= 0:
            raise ValueError("frameCfg: invalid chirp indices or loop count")
        frame_chirps = set(range(chirp_start, chirp_end + 1))
        uncovered = frame_chirps.copy()
        profile_id = int(profile[1])
        for chirp in commands.get("chirpCfg", []):
            if len(chirp) < 4:
                raise ValueError("chirpCfg: incomplete command")
            start, end, chirp_profile = map(int, chirp[1:4])
            if start > end:
                raise ValueError("chirpCfg: invalid chirp indices")
            active = frame_chirps.intersection(range(start, end + 1))
            if active and chirp_profile != profile_id:
                raise ValueError("chirpCfg: active chirp references a different profileCfg")
            uncovered.difference_update(active)
        if uncovered:
            raise ValueError("chirpCfg: frame chirp range is not fully defined")

        slope = _positive_float(profile[8], "profileCfg slope") * 1e12
        sample_rate = _positive_float(profile[11], "profileCfg sample rate") * 1e3
        if not all(math.isfinite(value) and value > 0 for value in (slope, sample_rate)):
            raise ValueError("profileCfg: converted slope or sample rate is not finite")
        frame_period = _positive_float(frame[6], "frameCfg period")
        fft_size = 1 << (adc_samples - 1).bit_length()
        if tlv2_bin_count != fft_size // 2:
            raise ValueError("TLV2 bin count differs from real-ADC FFT positive half")

        for row in commands.get("cfarFovCfg", []):
            if len(row) != 5 or row[1] not in ("-1", "0") or row[2] not in ("0", "1"):
                raise ValueError("cfarFovCfg: unsupported or incomplete command")
        range_fov = [row for row in commands.get("cfarFovCfg", []) if row[2] == "0"]
        if len(range_fov) > 1:
            raise ValueError("cfarFovCfg: conflicting range filters")
        cfar_range = None
        if range_fov:
            row = range_fov[0]
            if len(row) != 5 or row[1] not in ("-1", "0"):
                raise ValueError("cfarFovCfg: unsupported range-filter layout")
            low, high = float(row[3]), float(row[4])
            if not all(map(math.isfinite, (low, high))) or low < 0 or high <= low:
                raise ValueError("cfarFovCfg: invalid range filter")
            cfar_range = [low, high]
    except (ValueError, OverflowError, IndexError) as exc:
        return _fallback(str(exc), tlv2_bin_count, digest)

    step = SPEED_OF_LIGHT_MPS * sample_rate / (2 * slope * fft_size)
    if not math.isfinite(step) or step <= 0 or not math.isfinite(step * tlv2_bin_count):
        return _fallback("profileCfg: theoretical range axis is not finite", tlv2_bin_count, digest)
    return {
        "range_axis": "metres_theoretical_uncalibrated",
        "reason": None,
        "config_sha256": digest,
        "tlv2_bin_count": tlv2_bin_count,
        "fft_size": fft_size,
        "adc_samples": adc_samples,
        "adc_output_format": "real",
        "range_bin_size_m": step,
        "nominal_unambiguous_range_m": step * tlv2_bin_count,
        "last_tlv2_bin_m": step * (tlv2_bin_count - 1),
        "cfar_range_fov_m": cfar_range,
        "frame_period_ms": frame_period,
        "interpretation": (
            "Theoretical real-ADC FFT conversion from captured CFG; not a detection "
            "or field-validated range claim. cfarFovCfg filters output points only."
        ),
    }
