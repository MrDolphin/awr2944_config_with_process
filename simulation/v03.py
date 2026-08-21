"""Minimal V0.3 single-scatterer FMCW IQ and Range-Doppler processing."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class FmcwConfig:
    carrier_frequency_hz: float = 77.0e9
    sweep_bandwidth_hz: float = 1.0e9
    chirp_duration_s: float = 60.0e-6
    sample_rate_hz: float = 25.0e6
    samples_per_chirp: int = 256
    chirps_per_frame: int = 64
    pulse_repetition_interval_s: float = 100.0e-6
    tx_count: int = 4
    rx_count: int = 4
    propagation_speed_mps: float = 299_792_458.0

    @property
    def wavelength_m(self) -> float:
        return self.propagation_speed_mps / self.carrier_frequency_hz

    @property
    def slope_hz_per_s(self) -> float:
        return self.sweep_bandwidth_hz / self.chirp_duration_s

    @property
    def range_resolution_m(self) -> float:
        return self.propagation_speed_mps / (2.0 * self.sweep_bandwidth_hz)

    @property
    def max_range_m(self) -> float:
        return self.propagation_speed_mps * self.sample_rate_hz / (
            4.0 * self.slope_hz_per_s
        )

    @property
    def velocity_resolution_mps(self) -> float:
        return self.wavelength_m / (
            2.0 * self.chirps_per_frame * self.pulse_repetition_interval_s
        )

    @property
    def max_unambiguous_velocity_mps(self) -> float:
        return self.wavelength_m / (
            4.0 * self.pulse_repetition_interval_s
        )

    def validate(self) -> None:
        if self.samples_per_chirp <= 0 or self.chirps_per_frame <= 0:
            raise ValueError("sample and chirp counts must be positive")
        if self.tx_count != 4 or self.rx_count != 4:
            raise ValueError("V0.3 minimal contract requires 4 TX and 4 RX")
        if self.sample_rate_hz <= 2.0 * self.slope_hz_per_s * self.max_range_m / self.propagation_speed_mps:
            raise ValueError("sample rate is incompatible with configured range")


@dataclass(frozen=True)
class RangeDopplerResult:
    range_axis_m: np.ndarray
    velocity_axis_mps: np.ndarray
    power_linear: np.ndarray
    peak_range_m: float
    peak_velocity_mps: float
    peak_power_linear: float


def compute_controlled_scatterer_weights(
    slant_range_m: np.ndarray,
    grazing_angle_deg: np.ndarray,
    *,
    front_face_only: bool = True,
) -> np.ndarray:
    """Return a transparent, non-calibrated sea-facet amplitude proxy.

    The model applies amplitude proportional to ``1/R^2`` and optionally
    rejects locally back-facing facets (grazing angle <= 0).  It is a
    geometry regression aid, not a sea electromagnetic reflectivity model and
    does not perform global ray occlusion.
    """
    ranges = np.asarray(slant_range_m, dtype=float)
    grazing = np.asarray(grazing_angle_deg, dtype=float)
    if ranges.shape != grazing.shape:
        raise ValueError("range and grazing-angle arrays must have the same shape")
    if np.any(~np.isfinite(ranges)) or np.any(ranges <= 0.0):
        raise ValueError("slant ranges must be finite and positive")
    weights = 1.0 / np.square(ranges)
    if front_face_only:
        weights = np.where(grazing > 0.0, weights, 0.0)
    peak = float(np.max(weights))
    return weights / peak if peak > 0.0 else weights


def generate_single_scatterer_iq(
    config: FmcwConfig,
    *,
    slant_range_m: float,
    radial_velocity_mps: float,
    amplitude: float = 1.0,
) -> np.ndarray:
    """Generate coherent baseband beat IQ with shape (chirp, sample, rx, tx)."""

    config.validate()
    if slant_range_m < 0.0 or slant_range_m >= config.max_range_m:
        raise ValueError("slant range is outside the configured unambiguous range")
    fast_time = np.arange(config.samples_per_chirp) / config.sample_rate_hz
    slow_time = np.arange(config.chirps_per_frame) * config.pulse_repetition_interval_s
    beat_hz = 2.0 * config.slope_hz_per_s * slant_range_m / config.propagation_speed_mps
    doppler_hz = 2.0 * radial_velocity_mps / config.wavelength_m
    phase = 2.0 * np.pi * (
        doppler_hz * slow_time[:, None] + beat_hz * fast_time[None, :]
    )
    baseband = amplitude * np.exp(1j * phase)
    return np.broadcast_to(
        baseband[:, :, None, None],
        (
            config.chirps_per_frame,
            config.samples_per_chirp,
            config.rx_count,
            config.tx_count,
        ),
    ).copy()


def generate_multi_scatterer_iq(
    config: FmcwConfig,
    scatterers: list[tuple[float, float, float]],
) -> np.ndarray:
    """Sum coherent IQ for (slant_range_m, radial_velocity_mps, amplitude)."""

    if not scatterers:
        raise ValueError("at least one scatterer is required")
    total = np.zeros(
        (
            config.chirps_per_frame,
            config.samples_per_chirp,
            config.rx_count,
            config.tx_count,
        ),
        dtype=complex,
    )
    for slant_range_m, radial_velocity_mps, amplitude in scatterers:
        total += generate_single_scatterer_iq(
            config,
            slant_range_m=slant_range_m,
            radial_velocity_mps=radial_velocity_mps,
            amplitude=amplitude,
        )
    return total


def process_range_doppler(
    iq: np.ndarray, config: FmcwConfig
) -> RangeDopplerResult:
    """Apply separable Hann-window FFTs and return the strongest bin."""

    config.validate()
    expected = (
        config.chirps_per_frame,
        config.samples_per_chirp,
        config.rx_count,
        config.tx_count,
    )
    if iq.shape != expected:
        raise ValueError(f"IQ shape must be {expected}, got {iq.shape}")
    reference = np.mean(iq, axis=(2, 3))
    range_window = np.hanning(config.samples_per_chirp)
    doppler_window = np.hanning(config.chirps_per_frame)
    spectrum = np.fft.fft(reference * range_window[None, :], axis=1)
    spectrum = np.fft.fftshift(
        np.fft.fft(spectrum * doppler_window[:, None], axis=0), axes=0
    )
    power = np.abs(spectrum) ** 2
    positive_range = np.arange(config.samples_per_chirp // 2)
    power = power[:, positive_range]
    peak_doppler_index, peak_range_index = np.unravel_index(
        int(np.argmax(power)), power.shape
    )
    range_axis = (
        positive_range
        * config.sample_rate_hz
        / config.samples_per_chirp
        * config.propagation_speed_mps
        / (2.0 * config.slope_hz_per_s)
    )
    doppler_frequency = np.fft.fftshift(
        np.fft.fftfreq(
            config.chirps_per_frame,
            d=config.pulse_repetition_interval_s,
        )
    )
    velocity_axis = doppler_frequency * config.wavelength_m / 2.0
    return RangeDopplerResult(
        range_axis_m=range_axis,
        velocity_axis_mps=velocity_axis,
        power_linear=power,
        peak_range_m=float(range_axis[peak_range_index]),
        peak_velocity_mps=float(velocity_axis[peak_doppler_index]),
        peak_power_linear=float(power[peak_doppler_index, peak_range_index]),
    )
