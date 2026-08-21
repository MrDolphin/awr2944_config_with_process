"""V0.4 minimal virtual-array phase and AoA contract."""

from __future__ import annotations

import numpy as np

from simulation.v03 import FmcwConfig, generate_single_scatterer_iq


def virtual_array_positions(config: FmcwConfig) -> tuple[np.ndarray, np.ndarray]:
    """Return 4x4 synthetic virtual positions in wavelengths (x, y).

    This is a software contract for AoA testing, not the AWR2944P EVM layout.
    TX and RX elements are separated by d (half wavelength).
    """
    if config.tx_count != 4 or config.rx_count != 4:
        raise ValueError("V0.4 contract requires 4 TX and 4 RX")
    d = config.wavelength_m / 2.0
    tx_x = np.arange(4, dtype=float) * d
    rx_y = np.arange(4, dtype=float) * d
    x = np.repeat(tx_x[None, :], 4, axis=0)
    y = np.repeat(rx_y[:, None], 4, axis=1)
    return x, y


def generate_aoa_scatterer_iq(
    config: FmcwConfig, *, slant_range_m: float, radial_velocity_mps: float,
    azimuth_deg: float, elevation_deg: float, amplitude: float = 1.0,
) -> np.ndarray:
    """Generate V0.3 IQ with a deterministic 4x4 virtual-array phase ramp."""
    x, y = virtual_array_positions(config)
    return generate_aoa_iq_with_positions(
        config, slant_range_m=slant_range_m, radial_velocity_mps=radial_velocity_mps,
        azimuth_deg=azimuth_deg, elevation_deg=elevation_deg,
        x_positions_m=x, y_positions_m=y, amplitude=amplitude,
    )


def generate_aoa_iq_with_positions(
    config: FmcwConfig, *, slant_range_m: float, radial_velocity_mps: float,
    azimuth_deg: float, elevation_deg: float, x_positions_m: np.ndarray,
    y_positions_m: np.ndarray, amplitude: float = 1.0,
) -> np.ndarray:
    """Generate IQ for arbitrary 4x4 virtual positions in board coordinates."""
    if x_positions_m.shape != (4, 4) or y_positions_m.shape != (4, 4):
        raise ValueError("virtual positions must have shape (4, 4)")
    iq = generate_single_scatterer_iq(
        config, slant_range_m=slant_range_m,
        radial_velocity_mps=radial_velocity_mps, amplitude=amplitude,
    )
    az = np.deg2rad(azimuth_deg)
    el = np.deg2rad(elevation_deg)
    phase = 2.0 * np.pi / config.wavelength_m * (
        x_positions_m * np.sin(az) * np.cos(el) + y_positions_m * np.sin(el)
    )
    channel_phase = np.exp(1j * phase)[None, None, :, :]
    return iq * channel_phase


def estimate_aoa_from_channels(iq: np.ndarray, config: FmcwConfig) -> tuple[float, float]:
    """Estimate azimuth/elevation from channel phase slopes.

    Uses averaged complex channel phase and unwraps each virtual axis. This is
    intentionally a noiseless V0.4 contract, not a production MUSIC/ESPRIT
    estimator or the TI SDK AoA implementation.
    """
    expected = (config.chirps_per_frame, config.samples_per_chirp, 4, 4)
    if iq.shape != expected:
        raise ValueError(f"IQ shape must be {expected}, got {iq.shape}")
    channel = np.mean(iq, axis=(0, 1))
    x, y = virtual_array_positions(config)
    return estimate_aoa_from_positions(channel, config, x, y)


def estimate_aoa_from_positions(
    channel: np.ndarray, config: FmcwConfig, x_positions_m: np.ndarray,
    y_positions_m: np.ndarray,
) -> tuple[float, float]:
    """Estimate AoA from arbitrary virtual positions using a phase-plane fit."""
    if channel.shape != (4, 4) or x_positions_m.shape != (4, 4) or y_positions_m.shape != (4, 4):
        raise ValueError("channel and virtual positions must have shape (4, 4)")
    phase = np.unwrap(np.unwrap(np.angle(channel), axis=1), axis=0)
    design = np.column_stack((x_positions_m.ravel(), y_positions_m.ravel(), np.ones(16)))
    slope_x, slope_y, _ = np.linalg.lstsq(design, phase.ravel(), rcond=None)[0]
    sin_az_cos_el = np.clip(slope_x * config.wavelength_m / (2.0 * np.pi), -1.0, 1.0)
    sin_el = np.clip(slope_y * config.wavelength_m / (2.0 * np.pi), -1.0, 1.0)
    elevation = float(np.rad2deg(np.arcsin(sin_el)))
    azimuth = float(np.rad2deg(np.arcsin(sin_az_cos_el / max(np.cos(np.deg2rad(elevation)), 1e-9))))
    return azimuth, elevation
