"""Validated 4x4 complex channel calibration representation."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def complex_matrix(amplitude: np.ndarray, phase_deg: np.ndarray) -> np.ndarray:
    amplitude = np.asarray(amplitude, dtype=float)
    phase_deg = np.asarray(phase_deg, dtype=float)
    if amplitude.shape != (4, 4) or phase_deg.shape != (4, 4):
        raise ValueError("calibration amplitude and phase must both be 4x4")
    if np.any(amplitude <= 0) or not np.all(np.isfinite(amplitude)) or not np.all(np.isfinite(phase_deg)):
        raise ValueError("calibration amplitude must be positive and all values finite")
    return amplitude * np.exp(1j * np.deg2rad(phase_deg))


def apply_channel_correction(channel: np.ndarray, correction: np.ndarray) -> np.ndarray:
    channel = np.asarray(channel)
    correction = np.asarray(correction)
    if channel.shape[-2:] != (4, 4) or correction.shape != (4, 4):
        raise ValueError("channel trailing dimensions and correction must be 4x4")
    return channel * correction


def load_calibration(path: Path) -> tuple[np.ndarray, dict]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    amplitude = np.asarray(payload["amplitude"], dtype=float)
    phase_deg = np.asarray(payload["phase_deg"], dtype=float)
    correction = complex_matrix(amplitude, phase_deg)
    metadata = {key: value for key, value in payload.items() if key not in {"amplitude", "phase_deg"}}
    return correction, metadata
