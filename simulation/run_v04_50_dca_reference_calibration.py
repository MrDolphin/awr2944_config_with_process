"""Process a DCA1000 reference capture into a power-calibration artifact."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.dca1000_iq import decode_interleaved_iq, reshape_tdm_virtual_channels
from simulation.v03 import FmcwConfig


def generate_synthetic_reference(path: Path, *, chirps: int = 64, samples_per_chirp: int = 128, rx_count: int = 4, range_bin: int = 12, doppler_bin: int = 0) -> None:
    """Create a small deterministic reference capture for contract testing."""
    fast = np.arange(samples_per_chirp)[None, :, None]
    slow = np.arange(chirps)[:, None, None]
    phase = 2.0 * np.pi * (range_bin * fast / samples_per_chirp + doppler_bin * slow / max(chirps, 1))
    iq = (1200.0 * np.exp(1j * phase) * np.ones((1, 1, rx_count))).astype(complex)
    path.parent.mkdir(parents=True, exist_ok=True); path.write_bytes(encode_capture_for_test(iq))


def encode_capture_for_test(iq: np.ndarray) -> bytes:
    """Encode a synthetic capture without changing the production decoder."""
    values = np.empty(iq.shape + (2,), dtype="<i2"); values[..., 0] = np.rint(iq.real).astype("<i2"); values[..., 1] = np.rint(iq.imag).astype("<i2"); return values.tobytes()


def load_capture(path: Path, *, chirps: int, samples_per_chirp: int, rx_count: int) -> tuple[np.ndarray, dict]:
    if path.suffix.lower() in {".h5", ".hdf5"}:
        with h5py.File(path, "r") as handle:
            if "/radar/iq" in handle:
                iq = handle["/radar/iq"][...]
            elif "/decoded/iq" in handle:
                iq = handle["/decoded/iq"][...]
            else:
                raise ValueError("HDF5 must contain /radar/iq or /decoded/iq")
        return np.asarray(iq), {"source_format": "hdf5", "wire_order_assumption": str("hdf5_dataset")}
    raw = path.read_bytes()
    return decode_interleaved_iq(raw, chirps=chirps, samples_per_chirp=samples_per_chirp, rx_count=rx_count), {"source_format": "dca1000_int16_bin", "wire_order_assumption": "sample -> rx -> I,Q", "channel_order_verified": False}


def range_doppler(iq: np.ndarray, config: FmcwConfig, tx_sequence: tuple[int, ...]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if iq.ndim != 3:
        raise ValueError("decoded capture must have shape (chirp, sample, rx)")
    virtual = reshape_tdm_virtual_channels(iq, tx_sequence=tx_sequence) if len(tx_sequence) > 1 else iq[:, :, :, None]
    range_window = np.hanning(virtual.shape[1]); doppler_window = np.hanning(virtual.shape[0])
    spectrum = np.fft.fft(virtual * range_window[None, :, None, None], axis=1)
    spectrum = np.fft.fftshift(np.fft.fft(spectrum * doppler_window[:, None, None, None], axis=0), axes=0)
    positive = np.arange(virtual.shape[1] // 2); spectrum = spectrum[:, positive, :, :]
    power = np.mean(np.abs(spectrum) ** 2, axis=(2, 3))
    ranges = positive * config.sample_rate_hz / virtual.shape[1] * config.propagation_speed_mps / (2.0 * config.slope_hz_per_s)
    velocities = np.fft.fftshift(np.fft.fftfreq(virtual.shape[0], d=config.pulse_repetition_interval_s * len(tx_sequence))) * config.wavelength_m / 2.0
    return spectrum, power, ranges, velocities, virtual


def process_capture(input_path: Path, output_path: Path, *, chirps: int, samples_per_chirp: int, rx_count: int, expected_range_m: float, expected_velocity_mps: float, tx_sequence: tuple[int, ...], synthetic_reference_power_linear: float | None = None, radar_config: dict | None = None) -> dict:
    iq, source = load_capture(input_path, chirps=chirps, samples_per_chirp=samples_per_chirp, rx_count=rx_count)
    config_values = radar_config or {}
    config = FmcwConfig(carrier_frequency_hz=float(config_values.get("start_freq_ghz", 77.0)) * 1e9, sweep_bandwidth_hz=float(config_values.get("sweep_bandwidth_hz", 1e9)), chirp_duration_s=float(config_values.get("chirp_duration_s", 60e-6)), sample_rate_hz=float(config_values.get("sample_rate_hz", 25e6)), samples_per_chirp=samples_per_chirp, chirps_per_frame=max(1, iq.shape[0] // len(tx_sequence)), pulse_repetition_interval_s=float(config_values.get("pulse_repetition_interval_s", 100e-6)))
    spectrum, power, ranges, velocities, virtual = range_doppler(iq, config, tx_sequence)
    ri = int(np.argmin(np.abs(ranges - expected_range_m))); di = int(np.argmin(np.abs(velocities - expected_velocity_mps)))
    window = power[max(0, di - 2):di + 3, max(0, ri - 2):ri + 3]; local_d, local_r = np.unravel_index(int(np.argmax(window)), window.shape); peak_d = max(0, di - 2) + local_d; peak_r = max(0, ri - 2) + local_r
    reference_power = float(power[peak_d, peak_r]); adc_rms = float(np.sqrt(np.mean(np.abs(iq) ** 2))); ratio = None if synthetic_reference_power_linear is None or synthetic_reference_power_linear <= 0 else reference_power / synthetic_reference_power_linear
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_path, "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-dca-reference-calibration-v0.4.50"; handle.attrs["source_file"] = str(input_path.resolve()); handle.attrs["channel_order_verified"] = False
        handle.create_dataset("/decoded/iq", data=iq, compression="gzip"); handle.create_dataset("/virtual/iq", data=virtual, compression="gzip"); handle.create_dataset("/range_doppler/spectrum_complex", data=spectrum.astype(np.complex64), compression="gzip"); handle.create_dataset("/range_doppler/power_linear", data=power.astype(np.float32), compression="gzip"); handle.create_dataset("/axes/range_m", data=ranges); handle.create_dataset("/axes/velocity_mps", data=velocities)
        handle.create_dataset("/calibration/adc_rms", data=adc_rms); handle.create_dataset("/calibration/reference_power_linear", data=reference_power)
    calibration = {"schema_version": "awr2944p-power-calibration-v0.4.50", "status": "capture_processed_channel_order_unverified", "source_file": str(input_path.resolve()), "source_format": source["source_format"], "expected_range_m": expected_range_m, "expected_velocity_mps": expected_velocity_mps, "peak_range_m": float(ranges[peak_r]), "peak_velocity_mps": float(velocities[peak_d]), "reference_power_linear": reference_power, "reference_adc_rms": adc_rms, "synthetic_reference_power_linear": synthetic_reference_power_linear, "synthetic_to_measured_power_ratio": ratio, "channel_order_verified": False, "tx_sequence": list(tx_sequence), "wire_order_assumption": source["wire_order_assumption"], "measurement_provenance_required": True}
    output_path.with_suffix(".calibration.json").write_text(json.dumps(calibration, indent=2, ensure_ascii=False), encoding="utf-8")
    output_path.with_suffix(".csv").write_text("metric,value\nreference_power_linear," + f"{reference_power:.12g}\nadc_rms," + f"{adc_rms:.12g}\npeak_range_m," + f"{ranges[peak_r]:.12g}\npeak_velocity_mps," + f"{velocities[peak_d]:.12g}\n", encoding="utf-8")
    return calibration


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--chirps", type=int, required=True); parser.add_argument("--samples-per-chirp", type=int, required=True); parser.add_argument("--rx-count", type=int, default=4); parser.add_argument("--expected-range-m", type=float, required=True); parser.add_argument("--expected-velocity-mps", type=float, default=0.0); parser.add_argument("--tx-sequence", default="0,1,2,3"); parser.add_argument("--synthetic-reference-power-linear", type=float); parser.add_argument("--generate-synthetic", action="store_true"); args = parser.parse_args();
    if args.generate_synthetic:
        generate_synthetic_reference(args.input.resolve(), chirps=args.chirps, samples_per_chirp=args.samples_per_chirp, rx_count=args.rx_count)
    tx_sequence = tuple(int(item) for item in args.tx_sequence.split(",") if item.strip() != ""); print(json.dumps(process_capture(args.input.resolve(), args.output.resolve(), chirps=args.chirps, samples_per_chirp=args.samples_per_chirp, rx_count=args.rx_count, expected_range_m=args.expected_range_m, expected_velocity_mps=args.expected_velocity_mps, tx_sequence=tx_sequence, synthetic_reference_power_linear=args.synthetic_reference_power_linear), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
