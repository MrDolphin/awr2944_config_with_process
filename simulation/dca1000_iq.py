"""Minimal DCA1000 int16 IQ decoder with explicit layout metadata."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import h5py
import numpy as np


def decode_interleaved_iq(
    raw: bytes, *, chirps: int, samples_per_chirp: int, rx_count: int = 4,
) -> np.ndarray:
    """Decode signed int16 I/Q sequence to (chirp, sample, rx) complex IQ.

    Assumed wire order is per sample: rx0-I, rx0-Q, rx1-I, rx1-Q, ...;
    chirps and samples are contiguous. This assumption is recorded in output
    metadata and must be confirmed against the capture setup.
    """
    if chirps <= 0 or samples_per_chirp <= 0 or rx_count <= 0:
        raise ValueError("chirps, samples_per_chirp and rx_count must be positive")
    values = np.frombuffer(raw, dtype="<i2")
    expected = chirps * samples_per_chirp * rx_count * 2
    if values.size != expected:
        raise ValueError(f"expected {expected} int16 values, got {values.size}")
    values = values.reshape(chirps, samples_per_chirp, rx_count, 2)
    return values[..., 0].astype(np.float32) + 1j * values[..., 1].astype(np.float32)


def reshape_tdm_virtual_channels(
    iq: np.ndarray, *, tx_sequence: tuple[int, ...] = (0, 1, 2, 3),
) -> np.ndarray:
    """Group decoded TDM chirps into (frame, sample, rx, tx) virtual channels."""
    if iq.ndim != 3:
        raise ValueError("decoded IQ must have shape (chirp, sample, rx)")
    if not tx_sequence or len(set(tx_sequence)) != len(tx_sequence):
        raise ValueError("tx_sequence must contain unique TX indices")
    tx_count = len(tx_sequence)
    if iq.shape[0] % tx_count:
        raise ValueError("chirp count must be divisible by TX sequence length")
    frames = iq.shape[0] // tx_count
    output = np.empty((frames, iq.shape[1], iq.shape[2], tx_count), dtype=iq.dtype)
    for position, tx_index in enumerate(tx_sequence):
        output[:, :, :, tx_index] = iq[position::tx_count]
    return output


def flatten_tdm_virtual_channels(virtual: np.ndarray, *, tx_sequence: tuple[int, ...] = (0, 1, 2, 3)) -> np.ndarray:
    """Inverse of :func:`reshape_tdm_virtual_channels` for synthetic tests."""
    if virtual.ndim != 4 or virtual.shape[3] != len(tx_sequence):
        raise ValueError("virtual IQ must have shape (frame, sample, rx, tx)")
    tx_count = len(tx_sequence)
    output = np.empty((virtual.shape[0] * tx_count, virtual.shape[1], virtual.shape[2]), dtype=virtual.dtype)
    for position, tx_index in enumerate(tx_sequence):
        output[position::tx_count] = virtual[:, :, :, tx_index]
    return output


def encode_interleaved_iq(iq: np.ndarray) -> bytes:
    """Encode (chirp, sample, rx) complex IQ as little-endian int16 I/Q."""
    if iq.ndim != 3:
        raise ValueError("IQ must have shape (chirp, sample, rx)")
    if np.max(np.abs(iq.real)) > 32767 or np.max(np.abs(iq.imag)) > 32767:
        raise ValueError("IQ exceeds int16 range")
    values = np.empty(iq.shape + (2,), dtype="<i2")
    values[..., 0] = np.rint(iq.real).astype("<i2")
    values[..., 1] = np.rint(iq.imag).astype("<i2")
    return values.tobytes()


def decode_file(input_path: Path, output_path: Path, *, chirps: int,
                samples_per_chirp: int, rx_count: int = 4) -> None:
    iq = decode_interleaved_iq(input_path.read_bytes(), chirps=chirps,
                               samples_per_chirp=samples_per_chirp, rx_count=rx_count)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_path, "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-dca1000-iq-v0.4.7"
        handle.attrs["source_file"] = str(input_path.resolve())
        handle.attrs["raw_dtype"] = "little_endian_int16"
        handle.attrs["wire_order_assumption"] = "sample -> rx -> I,Q"
        handle.attrs["channel_order_verified"] = False
        handle.create_dataset("/radar/iq", data=iq, compression="gzip")
    output_path.with_suffix(".json").write_text(json.dumps({
        "source_file": str(input_path.resolve()), "chirps": chirps,
        "samples_per_chirp": samples_per_chirp, "rx_count": rx_count,
        "shape": list(iq.shape), "channel_order_verified": False,
    }, indent=2, ensure_ascii=False), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--chirps", type=int, required=True)
    parser.add_argument("--samples-per-chirp", type=int, required=True)
    parser.add_argument("--rx-count", type=int, default=4)
    args = parser.parse_args()
    decode_file(args.input.resolve(), args.output.resolve(), chirps=args.chirps,
                samples_per_chirp=args.samples_per_chirp, rx_count=args.rx_count)
    print(f"Decoded DCA1000 IQ into {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
