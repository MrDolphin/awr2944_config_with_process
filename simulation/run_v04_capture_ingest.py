"""Ingest a DCA1000 capture into decoded/TDM/calibrated HDF5 artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.calibration import apply_channel_correction, load_calibration
from simulation.dca1000_iq import decode_interleaved_iq, reshape_tdm_virtual_channels


def run(input_path: Path, cfg_path: Path, output: Path, *, chirps: int,
        samples_per_chirp: int, rx_count: int, tx_sequence: tuple[int, ...],
        calibration_path: Path | None = None) -> dict:
    raw = input_path.read_bytes()
    decoded = decode_interleaved_iq(raw, chirps=chirps, samples_per_chirp=samples_per_chirp, rx_count=rx_count)
    virtual = reshape_tdm_virtual_channels(decoded, tx_sequence=tx_sequence)
    correction = None; calibration_meta = {"applied": False}
    calibrated = None
    if calibration_path:
        correction, metadata = load_calibration(calibration_path)
        calibrated = apply_channel_correction(virtual, correction)
        calibration_meta = {"applied": True, "source": str(calibration_path.resolve()), **metadata}
    output.parent.mkdir(parents=True, exist_ok=True)
    cfg_hash = hashlib.sha256(cfg_path.read_bytes()).hexdigest()
    with h5py.File(output, "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-capture-ingest-v0.4.21"
        handle.attrs["source_file"] = str(input_path.resolve())
        handle.attrs["cfg_file"] = str(cfg_path.resolve())
        handle.attrs["cfg_sha256"] = cfg_hash
        handle.attrs["wire_order_assumption"] = "sample -> rx -> I,Q"
        handle.attrs["tx_sequence"] = ",".join(f"TX{index + 1}" for index in tx_sequence)
        handle.attrs["channel_order_verified"] = False
        handle.create_dataset("/decoded/iq", data=decoded, compression="gzip")
        handle.create_dataset("/recovered/virtual_iq", data=virtual, compression="gzip")
        if calibrated is not None:
            handle.create_dataset("/calibrated/virtual_iq", data=calibrated, compression="gzip")
    summary = {
        "input_file": str(input_path.resolve()), "input_bytes": len(raw),
        "cfg_file": str(cfg_path.resolve()), "cfg_sha256": cfg_hash,
        "chirps": chirps, "samples_per_chirp": samples_per_chirp, "rx_count": rx_count,
        "tx_sequence": list(tx_sequence), "decoded_shape": list(decoded.shape),
        "virtual_shape": list(virtual.shape), "calibration": calibration_meta,
        "channel_order_verified": False, "aoa_status": "not_estimated_capture_ingest_only",
    }
    output.with_suffix(".json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--cfg", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--chirps", type=int, required=True)
    parser.add_argument("--samples-per-chirp", type=int, required=True)
    parser.add_argument("--rx-count", type=int, default=4)
    parser.add_argument("--tx-sequence", type=int, nargs=4, default=(0, 1, 2, 3))
    parser.add_argument("--calibration", type=Path)
    args = parser.parse_args()
    summary = run(args.input.resolve(), args.cfg.resolve(), args.output.resolve(),
                  chirps=args.chirps, samples_per_chirp=args.samples_per_chirp,
                  rx_count=args.rx_count, tx_sequence=tuple(args.tx_sequence),
                  calibration_path=args.calibration.resolve() if args.calibration else None)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
