"""Link an AWR2944P CFG to DCA1000 decode/calibration parameters."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from simulation.run_v04_50_dca_reference_calibration import process_capture


def derive_capture_contract(cfg: dict) -> dict:
    required = ("num_rx", "num_adc_samples", "num_chirps_per_frame", "chirp_tx_masks")
    missing = [key for key in required if key not in cfg]
    if missing:
        raise ValueError(f"CFG is missing capture fields: {', '.join(missing)}")
    masks = cfg["chirp_tx_masks"]
    start = int(cfg.get("frame_chirp_start", min(masks)))
    end = int(cfg.get("frame_chirp_end", max(masks)))
    sequence = []
    for chirp in range(start, end + 1):
        mask = int(masks[chirp])
        active = [index for index in range(8) if mask & (1 << index)]
        if len(active) != 1:
            raise ValueError(f"chirp {chirp} must have exactly one active TX for TDM calibration; mask={mask}")
        sequence.append(active[0])
    if len(sequence) == 0 or len(set(sequence)) != len(sequence):
        raise ValueError("CFG TX sequence must contain unique TX indices")
    bytes_per_chirp = int(cfg["num_adc_samples"]) * int(cfg["num_rx"]) * 4
    return {"rx_count": int(cfg["num_rx"]), "samples_per_chirp": int(cfg["num_adc_samples"]), "chirps_per_frame": int(cfg["num_chirps_per_frame"]), "tx_sequence": tuple(sequence), "bytes_per_frame": bytes_per_chirp * int(cfg["num_chirps_per_frame"]), "radar_config": {"start_freq_ghz": float(cfg.get("start_freq_ghz", 77.0)), "sweep_bandwidth_hz": float(cfg.get("freq_slope_mhz_per_us", 0.0)) * float(cfg.get("ramp_end_time_us", 0.0)) * 1e6, "chirp_duration_s": float(cfg.get("ramp_end_time_us", 60.0)) * 1e-6, "sample_rate_hz": float(cfg.get("sample_rate_ksps", 25000.0)) * 1e3, "pulse_repetition_interval_s": float(cfg.get("frame_period_ms", 0.1)) * 1e-3 / max(1, int(cfg["num_chirps_per_frame"]))}}


def run(cfg_path: Path, input_path: Path, output_path: Path, *, expected_range_m: float, expected_velocity_mps: float, synthetic_reference_power_linear: float | None = None) -> dict:
    from tools.dca1000_capture import parse_radar_cfg
    parsed = parse_radar_cfg(str(cfg_path)); contract = derive_capture_contract(parsed)
    if input_path.suffix.lower() not in {".h5", ".hdf5"}:
        size = input_path.stat().st_size
        if size % contract["bytes_per_frame"] != 0:
            raise ValueError(f"capture bytes {size} is not a whole number of CFG frames ({contract['bytes_per_frame']} bytes/frame)")
    result = process_capture(input_path, output_path, chirps=contract["chirps_per_frame"], samples_per_chirp=contract["samples_per_chirp"], rx_count=contract["rx_count"], expected_range_m=expected_range_m, expected_velocity_mps=expected_velocity_mps, tx_sequence=contract["tx_sequence"], synthetic_reference_power_linear=synthetic_reference_power_linear, radar_config=contract["radar_config"])
    result.update({"cfg": str(cfg_path.resolve()), "capture_contract": {key: value for key, value in contract.items() if key != "radar_config"}, "radar_config": contract["radar_config"]}); output_path.with_suffix(".cfg_contract.json").write_text(json.dumps(result, indent=2, ensure_ascii=False, default=list), encoding="utf-8"); return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--cfg", type=Path, required=True); parser.add_argument("--input", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--expected-range-m", type=float, required=True); parser.add_argument("--expected-velocity-mps", type=float, default=0.0); parser.add_argument("--synthetic-reference-power-linear", type=float); args = parser.parse_args(); print(json.dumps(run(args.cfg.resolve(), args.input.resolve(), args.output.resolve(), expected_range_m=args.expected_range_m, expected_velocity_mps=args.expected_velocity_mps, synthetic_reference_power_linear=args.synthetic_reference_power_linear), indent=2, ensure_ascii=False, default=list)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
