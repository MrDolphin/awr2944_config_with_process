"""Run the V0.3 minimal single-scatterer echo contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil

import h5py
import numpy as np

from simulation.artifacts import create_run_directory
from simulation.v03 import FmcwConfig, generate_single_scatterer_iq, process_range_doppler


def run_single_scatterer(
    *,
    input_truth: Path,
    results_root: Path,
    run_id: str,
    time_index: int = 0,
    y_index: int = 0,
    x_index: int = 0,
    amplitude: float = 1.0,
) -> Path:
    with h5py.File(input_truth, "r") as handle:
        slant_range_m = float(handle["/truth/slant_range_m"][time_index, y_index, x_index])
        radial_velocity_mps = float(
            handle["/truth/slant_range_rate_mps"][time_index, y_index, x_index]
        )
        source_attrs = {
            "case_id": str(handle.attrs["case_id"]),
            "source_producer": str(handle.attrs["producer"]),
        }
    config = FmcwConfig()
    iq = generate_single_scatterer_iq(
        config,
        slant_range_m=slant_range_m,
        radial_velocity_mps=radial_velocity_mps,
        amplitude=amplitude,
    )
    result = process_range_doppler(iq, config)
    output_run = create_run_directory(
        results_root,
        producer="python",
        stage_id="v03_complex_echo_range_doppler",
        run_id=run_id,
    )
    source_run = input_truth.parent.parent
    for name in ("run_config.json", "radar_profile.cfg"):
        source = source_run / name
        if source.is_file():
            shutil.copy2(source, output_run / name)
    with h5py.File(output_run / "data" / "single_scatterer.h5", "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-complex-echo-v0.3"
        handle.attrs["producer"] = "python"
        handle.attrs["source_truth"] = str(input_truth.resolve())
        handle.create_dataset("/radar/iq", data=iq, compression="gzip")
        handle.create_dataset("/range_doppler/power_linear", data=result.power_linear)
        handle.create_dataset("/axes/range_m", data=result.range_axis_m)
        handle.create_dataset("/axes/velocity_mps", data=result.velocity_axis_mps)
        handle.create_dataset("/truth/slant_range_m", data=slant_range_m)
        handle.create_dataset("/truth/slant_range_rate_mps", data=radial_velocity_mps)
    summary = {
        **source_attrs,
        "source_truth": str(input_truth.resolve()),
        "slant_range_m": slant_range_m,
        "slant_range_rate_mps": radial_velocity_mps,
        "peak_range_m": result.peak_range_m,
        "peak_velocity_mps": result.peak_velocity_mps,
        "peak_power_linear": result.peak_power_linear,
        "fmcw": {
            "carrier_frequency_hz": config.carrier_frequency_hz,
            "sweep_bandwidth_hz": config.sweep_bandwidth_hz,
            "chirp_duration_s": config.chirp_duration_s,
            "sample_rate_hz": config.sample_rate_hz,
            "samples_per_chirp": config.samples_per_chirp,
            "chirps_per_frame": config.chirps_per_frame,
            "pulse_repetition_interval_s": config.pulse_repetition_interval_s,
            "tx_count": config.tx_count,
            "rx_count": config.rx_count,
        },
    }
    (output_run / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    (output_run / "validation.md").write_text(
        "# Validation\n\n"
        "- [x] Single-scatterer complex IQ generated\n"
        "- [x] Range-Doppler peak generated\n"
        "- [ ] MATLAB cross-language IQ comparison\n"
        "- [ ] Sea-clutter multi-scatterer accumulation\n"
        "- [ ] AoA/CFAR (reserved for V0.4)\n",
        encoding="utf-8",
    )
    return output_run


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-truth", type=Path, required=True)
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--time-index", type=int, default=0)
    parser.add_argument("--y-index", type=int, default=0)
    parser.add_argument("--x-index", type=int, default=0)
    parser.add_argument("--amplitude", type=float, default=1.0)
    args = parser.parse_args()
    output = run_single_scatterer(
        input_truth=args.input_truth.resolve(),
        results_root=args.results_root.resolve(),
        run_id=args.run_id,
        time_index=args.time_index,
        y_index=args.y_index,
        x_index=args.x_index,
        amplitude=args.amplitude,
    )
    print(f"Generated V0.3 single-scatterer run into {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
