"""Command-line entry point for the AWR2944P flat-sea V0.1 simulation."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

from simulation.v01 import load_config, plot_hdf5, run_sweep


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run or visualize the deterministic AWR2944P flat-sea V0.1 model."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("configs") / "baseline_1m.json",
        help="JSON run configuration used for a Python pitch sweep.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Override the output directory for a Python pitch sweep.",
    )
    parser.add_argument(
        "--plot-hdf5",
        type=Path,
        help="Only render an existing MATLAB/Python V0.1 HDF5 file.",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Generate HDF5 and summaries without PNG figures.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.plot_hdf5:
        output_path = plot_hdf5(args.plot_hdf5)
        print(f"Rendered {output_path}")
        return 0

    config = load_config(args.config)
    if args.output:
        config = replace(config, output_directory=args.output.resolve())
    summaries = run_sweep(config, render_plots=not args.no_plots)
    print(f"Generated {len(summaries)} cases in {config.output_directory}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
