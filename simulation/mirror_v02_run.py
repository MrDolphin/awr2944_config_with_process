"""Create a controlled spatially mirrored copy of a V0.2 raw run."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import shutil

from simulation.artifacts import create_run_directory
from simulation.v02 import read_raw_hdf5, write_raw_hdf5


def mirror_height_cube(height_m, axis: str):
    """Reflect a (time, y, x) height cube without changing its axes."""

    if axis == "x":
        return height_m[:, :, ::-1].copy()
    if axis == "y":
        return height_m[:, ::-1, :].copy()
    raise ValueError("axis must be 'x' or 'y'")


def mirror_run(*, input_run: Path, results_root: Path, run_id: str, axis: str) -> Path:
    output_run = create_run_directory(
        results_root,
        producer="python_mirror",
        stage_id="v02_dynamic_sea_truth",
        run_id=run_id,
    )
    source_config = input_run / "run_config.json"
    if source_config.is_file():
        shutil.copy2(source_config, output_run / "run_config.json")
    source_cfg = input_run / "radar_profile.cfg"
    if source_cfg.is_file():
        shutil.copy2(source_cfg, output_run / "radar_profile.cfg")

    for input_path in sorted((input_run / "data").glob("*.h5")):
        raw = read_raw_hdf5(input_path)
        mirrored = replace(
            raw,
            producer="python_mirror",
            height_m=mirror_height_cube(raw.height_m, axis),
        )
        write_raw_hdf5(
            mirrored,
            output_run / "data" / input_path.name,
        )
    with (output_run / "design_snapshot.md").open("a", encoding="utf-8") as handle:
        handle.write(
            f"\n- Input run: `{input_run.resolve()}`\n"
            f"- Spatial mirror axis: `{axis}`\n"
            "- This is a controlled geometry regression, not a new MATLAB sea-spectrum run.\n"
        )
    return output_run


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-run", type=Path, required=True)
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--axis", choices=("x", "y"), default="y")
    args = parser.parse_args()
    output = mirror_run(
        input_run=args.input_run.resolve(),
        results_root=args.results_root.resolve(),
        run_id=args.run_id,
        axis=args.axis,
    )
    print(f"Mirrored V0.2 run into {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
