"""Analyze MATLAB dynamic-sea samples into V0.2 truth and kinematics."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shutil

import numpy as np

from simulation.artifacts import create_run_directory
from simulation.v02 import (
    DynamicSeaTruth,
    RawSeaSurface,
    analyze_height_cube,
    load_config,
    read_raw_hdf5,
    write_truth_hdf5,
)


def _finite_or_none(value: float) -> float | None:
    return float(value) if math.isfinite(value) else None


def _wrap_direction_deg(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


def _summary(raw: RawSeaSurface, truth: DynamicSeaTruth) -> dict[str, object]:
    configured_vessel_direction_deg = _wrap_direction_deg(
        90.0 - raw.wind_direction_deg
    )
    direction_error_deg = (
        _wrap_direction_deg(
            truth.dominant_wave_direction_deg - configured_vessel_direction_deg
        )
        if math.isfinite(truth.dominant_wave_direction_deg)
        else math.nan
    )
    return {
        "case_id": raw.case_id,
        "sea_state": raw.sea_state,
        "random_seed": raw.random_seed,
        "target_hs_m": raw.target_hs_m,
        "raw_hs_m": raw.raw_hs_m,
        "amplitude_scale_factor": raw.amplitude_scale_factor,
        "achieved_hs_m": truth.achieved_hs_m,
        "hs_validation_passed": truth.hs_validation_passed,
        "min_radar_clearance_m": truth.min_radar_clearance_m,
        "geometry_valid": truth.min_radar_clearance_m > 0.0,
        "height_min_m": float(truth.height_m.min()),
        "height_max_m": float(truth.height_m.max()),
        "vertical_velocity_p95_mps": float(
            np.percentile(np.abs(truth.vertical_velocity_mps), 95.0)
        ),
        "slant_range_rate_p95_mps": float(
            np.percentile(np.abs(truth.slant_range_rate_mps), 95.0)
        ),
        "dominant_wave_direction_deg": _finite_or_none(
            truth.dominant_wave_direction_deg
        ),
        "dominant_wave_period_s": _finite_or_none(
            truth.dominant_wave_period_s
        ),
        "dominant_wavelength_m": _finite_or_none(
            truth.dominant_wavelength_m
        ),
        "dominant_phase_speed_mps": _finite_or_none(
            truth.dominant_phase_speed_mps
        ),
        "configured_vessel_wave_direction_deg": configured_vessel_direction_deg,
        "dominant_direction_error_deg": _finite_or_none(direction_error_deg),
        "grazing_angle_mean_deg": float(np.mean(truth.grazing_angle_deg)),
        "grazing_angle_spatiotemporal_std_deg": float(
            np.std(truth.grazing_angle_deg)
        ),
        "grazing_angle_temporal_std_mean_deg": float(
            np.mean(np.std(truth.grazing_angle_deg, axis=0))
        ),
        "elevation_spatiotemporal_std_deg": float(np.std(truth.elevation_deg)),
        "elevation_temporal_std_mean_deg": float(
            np.mean(np.std(truth.elevation_deg, axis=0))
        ),
        "wind_speed_mps": raw.wind_speed_mps,
        "wind_direction_deg": raw.wind_direction_deg,
        "fetch_m": raw.fetch_m,
    }


def _plot_truth_overview(
    raw: RawSeaSurface, truth: DynamicSeaTruth, output_path: Path
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    height_mesh = axes[0, 0].pcolormesh(
        truth.x_m,
        truth.y_m,
        truth.height_m[0],
        shading="auto",
        cmap="coolwarm",
    )
    axes[0, 0].set_title("Sea height at t=0 s")
    axes[0, 0].set_xlabel("x right (m)")
    axes[0, 0].set_ylabel("y forward (m)")
    figure.colorbar(height_mesh, ax=axes[0, 0], label="height (m)")

    grazing_mesh = axes[0, 1].pcolormesh(
        truth.x_m,
        truth.y_m,
        truth.grazing_angle_deg[0],
        shading="auto",
        cmap="viridis",
    )
    axes[0, 1].set_title("Local grazing angle at t=0 s")
    axes[0, 1].set_xlabel("x right (m)")
    axes[0, 1].set_ylabel("y forward (m)")
    figure.colorbar(grazing_mesh, ax=axes[0, 1], label="angle (deg)")

    flattened = truth.height_m.reshape(truth.time_s.size, -1)
    axes[1, 0].plot(truth.time_s, flattened.min(axis=1), label="min")
    axes[1, 0].plot(truth.time_s, flattened.mean(axis=1), label="mean")
    axes[1, 0].plot(truth.time_s, flattened.max(axis=1), label="max")
    axes[1, 0].set_title("Surface-height envelope")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 0].set_ylabel("height (m)")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].hist(truth.height_m.ravel(), bins=40, color="steelblue")
    axes[1, 1].set_title(
        f"Height distribution: target Hs={raw.target_hs_m:.2f} m, "
        f"achieved={truth.achieved_hs_m:.2f} m"
    )
    axes[1, 1].set_xlabel("height (m)")
    axes[1, 1].set_ylabel("samples")
    axes[1, 1].grid(True, alpha=0.3)

    vessel_direction = _wrap_direction_deg(90.0 - raw.wind_direction_deg)
    figure.suptitle(
        f"{raw.case_id}, seed={raw.random_seed}, sea state={raw.sea_state} | "
        f"MATLAB WindDirection={raw.wind_direction_deg:.1f} deg, "
        f"vessel wave direction={vessel_direction:.1f} deg (+y forward)"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def _plot_comparison(summaries: list[dict[str, object]], output_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    labels = [f"{item['case_id']}\nseed{item['random_seed']}" for item in summaries]
    positions = np.arange(len(labels))
    target = [float(item["target_hs_m"]) for item in summaries]
    achieved = [float(item["achieved_hs_m"]) for item in summaries]
    clearance = [float(item["min_radar_clearance_m"]) for item in summaries]
    grazing_temporal_std = [
        float(item["grazing_angle_temporal_std_mean_deg"])
        for item in summaries
    ]
    velocity_p95 = [
        float(item["slant_range_rate_p95_mps"]) for item in summaries
    ]

    figure, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    width = 0.38
    axes[0, 0].bar(positions - width / 2, target, width, label="target")
    axes[0, 0].bar(positions + width / 2, achieved, width, label="achieved")
    axes[0, 0].set_ylabel("Hs (m)")
    axes[0, 0].set_title("Significant wave-height validation")
    axes[0, 0].legend()

    axes[0, 1].bar(positions, clearance)
    axes[0, 1].axhline(0.0, color="red", linewidth=1)
    axes[0, 1].set_ylabel("minimum clearance (m)")
    axes[0, 1].set_title("Radar clearance above sampled surface")

    axes[1, 0].bar(positions, grazing_temporal_std)
    axes[1, 0].set_ylabel("mean per-cell temporal std (deg)")
    axes[1, 0].set_title("Wave-induced grazing-angle temporal variability")

    axes[1, 1].bar(positions, velocity_p95)
    axes[1, 1].set_ylabel("95th percentile |radial range rate| (m/s)")
    axes[1, 1].set_title("Eulerian surface radial-motion envelope")

    for axis in axes.ravel():
        axis.set_xticks(positions, labels, rotation=30, ha="right")
        axis.grid(True, axis="y", alpha=0.3)
    wind_direction = float(summaries[0]["wind_direction_deg"])
    vessel_direction = _wrap_direction_deg(90.0 - wind_direction)
    figure.suptitle(
        f"V0.2 sea-state comparison | MATLAB WindDirection={wind_direction:.1f} deg | "
        f"vessel wave direction={vessel_direction:.1f} deg (+y forward)"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def analyze_matlab_run(
    *,
    input_run: Path,
    config_path: Path,
    results_root: Path | None = None,
    run_id: str | None = None,
    render_plots: bool = True,
) -> Path:
    """Analyze every raw MATLAB HDF5 case into one isolated Python run."""

    config = load_config(config_path)
    radar_value = config.raw.get("radar")
    if not isinstance(radar_value, dict) or "cfg_path" not in radar_value:
        raise ValueError("radar.cfg_path is required for the run snapshot")
    radar_path = (
        config.source_path.parent / str(radar_value["cfg_path"])
    ).resolve()
    if not radar_path.is_file():
        raise FileNotFoundError(f"radar CFG does not exist: {radar_path}")
    input_files = sorted((input_run / "data").glob("*.h5"))
    if not input_files:
        raise ValueError(f"no HDF5 files found in {input_run / 'data'}")
    expected_cases = {case.case_id: case for case in config.cases}
    sea_surface_value = config.raw.get("sea_surface")
    if not isinstance(sea_surface_value, dict) or "wind_direction_deg" not in sea_surface_value:
        raise ValueError("sea_surface.wind_direction_deg is required")
    expected_matlab_wind_direction_deg = float(
        sea_surface_value["wind_direction_deg"]
    )
    for input_path in input_files:
        raw = read_raw_hdf5(input_path)
        expected = expected_cases.get(raw.case_id)
        if expected is None:
            raise ValueError(f"unknown case_id {raw.case_id!r}")
        if raw.sea_state != expected.sea_state or not math.isclose(
            raw.target_hs_m, expected.target_hs_m, rel_tol=0.0, abs_tol=1e-12
        ):
            raise ValueError(
                f"{raw.case_id} identity mismatch: expected sea_state="
                f"{expected.sea_state}, target_hs_m={expected.target_hs_m}"
            )
        if not math.isclose(
            raw.wind_direction_deg,
            expected_matlab_wind_direction_deg,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError(
                f"{raw.case_id} wind direction mismatch: expected MATLAB "
                f"WindDirection={expected_matlab_wind_direction_deg}, got "
                f"{raw.wind_direction_deg}"
            )
    output_root_value = config.raw["output"]
    if not isinstance(output_root_value, dict):
        raise ValueError("output must be an object")
    configured_root = (
        config.source_path.parent / str(output_root_value["directory"])
    ).resolve()
    output_run = create_run_directory(
        results_root or configured_root,
        producer="python",
        stage_id="v02_dynamic_sea_truth",
        run_id=run_id,
    )
    shutil.copy2(config.source_path, output_run / "run_config.json")
    shutil.copy2(radar_path, output_run / "radar_profile.cfg")
    (output_run / "input_source.json").write_text(
        json.dumps(
            {"matlab_run": str(input_run.resolve())},
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    validation_value = config.raw["validation"]
    if not isinstance(validation_value, dict):
        raise ValueError("validation must be an object")
    hs_relative_tolerance = float(validation_value["hs_relative_tolerance"])
    flat_tolerance = float(validation_value["flat_hs_absolute_tolerance_m"])
    summaries: list[dict[str, object]] = []
    for input_path in input_files:
        raw = read_raw_hdf5(input_path)
        truth = analyze_height_cube(
            x_m=raw.x_m,
            y_m=raw.y_m,
            time_s=raw.time_s,
            height_m=raw.height_m,
            radar_height_m=raw.radar_height_m,
            mounting_pitch_deg=raw.mounting_pitch_deg,
            target_hs_m=raw.target_hs_m,
            hs_relative_tolerance=hs_relative_tolerance,
            flat_hs_absolute_tolerance_m=flat_tolerance,
        )
        stem = f"{raw.case_id}_seed{raw.random_seed}"
        write_truth_hdf5(raw, truth, output_run / "data" / f"{stem}_truth.h5")
        if render_plots:
            _plot_truth_overview(
                raw, truth, output_run / "figures" / f"{stem}_overview.png"
            )
        summaries.append(_summary(raw, truth))

    (output_run / "summary.json").write_text(
        json.dumps(summaries, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )
    if render_plots:
        _plot_comparison(
            summaries, output_run / "figures" / "sea_state_comparison.png"
        )
    passed = sum(bool(item["hs_validation_passed"]) for item in summaries)
    geometry_warnings = sum(not bool(item["geometry_valid"]) for item in summaries)
    (output_run / "validation.md").write_text(
        "# Validation\n\n"
        f"- [{'x' if passed == len(summaries) else ' '}] Hs checks passed: "
        f"{passed}/{len(summaries)}\n"
        f"- [ ] Non-positive radar-clearance warnings: {geometry_warnings}\n"
        "- [ ] Manual figure review recorded\n",
        encoding="utf-8",
    )
    with (output_run / "design_snapshot.md").open("a", encoding="utf-8") as handle:
        handle.write(
            "\n- Input MATLAB run: `" + str(input_run.resolve()) + "`\n"
            "- Scope: dynamic surface truth only; no IQ, clutter power, AoA, or CFAR.\n"
        )
    return output_run


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze MATLAB seaSurface samples into V0.2 truth and kinematics."
    )
    parser.add_argument("--input-run", type=Path, required=True)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("configs") / "sea_states_0_to_3.json",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--run-id")
    parser.add_argument("--no-plots", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output_run = analyze_matlab_run(
        input_run=args.input_run.resolve(),
        config_path=args.config.resolve(),
        results_root=args.output.resolve() if args.output else None,
        run_id=args.run_id,
        render_plots=not args.no_plots,
    )
    print(f"Analyzed MATLAB V0.2 run into {output_run}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
