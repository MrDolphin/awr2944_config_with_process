"""Compare ideal and PCB-derived AWR2944P virtual arrays for AoA."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.artifacts import create_run_directory
from simulation.v03 import FmcwConfig
from simulation.v04 import (
    estimate_aoa_from_positions,
    generate_aoa_iq_with_positions,
    virtual_array_positions,
)


def _load_cad(path: Path, config: FmcwConfig) -> tuple[np.ndarray, np.ndarray]:
    rows = list(csv.DictReader(path.open(encoding="utf-8", newline="")))
    if len(rows) != 16:
        raise ValueError("CAD virtual array CSV must contain 16 channels")
    x = np.zeros((4, 4), dtype=float)
    y = np.zeros((4, 4), dtype=float)
    names = {f"TX{i}": i - 1 for i in range(1, 5)}
    rnames = {f"RX{i}": i - 1 for i in range(1, 5)}
    for row in rows:
        x[rnames[row["rx"]], names[row["tx"]]] = float(row["x_mm"]) * 1e-3
        y[rnames[row["rx"]], names[row["tx"]]] = float(row["y_mm"]) * 1e-3
    return x, y


def _scan(config: FmcwConfig, x: np.ndarray, y: np.ndarray) -> dict[str, float]:
    azimuths = np.arange(-60.0, 60.1, 20.0)
    elevations = np.arange(-20.0, 20.1, 10.0)
    errors = []
    for elevation in elevations:
        for azimuth in azimuths:
            iq = generate_aoa_iq_with_positions(
                config, slant_range_m=30.0, radial_velocity_mps=0.0,
                azimuth_deg=float(azimuth), elevation_deg=float(elevation),
                x_positions_m=x, y_positions_m=y,
            )
            estimate_az, estimate_el = estimate_aoa_from_positions(
                np.mean(iq, axis=(0, 1)), config, x, y
            )
            errors.append((estimate_az - azimuth, estimate_el - elevation))
    values = np.asarray(errors)
    return {
        "azimuth_rmse_deg": float(np.sqrt(np.mean(values[:, 0] ** 2))),
        "elevation_rmse_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))),
        "azimuth_max_abs_error_deg": float(np.max(np.abs(values[:, 0]))),
        "elevation_max_abs_error_deg": float(np.max(np.abs(values[:, 1]))),
    }


def run(*, results_root: Path, run_id: str, cad_csv: Path) -> Path:
    config = FmcwConfig()
    ideal_x, ideal_y = virtual_array_positions(config)
    cad_x, cad_y = _load_cad(cad_csv, config)
    summary = {"ideal_half_wavelength": _scan(config, ideal_x, ideal_y),
               "pcb_copper_centroid_approximation": _scan(config, cad_x, cad_y),
               "coordinate_source": str(cad_csv.resolve()),
               "coordinate_status": "cad_copper_centroid_not_electrical_phase_center",
               "scan_azimuth_deg": [-60.0, 60.0], "scan_elevation_deg": [-20.0, 20.0]}
    output = create_run_directory(results_root, producer="python",
                                  stage_id="v04_aoa_cfar_point_cloud", run_id=run_id)
    with h5py.File(output / "data" / "array_comparison.h5", "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-aoa-v0.4.5"
        handle.attrs["coordinate_status"] = summary["coordinate_status"]
        handle.create_dataset("/arrays/ideal_x_m", data=ideal_x)
        handle.create_dataset("/arrays/ideal_y_m", data=ideal_y)
        handle.create_dataset("/arrays/cad_x_m", data=cad_x)
        handle.create_dataset("/arrays/cad_y_m", data=cad_y)
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.5 理想阵列与 PCB-derived 阵列对比\n\n"
        "两种阵列使用同一 77 GHz、同一角度网格（方位 -60°～60°，俯仰 -20°～20°），"
        "只改变虚拟阵列坐标。理想阵列为半波长规则阵列；PCB 阵列为 PcbDoc 铜区几何中心求和近似。\n\n"
        f"- 理想阵列方位 RMSE：{summary['ideal_half_wavelength']['azimuth_rmse_deg']:.6f}°。\n"
        f"- PCB-derived 方位 RMSE：{summary['pcb_copper_centroid_approximation']['azimuth_rmse_deg']:.6f}°。\n"
        f"- 理想阵列俯仰 RMSE：{summary['ideal_half_wavelength']['elevation_rmse_deg']:.6f}°。\n"
        f"- PCB-derived 俯仰 RMSE：{summary['pcb_copper_centroid_approximation']['elevation_rmse_deg']:.6f}°。\n\n"
        "这个结果只比较坐标模型造成的角度恢复差异。PCB-derived 坐标是铜区几何中心，不是电气相位中心，"
        "因此不能直接称为 EVM 实测 AoA 精度。最终还需要馈电/相位中心建模、通道校准和角反射器验证。\n",
        encoding="utf-8")
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--cad-csv", type=Path, required=True)
    args = parser.parse_args()
    print(run(results_root=args.results_root.resolve(), run_id=args.run_id,
              cad_csv=args.cad_csv.resolve()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
