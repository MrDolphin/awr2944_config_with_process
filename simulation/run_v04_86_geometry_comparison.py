"""Compare the ideal V0.4 virtual array with the PCB-derived candidate geometry."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.v03 import FmcwConfig
from simulation.v04 import generate_aoa_iq_with_positions, virtual_array_positions


def _load_candidate(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != 16:
        raise ValueError("virtual array candidate must contain 16 rows")
    rows.sort(key=lambda row: int(row["virtual_channel"]))
    x = np.asarray([float(row["x_mm"]) for row in rows], dtype=float).reshape(4, 4) / 1000.0
    y = np.asarray([float(row["y_mm"]) for row in rows], dtype=float).reshape(4, 4) / 1000.0
    return x, y


def _steering(config: FmcwConfig, x: np.ndarray, y: np.ndarray, az: np.ndarray, el: np.ndarray) -> np.ndarray:
    azr, elr = np.meshgrid(np.deg2rad(az), np.deg2rad(el), indexing="ij")
    phase = 2.0 * np.pi / config.wavelength_m * (
        x.ravel()[None, None, :] * np.sin(azr)[..., None] * np.cos(elr)[..., None]
        + y.ravel()[None, None, :] * np.sin(elr)[..., None]
    )
    return np.exp(1j * phase).reshape(-1, 16) / 4.0


def _beam(config: FmcwConfig, x: np.ndarray, y: np.ndarray, az: np.ndarray, el: np.ndarray) -> np.ndarray:
    steering = _steering(config, x, y, az, el)
    broadside = np.ones(16, dtype=complex) / 4.0
    return np.abs(steering @ broadside.conj()) ** 2


def _estimate(config: FmcwConfig, channel: np.ndarray, x: np.ndarray, y: np.ndarray, az: np.ndarray, el: np.ndarray) -> tuple[float, float, float]:
    steering = _steering(config, x, y, az, el)
    scores = np.abs(steering.conj() @ channel.ravel() / max(np.linalg.norm(channel), 1e-12)) ** 2
    index = int(np.argmax(scores))
    return float(az[index // len(el)]), float(el[index % len(el)]), float(scores[index])


def _width(grid: np.ndarray, response: np.ndarray) -> float:
    peak = float(np.max(response))
    selected = grid[response >= peak / 2.0]
    return float(selected.max() - selected.min()) if len(selected) else float("nan")


def run(candidate_csv: Path, output: Path) -> dict:
    config = FmcwConfig()
    candidate = _load_candidate(candidate_csv)
    ideal = virtual_array_positions(config)
    az = np.arange(-60.0, 60.01, 1.0)
    el = np.arange(-20.0, 20.01, 1.0)
    models = {"ideal_v04": ideal, "pcb_candidate": candidate}
    output.mkdir(parents=True, exist_ok=True)

    beam_rows = []
    beam_maps = {}
    for name, positions in models.items():
        response = _beam(config, positions[0], positions[1], az, el)
        beam_maps[name] = response.reshape(len(az), len(el))
        for i, a in enumerate(az):
            for j, e in enumerate(el):
                beam_rows.append({"model": name, "azimuth_deg": a, "elevation_deg": e, "power_linear": float(beam_maps[name][i, j])})
    with (output / "beam_response.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(beam_rows[0]))
        writer.writeheader(); writer.writerows(beam_rows)

    truth_az = np.arange(-40.0, 40.1, 20.0)
    truth_el = np.arange(-10.0, 10.1, 10.0)
    solver_az = np.arange(-60.0, 60.01, 1.0)
    solver_el = np.arange(-20.0, 20.01, 1.0)
    errors = []
    actual = candidate
    for e in truth_el:
        for a in truth_az:
            iq = generate_aoa_iq_with_positions(config, slant_range_m=30.0, radial_velocity_mps=0.0, azimuth_deg=float(a), elevation_deg=float(e), x_positions_m=actual[0], y_positions_m=actual[1])
            channel = np.mean(iq, axis=(0, 1))
            for assumed_name, assumed in models.items():
                ea, ee, score = _estimate(config, channel, assumed[0], assumed[1], solver_az, solver_el)
                errors.append({"assumed_model": assumed_name, "truth_azimuth_deg": a, "truth_elevation_deg": e, "estimated_azimuth_deg": ea, "estimated_elevation_deg": ee, "azimuth_error_deg": ea - a, "elevation_error_deg": ee - e, "score": score})
    with (output / "known_angle_errors.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(errors[0])); writer.writeheader(); writer.writerows(errors)

    metrics = []
    for name in models:
        values = np.asarray([[r["azimuth_error_deg"], r["elevation_error_deg"]] for r in errors if r["assumed_model"] == name], dtype=float)
        metrics.append({"model": name, "azimuth_rmse_deg": float(np.sqrt(np.mean(values[:, 0] ** 2))), "elevation_rmse_deg": float(np.sqrt(np.mean(values[:, 1] ** 2))), "combined_rmse_deg": float(np.sqrt(np.mean(values ** 2)))})
    candidate_x = candidate[0].ravel(); candidate_y = candidate[1].ravel()
    summary = {
        "status": "completed_pcb_candidate_geometry_comparison",
        "candidate_csv": str(candidate_csv.resolve()), "candidate_virtual_channels": 16,
        "wavelength_mm": config.wavelength_m * 1000.0,
        "candidate_aperture_x_mm": float(candidate_x.max() - candidate_x.min()) * 1000.0,
        "candidate_aperture_y_mm": float(candidate_y.max() - candidate_y.min()) * 1000.0,
        "beam_3db_width_az_deg": {name: _width(az, beam_maps[name][:, len(el) // 2]) for name in models},
        "beam_3db_width_el_deg": {name: _width(el, beam_maps[name][len(az) // 2, :]) for name in models},
        "known_angle_metrics": metrics,
        "hardware_aoa_validated": False,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 4), constrained_layout=True)
    for ax, (name, values) in zip(axes, beam_maps.items()):
        image = 10.0 * np.log10(np.maximum(values.T, 1e-8))
        image = np.maximum(image, -35.0)
        mesh = ax.imshow(image, origin="lower", aspect="auto", extent=[az.min(), az.max(), el.min(), el.max()], vmin=-35, vmax=0, cmap="viridis")
        ax.set_title(name); ax.set_xlabel("azimuth (deg)"); ax.set_ylabel("elevation (deg)")
        ax.contour(az, el, image, levels=[-3], colors="white", linewidths=.7)
    fig.colorbar(mesh, ax=axes, label="relative power (dB)"); fig.savefig(output / "beam_comparison.png", dpi=160); plt.close(fig)
    fig, ax = plt.subplots(figsize=(7, 4), constrained_layout=True)
    for name in models:
        subset = [r for r in errors if r["assumed_model"] == name]
        ax.scatter([r["truth_azimuth_deg"] for r in subset], [r["azimuth_error_deg"] for r in subset], label=name, s=18)
    ax.axhline(0, color="black", linewidth=.7); ax.set_xlabel("truth azimuth (deg)"); ax.set_ylabel("azimuth error (deg)"); ax.grid(alpha=.25); ax.legend(); fig.savefig(output / "known_angle_azimuth_error.png", dpi=160); plt.close(fig)

    lines = ["# V0.4.86 PCB 候选阵列与理想阵列对比", "", "本阶段只改变虚拟阵列坐标，其余 77 GHz、角度网格、合成已知角 IQ 和导向矢量搜索保持一致。", "", "## 结果", "", "| 模型 | 方位 3 dB 宽度(°) | 俯仰 3 dB 宽度(°) | 方位 RMSE(°) | 俯仰 RMSE(°) | 综合 RMSE(°) |", "|---|---:|---:|---:|---:|---:|"]
    metric_map = {row["model"]: row for row in metrics}
    for name in models:
        m = metric_map[name]
        lines.append(f"| {name} | {summary['beam_3db_width_az_deg'][name]:.1f} | {summary['beam_3db_width_el_deg'][name]:.1f} | {m['azimuth_rmse_deg']:.3f} | {m['elevation_rmse_deg']:.3f} | {m['combined_rmse_deg']:.3f} |")
    lines += ["", "## 图形如何阅读", "", "- `beam_comparison.png`：颜色越亮表示相对阵列增益越高；白色曲线是 -3 dB 等值线。候选阵列的横向孔径和 y 向孔径会分别影响方位和俯仰主瓣宽度。", "- `known_angle_azimuth_error.png`：每个点是一个已知角合成 IQ 测试；纵轴是估计方位减去真实方位。", "", "## 阶段结论", "", "PCB 候选几何已经进入 AoA 导向矢量搜索。候选几何的 0° RMSE 只是‘用同一候选几何生成并估计’的自洽性检查，不能解释为实机精度；理想模型的误差表示用 V0.4 理想阵列去解释 PCB 候选几何时产生的模型失配。", "", "候选阵列横向孔径约 21.37 mm，方位 -3 dB 主瓣约 8°；其 y 向孔径约 3.14 mm，俯仰 -3 dB 主瓣在当前 ±20°扫描边界内约 40°，说明该候选几何的方位分辨率明显强于俯仰分辨率。这个数值仍受扫描边界、铜区质心近似和虚拟阵列相位中心假设影响。", "", "PCB Region 几何中心仍不等于电气相位中心，通道校准、板面坐标到雷达坐标变换、方向图和互耦仍待实测。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.candidate_csv.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
