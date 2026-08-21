"""Generate per-sea-state point-cloud figures and a Markdown comparison report."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import h5py
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


FIELDS = ("range_m", "velocity_mps", "azimuth_deg", "elevation_deg", "power_linear", "x_m", "y_m", "z_m")


def load(path: Path) -> dict[str, np.ndarray]:
    with h5py.File(path, "r") as handle:
        return {field: handle[f"/point_cloud/{field}"][...] for field in FIELDS}


def save_run_figures(label: str, points: dict[str, np.ndarray], directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    power_db = 10.0 * np.log10(np.maximum(points["power_linear"], 1e-12))
    fig, axis = plt.subplots(figsize=(7, 5))
    scatter = axis.scatter(points["range_m"], points["velocity_mps"], c=power_db, cmap="viridis")
    axis.set(xlabel="Range (m)", ylabel="Radial velocity (m/s)", title=f"{label}: Range-Doppler detections")
    fig.colorbar(scatter, ax=axis, label="Power (dB, relative)"); fig.tight_layout(); fig.savefig(directory / f"{label}_range_velocity.png", dpi=150); plt.close(fig)
    fig, axis = plt.subplots(figsize=(7, 5))
    axis.scatter(points["azimuth_deg"], points["elevation_deg"], c=power_db, cmap="plasma")
    axis.set(xlabel="Azimuth (deg)", ylabel="Elevation (deg)", title=f"{label}: AoA detections"); fig.tight_layout(); fig.savefig(directory / f"{label}_azimuth_elevation.png", dpi=150); plt.close(fig)
    fig = plt.figure(figsize=(7, 5)); axis = fig.add_subplot(111, projection="3d")
    axis.scatter(points["x_m"], points["y_m"], points["z_m"], c=power_db, cmap="viridis")
    axis.set(xlabel="x (m)", ylabel="y (m)", zlabel="z (m)", title=f"{label}: 3-D point cloud"); fig.tight_layout(); fig.savefig(directory / f"{label}_point_cloud_3d.png", dpi=150); plt.close(fig)


def run(inputs: list[tuple[str, Path]], output: Path) -> Path:
    output.mkdir(parents=True, exist_ok=True)
    figures = output / "figures"; figures.mkdir(exist_ok=True)
    summaries = []
    for label, path in inputs:
        points = load(path); save_run_figures(label, points, figures)
        count = len(points["range_m"])
        summaries.append({"label": label, "point_count": count,
                          "velocity_std_mps": float(np.std(points["velocity_mps"])) if count else 0.0,
                          "azimuth_std_deg": float(np.std(points["azimuth_deg"])) if count else 0.0,
                          "elevation_std_deg": float(np.std(points["elevation_deg"])) if count else 0.0})
    labels = [item["label"] for item in summaries]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    axes[0].bar(labels, [item["point_count"] for item in summaries]); axes[0].set_title("CFAR point count"); axes[0].set_ylabel("points")
    axes[1].bar(labels, [item["velocity_std_mps"] for item in summaries]); axes[1].set_title("Velocity spread"); axes[1].set_ylabel("std (m/s)")
    fig.tight_layout(); fig.savefig(figures / "sea_state_comparison.png", dpi=150); plt.close(fig)
    with (output / "sea_state_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0]) if summaries else ["label"]); writer.writeheader(); writer.writerows(summaries)
    lines = ["# 海况点云图文对比", "", "| 海况 | 点数 | 速度标准差 (m/s) | 方位标准差 (°) | 俯仰标准差 (°) |", "|---|---:|---:|---:|---:|"]
    for item in summaries:
        lines.append(f"| {item['label']} | {item['point_count']} | {item['velocity_std_mps']:.4f} | {item['azimuth_std_deg']:.4f} | {item['elevation_std_deg']:.4f} |")
        lines.append("")
        lines.append(f"- [{item['label']} 距离-速度图](figures/{item['label']}_range_velocity.png)")
        lines.append(f"- [{item['label']} 方位-俯仰图](figures/{item['label']}_azimuth_elevation.png)")
        lines.append(f"- [{item['label']} 三维点云图](figures/{item['label']}_point_cloud_3d.png)")
    lines += ["", "![海况对比](figures/sea_state_comparison.png)", "", "图中差异只反映当前 CFAR 和 AoA 仿真链路的输出；需要保持采集时长、CFG、Pfa、船体姿态和坐标系一致后才可比较。", ""]
    (output / "sea_state_comparison.md").write_text("\n".join(lines), encoding="utf-8")
    return output / "sea_state_comparison.md"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", action="append", nargs=2, metavar=("LABEL", "H5"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(run([(label, Path(path).resolve()) for label, path in args.input], args.output.resolve()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
