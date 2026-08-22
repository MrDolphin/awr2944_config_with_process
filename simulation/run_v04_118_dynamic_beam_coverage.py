"""Map V0.2 dynamic sea-surface facets into first-order beam coverage."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np


def load_surface(path: Path) -> dict:
    with h5py.File(path, "r") as handle:
        return {"case_id": str(np.asarray(handle.attrs["case_id"]).reshape(-1)[0]), "time_s": handle["/axes/time_s"][...], "x_m": handle["/axes/x_m"][...], "y_m": handle["/axes/y_m"][...], "height_m": handle["/truth/height_m"][...], "radar_height_m": float(np.asarray(handle["/installation/height_m"][...]).reshape(-1)[0]), "wind_direction_deg": float(np.asarray(handle["/case/wind_direction_deg"][...]).reshape(-1)[0])}


def run(input_root: Path, output: Path, mounting_pitch_deg: float = 5.0, azimuth_half_deg: float = 30.0, beam_3db_half_deg: float = 3.0, beam_6db_half_deg: float = 5.0) -> dict:
    paths = sorted(input_root.glob("ss*_seed101.h5"))
    if not paths:
        raise FileNotFoundError(f"no sea HDF5 files under {input_root}")
    x, y = np.meshgrid(None, None) if False else (None, None)
    rows = []
    for path in paths:
        surface = load_surface(path)
        xx, yy = np.meshgrid(surface["x_m"], surface["y_m"], indexing="xy")
        horizontal = np.hypot(xx, yy)
        azimuth = np.degrees(np.arctan2(xx, yy))
        for frame, time_s in enumerate(surface["time_s"]):
            z = surface["height_m"][frame]
            elevation = np.degrees(np.arctan2(surface["radar_height_m"] - z, horizontal))
            finite = np.isfinite(elevation) & (horizontal > 0)
            in_az = np.abs(azimuth) <= azimuth_half_deg
            in_3 = finite & in_az & (np.abs(elevation - mounting_pitch_deg) <= beam_3db_half_deg)
            in_6 = finite & in_az & (np.abs(elevation - mounting_pitch_deg) <= beam_6db_half_deg)
            row = {"case_id": surface["case_id"], "frame": frame, "time_s": float(time_s), "target_hs_m": float(np.nanmax(z) - np.nanmin(z)) if surface["case_id"] == "unknown" else None, "wind_direction_deg": surface["wind_direction_deg"], "grid_points": int(finite.sum()), "three_db_points": int(in_3.sum()), "six_db_points": int(in_6.sum()), "three_db_fraction": float(in_3.sum() / max(finite.sum(), 1)), "six_db_fraction": float(in_6.sum() / max(finite.sum(), 1)), "three_db_near_m": float(np.min(horizontal[in_3])) if np.any(in_3) else None, "three_db_far_m": float(np.max(horizontal[in_3])) if np.any(in_3) else None, "six_db_near_m": float(np.min(horizontal[in_6])) if np.any(in_6) else None, "six_db_far_m": float(np.max(horizontal[in_6])) if np.any(in_6) else None, "surface_min_m": float(np.nanmin(z)), "surface_max_m": float(np.nanmax(z))}
            rows.append(row)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "dynamic_beam_coverage_frames.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summaries = []
    for case_id in sorted({row["case_id"] for row in rows}):
        part = [row for row in rows if row["case_id"] == case_id]
        summaries.append({"case_id": case_id, "frames": len(part), "mean_three_db_fraction": float(np.mean([row["three_db_fraction"] for row in part])), "mean_six_db_fraction": float(np.mean([row["six_db_fraction"] for row in part])), "three_db_near_min_m": float(np.nanmin([row["three_db_near_m"] if row["three_db_near_m"] is not None else np.nan for row in part])), "three_db_far_max_m": float(np.nanmax([row["three_db_far_m"] if row["three_db_far_m"] is not None else np.nan for row in part]))})
    with (output / "dynamic_beam_coverage_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    result = {"status": "completed_dynamic_beam_coverage", "input_root": str(input_root.resolve()), "case_count": len(paths), "frame_count": len(rows), "installation": {"height_m": 1.0, "mounting_pitch_deg": mounting_pitch_deg, "azimuth_half_deg": azimuth_half_deg, "beam_3db_half_deg": beam_3db_half_deg, "beam_6db_half_deg": beam_6db_half_deg}, "cases": summaries, "model_boundary": "height-only geometric beam mask; no measured antenna pattern, reflectivity, shadowing, vessel motion or CFAR", "hardware_aoa_validated": False}
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        figure, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, constrained_layout=True)
        for case_id in sorted({row["case_id"] for row in rows}):
            part = [row for row in rows if row["case_id"] == case_id]
            axes[0].plot([row["frame"] for row in part], [row["three_db_fraction"] for row in part], label=case_id)
            axes[1].plot([row["frame"] for row in part], [row["three_db_near_m"] if row["three_db_near_m"] is not None else np.nan for row in part], label=case_id)
        axes[0].set_ylabel("3 dB covered fraction"); axes[1].set_ylabel("3 dB near range (m)"); axes[1].set_xlabel("frame"); axes[0].legend(ncol=3, fontsize=8); [axis.grid(True, alpha=0.25) for axis in axes]; figure.savefig(output / "dynamic_beam_coverage.png", dpi=160); plt.close(figure)
    except Exception:
        pass
    lines = ["# V0.4.118 动态海面波束覆盖统计", "", "本阶段将 V0.2 的实际动态海面高度网格映射到安装高度 1 m、向下俯仰 5°、方位 ±30° 的几何波束窗口，统计每帧海面微元落入 3 dB/6 dB 俯仰范围的比例。", "", "## 指标含义", "", "- `three_db_fraction`/`six_db_fraction`：当前网格中落入对应几何波束窗口的海面点比例，不是功率比例。", "- `near_m`/`far_m`：落入窗口的海面网格水平距离最小/最大值，不是探测距离。", "- 波浪使微元俯仰角随时间变化，因此覆盖比例和近端边界会抖动。", "", "## 证据边界", "", "没有使用真实天线方向图、海面反射系数、复数回波、船体姿态、遮挡或 CFAR；因此结果只用于解释安装角和海面几何关系，不能作为实船检测概率或海杂波功率结论。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
