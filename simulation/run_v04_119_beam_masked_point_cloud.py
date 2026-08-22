"""Export 3-D dynamic sea points with first-order beam masks."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np


def _load(path: Path) -> dict:
    with h5py.File(path, "r") as handle:
        return {"case_id": str(np.asarray(handle.attrs["case_id"]).reshape(-1)[0]), "time_s": handle["/axes/time_s"][...], "x_m": handle["/axes/x_m"][...], "y_m": handle["/axes/y_m"][...], "height_m": handle["/truth/height_m"][...], "radar_height_m": float(np.asarray(handle["/installation/height_m"][...]).reshape(-1)[0])}


def run(input_root: Path, output: Path, frame_index: int | None = None, mounting_pitch_deg: float = 5.0, azimuth_half_deg: float = 30.0, beam_3db_half_deg: float = 3.0, beam_6db_half_deg: float = 5.0) -> dict:
    paths = sorted(input_root.glob("ss*_seed101.h5"))
    if not paths:
        raise FileNotFoundError(f"no sea HDF5 files under {input_root}")
    rows = []
    selected = []
    for path in paths:
        surface = _load(path)
        frame = len(surface["time_s"]) // 2 if frame_index is None else min(max(frame_index, 0), len(surface["time_s"]) - 1)
        selected.append({"case_id": surface["case_id"], "frame": frame, "time_s": float(surface["time_s"][frame])})
        xx, yy = np.meshgrid(surface["x_m"], surface["y_m"], indexing="xy")
        z = surface["height_m"][frame]
        horizontal = np.hypot(xx, yy)
        slant = np.sqrt(horizontal ** 2 + (surface["radar_height_m"] - z) ** 2)
        azimuth = np.degrees(np.arctan2(xx, yy))
        elevation = np.degrees(np.arctan2(surface["radar_height_m"] - z, horizontal))
        valid = np.isfinite(z) & (horizontal > 0)
        mask_3 = valid & (np.abs(azimuth) <= azimuth_half_deg) & (np.abs(elevation - mounting_pitch_deg) <= beam_3db_half_deg)
        mask_6 = valid & (np.abs(azimuth) <= azimuth_half_deg) & (np.abs(elevation - mounting_pitch_deg) <= beam_6db_half_deg)
        for iy, ix in zip(*np.where(valid)):
            rows.append({"case_id": surface["case_id"], "frame": frame, "time_s": float(surface["time_s"][frame]), "x_m": float(xx[iy, ix]), "y_m": float(yy[iy, ix]), "z_m": float(z[iy, ix]), "slant_range_m": float(slant[iy, ix]), "azimuth_deg": float(azimuth[iy, ix]), "elevation_deg": float(elevation[iy, ix]), "mask_3db": int(mask_3[iy, ix]), "mask_6db": int(mask_6[iy, ix])})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "beam_masked_point_cloud.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summary_cases = []
    for item in selected:
        part = [row for row in rows if row["case_id"] == item["case_id"]]
        summary_cases.append({"case_id": item["case_id"], "frame": item["frame"], "time_s": item["time_s"], "point_count": len(part), "three_db_points": sum(row["mask_3db"] for row in part), "six_db_points": sum(row["mask_6db"] for row in part)})
    result = {"status": "completed_beam_masked_3d_point_cloud", "input_root": str(input_root.resolve()), "output": str(output.resolve()), "selected_cases": summary_cases, "installation": {"mounting_pitch_deg": mounting_pitch_deg, "azimuth_half_deg": azimuth_half_deg, "beam_3db_half_deg": beam_3db_half_deg, "beam_6db_half_deg": beam_6db_half_deg}, "point_cloud_semantics": "surface geometry points with beam masks; no scattering amplitude or radar detection", "hardware_aoa_validated": False}
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        target = next(item for item in selected if item["case_id"] == "ss3_upper")
        part = [row for row in rows if row["case_id"] == target["case_id"]]
        figure = plt.figure(figsize=(10, 7), dpi=160); axis = figure.add_subplot(111, projection="3d")
        outside = [row for row in part if not row["mask_3db"]]; inside = [row for row in part if row["mask_3db"]]
        axis.scatter([r["x_m"] for r in outside], [r["y_m"] for r in outside], [r["z_m"] for r in outside], s=2, alpha=0.12, color="steelblue", label="outside 3 dB")
        axis.scatter([r["x_m"] for r in inside], [r["y_m"] for r in inside], [r["z_m"] for r in inside], s=7, alpha=0.65, color="crimson", label="inside 3 dB")
        axis.scatter([0], [0], [1], marker="^", s=60, color="black", label="radar (1 m)"); axis.set_xlabel("x lateral (m)"); axis.set_ylabel("y forward (m)"); axis.set_zlabel("sea height (m)"); axis.set_title("ss3_upper dynamic sea surface with 3 dB geometric mask"); axis.legend(); figure.savefig(output / "ss3_upper_beam_masked_3d.png", bbox_inches="tight"); plt.close(figure)
    except Exception:
        pass
    lines = ["# V0.4.119 波束掩膜海面三维点云", "", "本阶段把 V0.2 动态海面高度、网格坐标、斜距、方位角、俯仰角和 3 dB/6 dB 几何波束掩膜组合为 CSV 点云，并为 ss3_upper 中间帧生成 3D 图。", "", "## 点云字段", "", "`x_m/y_m/z_m` 是海面网格坐标；`slant_range_m` 是雷达到微元的几何斜距；`azimuth_deg/elevation_deg` 是相对雷达的几何角；`mask_3db/mask_6db` 是是否落入相应几何波束窗口。", "", "## 重要边界", "", "这些是海面几何点，不是雷达检测点云：没有散射强度、相位、噪声、CFAR、真实方向图、遮挡或校准。红色点只表示落入 3 dB 几何角窗口，不能表示一定能被检测。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--frame", type=int); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve(), args.frame), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
