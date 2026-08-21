"""Screen PCB axis semantics against the known-angle CFG AoA fixture."""

from __future__ import annotations

import argparse, csv, json
from pathlib import Path
import numpy as np

from simulation.run_v04_coordinate_transform_scan import _load_cfg_geometry, scan
from simulation.v03 import FmcwConfig


def _load_cad(path: Path, config: FmcwConfig):
    with path.open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    x = np.zeros((4, 4)); y = np.zeros((4, 4))
    for row in rows:
        index = int(row["virtual_channel"]); tx, rx = divmod(index, 4)
        x[rx, tx] = float(row["x_lambda"]) * config.wavelength_m
        y[rx, tx] = float(row["y_lambda"]) * config.wavelength_m
    return x, y


def candidates(x, y, target_x, target_y):
    """Return physical sign/swap candidates and separately labeled extent-normalized candidates."""
    cx, cy = (np.min(x) + np.max(x))/2, (np.min(y) + np.max(y))/2
    x0, y0 = x-cx, y-cy
    raw = {"raw": (x0, y0), "x_mirror": (-x0, y0), "y_mirror": (x0, -y0), "rotate_180": (-x0, -y0),
           "swap_xy": (y0, x0), "swap_xy_x_mirror": (-y0, x0), "swap_xy_y_mirror": (y0, -x0), "swap_xy_rotate_180": (-y0, -x0)}
    tx_span, ty_span = np.ptp(target_x), np.ptp(target_y)
    result = dict(raw)
    for name, (xx, yy) in raw.items():
        sx, sy = tx_span/max(np.ptp(xx), 1e-12), ty_span/max(np.ptp(yy), 1e-12)
        result["normalized_"+name] = (xx*sx, yy*sy)
    return result


def run(cad_csv: Path, cfg_csv: Path, output: Path) -> dict:
    config = FmcwConfig(); cfg_x, cfg_y = _load_cfg_geometry(cfg_csv, config); cad_x, cad_y = _load_cad(cad_csv, config)
    rows = []
    for name, (x, y) in candidates(cad_x, cad_y, cfg_x, cfg_y).items():
        metric = scan(config, x, y, cfg_x, cfg_y)
        metric["candidate"] = name
        metric["normalization"] = "extent_match_for_semantic_screen_only" if name.startswith("normalized_") else "none"
        rows.append(metric)
    output.mkdir(parents=True, exist_ok=True)
    with (output/"coordinate_transform_validation.csv").open("w", newline="", encoding="utf-8") as handle:
        writer=csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    best=min(rows, key=lambda r:r["azimuth_rmse_deg"]+r["elevation_rmse_deg"])
    summary={"status":"completed_coordinate_transform_validation", "candidate_count":len(rows), "best_candidate":best["candidate"], "best_azimuth_rmse_deg":best["azimuth_rmse_deg"], "best_elevation_rmse_deg":best["elevation_rmse_deg"], "physical_pose_confirmed":False, "phase_center_confirmed":False}
    (output/"summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output/"output_analysis.md").write_text("# V0.4.75 PCB 坐标变换与已知角度验证\n\n"
        "本阶段使用 V0.4.33 的已知角度合成夹具，在 -60°～60° 方位、-20°～20° 俯仰网格上筛查 PCB x/y 轴、镜像和交换候选。\n\n"
        f"综合 RMSE 最小候选为 `{best['candidate']}`，方位 RMSE={best['azimuth_rmse_deg']:.3f}°，俯仰 RMSE={best['elevation_rmse_deg']:.3f}°。\n\n"
        "`normalized_*` 只把 PCB 坐标的两个轴按范围缩放到 CFG 阵列范围，用于分离‘轴方向语义’与‘物理孔径尺寸’影响，不能作为真实硬件尺寸。当前没有机械安装测量、相位中心或实测角反射器数据，因此不选择任何候选作为正式雷达坐标。\n", encoding="utf-8")
    return summary


def main():
    p=argparse.ArgumentParser(); p.add_argument("--cad-csv", type=Path, required=True); p.add_argument("--cfg-csv", type=Path, required=True); p.add_argument("--output", type=Path, required=True); a=p.parse_args(); print(json.dumps(run(a.cad_csv.resolve(), a.cfg_csv.resolve(), a.output.resolve()), indent=2, ensure_ascii=False))


if __name__ == "__main__": main()
