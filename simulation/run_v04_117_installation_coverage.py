"""Compute first-order sea-surface coverage from mounting geometry and beam edges."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def intersection_m(height_m: float, surface_height_m: float, elevation_deg: float) -> float | None:
    """Forward horizontal intersection for an elevation angle from horizontal."""
    if elevation_deg <= 0.0:
        return None
    tangent = math.tan(math.radians(elevation_deg))
    return (height_m - surface_height_m) / tangent if height_m > surface_height_m else 0.0


def run(output: Path, height_m: float = 1.0, beam_3db_half_deg: float = 3.0, beam_6db_half_deg: float = 5.0) -> dict:
    pitches = [0.0, 3.0, 5.0, 8.0, 10.0]
    sea_states = [("ss0_flat", 0.0), ("ss1_rippled", 0.05), ("ss2_normal", 0.30), ("ss3_nominal", 0.85), ("ss3_upper", 1.00)]
    rows = []
    for case_id, hs in sea_states:
        for surface_label, surface_h in (("mean", 0.0), ("crest_proxy", hs / 2.0), ("trough_proxy", -hs / 2.0)):
            for pitch in pitches:
                row = {"case_id": case_id, "target_hs_m": hs, "surface_proxy": surface_label, "surface_height_m": surface_h, "mounting_pitch_deg": pitch, "boresight_m": intersection_m(height_m, surface_h, pitch), "three_db_near_m": intersection_m(height_m, surface_h, pitch + beam_3db_half_deg), "three_db_far_m": intersection_m(height_m, surface_h, pitch - beam_3db_half_deg), "six_db_near_m": intersection_m(height_m, surface_h, pitch + beam_6db_half_deg), "six_db_far_m": intersection_m(height_m, surface_h, pitch - beam_6db_half_deg)}
                rows.append(row)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "installation_coverage.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summary = {"status": "completed_first_order_installation_coverage", "height_m": height_m, "beam_3db_half_deg": beam_3db_half_deg, "beam_6db_half_deg": beam_6db_half_deg, "mounting_pitch_deg": pitches, "sea_states": [{"case_id": case, "target_hs_m": hs} for case, hs in sea_states], "model_boundary": "flat mean surface plus crest/trough height proxies; no antenna pattern, shadowing, vessel motion or measured sea surface", "hardware_aoa_validated": False}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        figure, axis = plt.subplots(figsize=(8, 4.5), constrained_layout=True)
        for label in ("three_db_near_m", "three_db_far_m", "boresight_m"):
            part = [row for row in rows if row["case_id"] == "ss3_upper" and row["surface_proxy"] == "mean"]
            axis.plot([row["mounting_pitch_deg"] for row in part], [row[label] if row[label] is not None else float("nan") for row in part], "o-", label=label)
        axis.set_xlabel("mounting pitch downward (deg)"); axis.set_ylabel("horizontal intersection (m)"); axis.set_title("1 m radar first-order sea coverage"); axis.grid(True, alpha=0.25); axis.legend(); figure.savefig(output / "installation_coverage.png", dpi=160); plt.close(figure)
    except Exception:
        pass
    lines = ["# V0.4.117 安装俯仰与海面覆盖几何", "", "本阶段计算的是从雷达到平均海面/波峰波谷代理面的几何交点，不是天线方向图的完整探测性能。正俯仰表示波束向下。", "", "## 1 m 安装高度、5° 向下安装的平静平均海面示例", "", f"波束中心交点：`{intersection_m(height_m, 0.0, 5.0):.3f} m`；3 dB 近边缘（8°）：`{intersection_m(height_m, 0.0, 8.0):.3f} m`；3 dB 远边缘（2°）：`{intersection_m(height_m, 0.0, 2.0):.3f} m`；6 dB 近边缘（10°）：`{intersection_m(height_m, 0.0, 10.0):.3f} m`；6 dB 远边缘（0°）：无有限交点。", "", "## 如何理解", "", "俯仰安装角越大，波束整体压向近海面：近端覆盖距离缩短，远端覆盖边界也更近；俯仰为 0° 时，水平波束不会与无限平面海面相交，实际覆盖由波束下倾、海浪、地球曲率和天线方向图共同决定。3 dB/6 dB 是相对峰值功率下降 3/6 dB 的边缘，不能当成芯片可测角度极限。", "", "## 证据边界", "", "波峰/波谷代理使用 ±Hs/2，不等于有效波高的最大瞬时波高；没有纳入船体纵摇、横摇、遮挡、海面反射系数、真实方向图和 CFAR。因此结果只用于安装角度初选和盲区几何解释，不能宣称实船探测距离。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
