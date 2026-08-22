"""Combine V0.42 microfacet physics proxies with the V0.119 beam mask."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_42_physical_sea_microfacets import derive_frame, load_surface


def run(input_root: Path, output: Path, mounting_pitch_deg: float = 5.0, azimuth_half_deg: float = 30.0, beam_3db_half_deg: float = 3.0, beam_6db_half_deg: float = 5.0) -> dict:
    paths = sorted(input_root.glob("ss*_seed101.h5"))
    if not paths:
        raise FileNotFoundError(f"no sea HDF5 files under {input_root}")
    rows = []; summaries = []
    for path in paths:
        surface = load_surface(path); previous = None; case_rows = []
        for frame, time_s in enumerate(surface["time_s"]):
            facets, physics = derive_frame(surface, frame, previous, np.random.default_rng(4242 + frame), max_facets=48); previous = surface["height_m"][frame]
            for index, facet in enumerate(facets):
                facet = {key: facet[key] for key in facet.dtype.names} if getattr(facet, "dtype", None) is not None and facet.dtype.names else dict(facet)
                az = float(facet["azimuth_deg"]); elevation_down = -float(facet["elevation_deg"])
                mask_3 = abs(az) <= azimuth_half_deg and abs(elevation_down - mounting_pitch_deg) <= beam_3db_half_deg
                mask_6 = abs(az) <= azimuth_half_deg and abs(elevation_down - mounting_pitch_deg) <= beam_6db_half_deg
                row = {"case_id": surface["case_id"], "frame": frame, "time_s": float(time_s), "facet_index": index, "x_m": float(facet["x_m"]), "y_m": float(facet["y_m"]), "z_m": float(facet["z_m"]), "azimuth_deg": az, "elevation_down_deg": elevation_down, "slant_range_m": float(facet["slant_range_m"]), "incidence_cos": float(facet["incidence_cos"]), "scatter_proxy": float(facet["scatter_proxy"]), "radial_velocity_mps": float(facet["radial_velocity_mps"]), "doppler_hz": float(facet["doppler_hz"]), "mask_3db": int(mask_3), "mask_6db": int(mask_6), "masked_scatter_3db": float(facet["scatter_proxy"]) if mask_3 else 0.0, "masked_scatter_6db": float(facet["scatter_proxy"]) if mask_6 else 0.0}
                rows.append(row); case_rows.append(row)
        masked3 = [row for row in case_rows if row["mask_3db"]]
        masked6 = [row for row in case_rows if row["mask_6db"]]
        summaries.append({"case_id": surface["case_id"], "frames": len(surface["time_s"]), "candidate_facets": len(case_rows), "three_db_facets": len(masked3), "six_db_facets": len(masked6), "mean_three_db_scatter_proxy": float(np.mean([row["masked_scatter_3db"] for row in case_rows])), "mean_six_db_scatter_proxy": float(np.mean([row["masked_scatter_6db"] for row in case_rows])), "three_db_doppler_min_hz": float(np.min([row["doppler_hz"] for row in masked3])) if masked3 else None, "three_db_doppler_max_hz": float(np.max([row["doppler_hz"] for row in masked3])) if masked3 else None, "rms_three_db_radial_velocity_mps": float(np.sqrt(np.mean([row["radial_velocity_mps"] ** 2 for row in masked3]))) if masked3 else None})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "masked_microfacet_clutter.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    with (output / "case_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    result = {"status": "completed_masked_microfacet_clutter_proxy", "input_root": str(input_root.resolve()), "case_count": len(paths), "row_count": len(rows), "installation": {"mounting_pitch_deg": mounting_pitch_deg, "azimuth_half_deg": azimuth_half_deg, "beam_3db_half_deg": beam_3db_half_deg, "beam_6db_half_deg": beam_6db_half_deg}, "physics_source": "V0.42 normal/incidence/radial velocity/Doppler/scatter proxy", "point_cloud_semantics": "masked microfacet clutter proxy, not measured radar detections", "hardware_aoa_validated": False, "cases": summaries}
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        target = [row for row in rows if row["case_id"] == "ss3_upper" and row["frame"] == len([r for r in rows if r["case_id"] == "ss3_upper"]) // 96]
        if not target: target = [row for row in rows if row["case_id"] == "ss3_upper"]
        figure, axis = plt.subplots(figsize=(9, 5), dpi=160); outside = [row for row in target if not row["mask_3db"]]; inside = [row for row in target if row["mask_3db"]]
        axis.scatter([r["slant_range_m"] for r in outside], [r["doppler_hz"] for r in outside], s=10, alpha=.3, label="outside 3 dB")
        axis.scatter([r["slant_range_m"] for r in inside], [r["doppler_hz"] for r in inside], s=18, c=[r["scatter_proxy"] for r in inside], cmap="inferno", label="inside 3 dB")
        axis.set_xlabel("slant range (m)"); axis.set_ylabel("Doppler proxy (Hz)"); axis.set_title("ss3_upper masked microfacet range-Doppler proxy"); axis.grid(True, alpha=.25); axis.legend(); figure.tight_layout(); figure.savefig(output / "ss3_upper_masked_range_doppler_proxy.png"); plt.close(figure)
    except Exception:
        pass
    lines = ["# V0.4.120 波束掩膜海面微元杂波代理", "", "本阶段复用 V0.42 的海面法向、入射余弦、径向速度、Doppler 和 `incidence²/range²` 散射代理，再套用 V0.119 的 3 dB/6 dB 几何波束掩膜。", "", "## 字段含义", "", "`scatter_proxy` 是相对散射权重代理；`radial_velocity_mps` 和 `doppler_hz` 来自 V0.2 高度场时间差分；`masked_scatter_3db/6db` 只保留落入对应几何窗口的权重。", "", "## 结果边界", "", "这是微元海杂波候选的几何/物理代理，不是实测雷达点云或功率谱。没有真实海面散射系数、水平流速、极化、天线方向图、通道校准、噪声、CFAR 和 DCA1000 IQ。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
