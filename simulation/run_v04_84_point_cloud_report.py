"""Create sea-state point-cloud figures and report tables from V0.4.83."""

from __future__ import annotations

import argparse, csv, json
from collections import defaultdict
from pathlib import Path
import numpy as np


def _read(path):
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def run(input_dir: Path, output: Path) -> dict:
    points = _read(input_dir / "detector_point_cloud.csv")
    summary = _read(input_dir / "detector_point_cloud_summary.csv")
    output.mkdir(parents=True, exist_ok=True); figures = output / "figures"; figures.mkdir(exist_ok=True)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    detectors = sorted({row["detector"] for row in points})
    colors = {name: plt.cm.tab10(index % 10) for index, name in enumerate(detectors)}
    grouped = defaultdict(list)
    for row in points: grouped[row["case_id"]].append(row)
    for case_id, rows in grouped.items():
        fig = plt.figure(figsize=(17, 5), constrained_layout=True)
        ax = fig.add_subplot(131, projection="3d")
        for detector in detectors:
            data = [r for r in rows if r["detector"] == detector]
            if not data: continue
            ax.scatter([float(r["x_m"]) for r in data], [float(r["y_m"]) for r in data], [float(r["z_m"]) for r in data], s=5, alpha=.45, color=colors[detector], label=detector)
        ax.set_title(f"{case_id} XYZ point cloud"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
        ax = fig.add_subplot(132)
        for detector in detectors:
            data=[r for r in rows if r["detector"]==detector]
            if data: ax.scatter([float(r["range_m"]) for r in data],[float(r["velocity_mps"]) for r in data],s=5,alpha=.45,color=colors[detector],label=detector)
        ax.set_title("Range-velocity"); ax.set_xlabel("range (m)"); ax.set_ylabel("velocity (m/s)"); ax.grid(alpha=.25)
        ax = fig.add_subplot(133)
        for detector in detectors:
            data=[r for r in rows if r["detector"]==detector]
            if data: ax.scatter([float(r["azimuth_deg"]) for r in data],[float(r["elevation_deg"]) for r in data],s=5,alpha=.45,color=colors[detector],label=detector)
        ax.set_title("Azimuth-elevation"); ax.set_xlabel("azimuth (deg)"); ax.set_ylabel("elevation (deg)"); ax.grid(alpha=.25)
        fig.legend(loc="upper center", ncol=4, fontsize=7); fig.savefig(figures / f"{case_id}_point_cloud.png", dpi=150); plt.close(fig)
    aggregate = defaultdict(lambda: {"detection_count":0,"stored_point_count":0,"points":[]})
    for row in summary:
        key=(row["case_id"],row["detector"]); aggregate[key]["detection_count"] += int(row["detection_count"]); aggregate[key]["stored_point_count"] += int(row["stored_point_count"])
    for row in points: aggregate[(row["case_id"],row["detector"])] ["points"].append(row)
    stats=[]
    for (case,detector), value in sorted(aggregate.items()):
        p=value["points"]
        def stat(key):
            vals=np.asarray([float(r[key]) for r in p], dtype=float); vals=vals[np.isfinite(vals)]; return float(np.mean(vals)) if len(vals) else float("nan"), float(np.std(vals)) if len(vals) else float("nan")
        ma,sa=stat("azimuth_deg"); me,se=stat("elevation_deg")
        stats.append({"case_id":case,"detector":detector,"detection_count":value["detection_count"],"stored_point_count":value["stored_point_count"],"point_mean_azimuth_deg":ma,"point_std_azimuth_deg":sa,"point_mean_elevation_deg":me,"point_std_elevation_deg":se,"nonfinite_azimuth_count":sum(not np.isfinite(float(r["azimuth_deg"])) for r in p)})
    with (output/"sea_state_detector_statistics.csv").open("w",newline="",encoding="utf-8") as handle:
        writer=csv.DictWriter(handle,fieldnames=list(stats[0])); writer.writeheader(); writer.writerows(stats)
    fig, ax=plt.subplots(figsize=(10,5), constrained_layout=True)
    cases=sorted({r["case_id"] for r in stats}); width=.8/max(len(detectors),1); indices=np.arange(len(cases))
    for index,detector in enumerate(detectors):
        vals=[next((r["detection_count"] for r in stats if r["case_id"]==case and r["detector"]==detector),0) for case in cases]
        ax.bar(indices+index*width,vals,width,label=detector,color=colors[detector])
    ax.set_xticks(indices+width*(len(detectors)-1)/2,cases); ax.set_ylabel("detection count"); ax.set_title("Detector retention by sea state"); ax.legend(fontsize=7); ax.grid(axis="y",alpha=.25); fig.savefig(figures/"detector_retention_by_sea_state.png",dpi=150); plt.close(fig)
    result={"status":"completed_point_cloud_report", "case_count":len(cases), "detector_count":len(detectors), "point_count":len(points), "figure_count":len(list(figures.glob("*.png"))), "hardware_validated":False}
    (output/"summary.json").write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding="utf-8")
    (output/"output_analysis.md").write_text("# V0.4.84 海况级海杂波点云图文分析\n\n"
        "本阶段读取 V0.4.83 的逐检测点 CSV，按海况生成 XYZ 三维点云、距离-速度图和方位-俯仰图；同时生成检测器保留量柱状图与海况/检测器统计表。\n\n"
        "## 图形如何阅读\n\n"
        "- XYZ 图：每个点是一个 CFAR/能量门保留单元，坐标由距离、方位和俯仰换算；不同颜色表示不同检测器。\n- 距离-速度图：观察海杂波点集中在哪些距离和径向速度区间。\n- 方位-俯仰图：观察点云角度偏置、离散程度和非有限角度。\n- 柱状图：比较相同海况下不同检测器保留的距离-多普勒单元数量。\n\n"
        "## 汇报时的正确表述\n\n"
        "当前图形证明的是合成海况和候选 AoA 管线下不同检测规则/海况的相对差异，不是实船海杂波虚警率或实测雷达性能。阵列坐标、通道顺序、TI 校准和方向图仍需实测验证。\n",encoding="utf-8")
    return result


def main():
    p=argparse.ArgumentParser(); p.add_argument("--input-dir",type=Path,required=True); p.add_argument("--output",type=Path,required=True); a=p.parse_args(); print(json.dumps(run(a.input_dir.resolve(),a.output.resolve()),indent=2,ensure_ascii=False))


if __name__ == "__main__": main()
