"""Compare AoA point-cloud statistics from abstract and CAD-derived arrays."""

from __future__ import annotations

import argparse, csv, json
from pathlib import Path

KEY = ("case_id", "scenario_id", "detector")


def _read(path):
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def compare(abstract_summary: Path, cad_summary: Path, output: Path) -> dict:
    left = {tuple(row[k] for k in KEY): row for row in _read(abstract_summary)}
    right = {tuple(row[k] for k in KEY): row for row in _read(cad_summary)}
    rows = []
    for key in sorted(left):
        a, c = left[key], right[key]
        item = {k: a[k] for k in KEY}
        for field in ("detection_count", "stored_point_count", "mean_azimuth_deg", "std_azimuth_deg", "mean_elevation_deg", "std_elevation_deg"):
            av, cv = float(a[field]), float(c[field])
            item[f"abstract_{field}"] = av; item[f"cad_{field}"] = cv
            item[f"delta_{field}"] = cv - av if av == av and cv == cv else float("nan")
        rows.append(item)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "array_comparison.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    def metadata(summary):
        with (summary.parent / "summary.json").open(encoding="utf-8") as handle:
            return json.load(handle)
    abstract_meta, cad_meta = metadata(abstract_summary), metadata(cad_summary)
    result = {"status": "completed_array_comparison", "groups": len(rows), "abstract_summary": str(abstract_summary.resolve()), "cad_summary": str(cad_summary.resolve()), "abstract_nonfinite_aoa_count": abstract_meta.get("nonfinite_aoa_count"), "cad_nonfinite_aoa_count": cad_meta.get("nonfinite_aoa_count"), "cad_geometry_status": "copper_centroid_virtual_array_not_phase_center", "hardware_validated": False}
    (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.74 抽象阵列与 PCB-derived 阵列对照\n\n"
        "两组结果使用相同的 5 个海况、9 组 CFAR 参数、4 种检测器和相同复数距离-多普勒谱；唯一改变是 AoA 相位平面拟合所用的 4×4 虚拟阵列坐标。\n\n"
        "## 如何阅读\n\n"
        "`delta_* = CAD-derived - abstract`。检测数量差异理论上应为零，因为检测器只使用功率谱；方位/俯仰均值、标准差和非有限 AoA 数量的差异来自阵列坐标对相位拟合的影响。\n\n"
        "## 证据边界\n\n"
        "PCB-derived 坐标由 RF 铜区几何中心求得，不是电气相位中心；因此本对照只能回答‘阵列几何假设变化会怎样’，不能作为实板 AoA 精度结论。\n", encoding="utf-8")
    return result


def main():
    p = argparse.ArgumentParser(); p.add_argument("--abstract-summary", type=Path, required=True); p.add_argument("--cad-summary", type=Path, required=True); p.add_argument("--output", type=Path, required=True); a=p.parse_args(); print(json.dumps(compare(a.abstract_summary.resolve(), a.cad_summary.resolve(), a.output.resolve()), indent=2, ensure_ascii=False))


if __name__ == "__main__": main()
