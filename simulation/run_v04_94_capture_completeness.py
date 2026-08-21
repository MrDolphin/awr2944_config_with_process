"""Check execution completeness of the V0.4.93 known-angle capture plan."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def _find_case_dir(root: Path, case_id: str) -> Path | None:
    direct = root / case_id
    if direct.is_dir():
        return direct
    for candidate in root.rglob("*"):
        if candidate.is_dir() and candidate.name == case_id:
            return candidate
    return None


def run(plan_csv: Path, capture_root: Path, output: Path) -> dict:
    with plan_csv.open(encoding="utf-8", newline="") as handle:
        plan = list(csv.DictReader(handle))
    rows = []
    for item in plan:
        case_id = item["case_id"]
        case_dir = _find_case_dir(capture_root, case_id) if capture_root.is_dir() else None
        manifest = case_dir / "manifest.json" if case_dir else None
        iq = next((case_dir / name for name in ("capture.bin", "capture.h5", "capture.hdf5") if case_dir and (case_dir / name).is_file()), None)
        cfg = case_dir / "profile.cfg" if case_dir else None
        rows.append({"case_id": case_id, "azimuth_deg": item["azimuth_deg"], "elevation_deg": item["elevation_deg"], "range_m": item["range_m"], "case_dir": str(case_dir.resolve()) if case_dir else "", "manifest": bool(manifest and manifest.is_file()), "raw_iq": bool(iq), "cfg": bool(cfg and cfg.is_file()), "complete_basic_capture": bool(manifest and manifest.is_file() and iq and cfg and cfg.is_file())})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "capture_completeness.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    complete = sum(row["complete_basic_capture"] for row in rows)
    summary = {"status": "completed_capture_completeness_check", "plan_csv": str(plan_csv.resolve()), "capture_root": str(capture_root.resolve()), "planned_count": len(rows), "complete_basic_capture_count": complete, "coverage_fraction": complete / len(rows) if rows else 0.0, "missing_case_count": len(rows) - complete, "hardware_aoa_validated": False, "next_action": "collect missing cases before running V0.4.92 batch" if complete < len(rows) else "run V0.4.92 known-angle batch"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    azimuths = sorted({float(row["azimuth_deg"]) for row in rows}); elevations = sorted({float(row["elevation_deg"]) for row in rows}); distances = sorted({float(row["range_m"]) for row in rows})
    figure, axes = plt.subplots(1, len(distances), figsize=(5 * len(distances), 4), squeeze=False, constrained_layout=True)
    for index, distance in enumerate(distances):
        axis = axes[0][index]
        matrix = [[1 if any(float(row["azimuth_deg"]) == az and float(row["elevation_deg"]) == el and float(row["range_m"]) == distance and row["complete_basic_capture"] for row in rows) else 0 for az in azimuths] for el in elevations]
        image = axis.imshow(matrix, origin="lower", aspect="auto", vmin=0, vmax=1, cmap="RdYlGn")
        axis.set_xticks(range(len(azimuths)), [str(int(value)) for value in azimuths]); axis.set_yticks(range(len(elevations)), [str(int(value)) for value in elevations]); axis.set_xlabel("azimuth (deg)"); axis.set_ylabel("elevation (deg)"); axis.set_title(f"range {distance:g} m")
    figure.savefig(output / "capture_completeness.png", dpi=160); plt.close(figure)
    lines = ["# V0.4.94 已知角采集完整性检查", "", f"计划工况：{len(rows)}；基础文件齐全：{complete}；覆盖率：{summary['coverage_fraction']:.1%}。", "", "## 完整条件", "", "每个工况至少必须同时存在 `manifest.json`、`capture.bin/HDF5` 和 `profile.cfg`。这只是文件完整性，不等于通道顺序、校准矩阵或机械姿态已经验证。", "", "## 下一步", "", summary["next_action"], "", "图中绿色表示基础文件齐全，红色表示至少缺少一个文件。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--plan", type=Path, required=True); parser.add_argument("--capture-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.plan.resolve(), args.capture_root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
