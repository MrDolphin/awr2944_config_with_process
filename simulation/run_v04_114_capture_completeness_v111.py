"""Check the V0.4.113 scaffold with CFG/hash-aware evidence gates."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run(plan_csv: Path, capture_root: Path, output: Path) -> dict:
    with plan_csv.open(encoding="utf-8", newline="") as handle:
        plan = list(csv.DictReader(handle))
    rows = []
    for item in plan:
        case_dir = capture_root / item["case_id"]
        manifest_path = case_dir / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.is_file() else {}
        cfg_name = manifest.get("capture", {}).get("cfg_file", "profile.cfg")
        cfg_path = case_dir / cfg_name
        iq_path = next((case_dir / name for name in ("capture.bin", "capture.h5", "capture.hdf5") if (case_dir / name).is_file()), None)
        expected_sha = manifest.get("capture", {}).get("cfg_sha256")
        actual_sha = sha256(cfg_path) if cfg_path.is_file() else None
        geometry_ok = (manifest.get("capture", {}).get("samples_per_chirp") == 656 and manifest.get("capture", {}).get("chirps") == 64 and manifest.get("capture", {}).get("rx_count") == 4 and manifest.get("capture", {}).get("tdm_tx_sequence") == [0, 2, 3, 1])
        rows.append({"case_id": item["case_id"], "manifest": manifest_path.is_file(), "raw_iq": iq_path is not None, "cfg": cfg_path.is_file(), "cfg_hash_match": bool(cfg_path.is_file() and expected_sha and expected_sha == actual_sha), "geometry_match": geometry_ok, "basic_complete": bool(manifest_path.is_file() and iq_path and cfg_path.is_file()), "ready_for_v0112": bool(manifest_path.is_file() and iq_path and cfg_path.is_file() and expected_sha == actual_sha and geometry_ok), "missing_or_invalid": ";".join([name for name, present in (("manifest", manifest_path.is_file()), ("raw_iq", iq_path is not None), ("cfg", cfg_path.is_file()), ("cfg_hash", bool(cfg_path.is_file() and expected_sha == actual_sha)), ("geometry", geometry_ok)) if not present])})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "capture_completeness_v111.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    complete = sum(row["basic_complete"] for row in rows); ready = sum(row["ready_for_v0112"] for row in rows)
    summary = {"status": "completed_v111_capture_completeness_check", "planned_count": len(rows), "basic_complete_count": complete, "ready_for_v0112_count": ready, "coverage_fraction": complete / len(rows) if rows else 0.0, "ready_fraction": ready / len(rows) if rows else 0.0, "hardware_aoa_validated": False, "next_action": "collect missing capture.bin/HDF5 files" if ready < len(rows) else "run V0.4.112 on ready cases"}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        figure, axis = plt.subplots(figsize=(10, 3.5), constrained_layout=True)
        axis.imshow([[1 if row["ready_for_v0112"] else 0 for row in rows]], aspect="auto", cmap="RdYlGn", vmin=0, vmax=1)
        axis.set_yticks([]); axis.set_xlabel("planned case index"); axis.set_title(f"V0.4.114 ready for IQ decode: {ready}/{len(rows)}")
        figure.savefig(output / "capture_completeness_v111.png", dpi=160); plt.close(figure)
    except Exception:
        pass
    lines = ["# V0.4.114 V0.4.113 采集完整性检查", "", f"计划工况：{len(rows)}；基础文件齐全：{complete}；满足 V0.4.112 解码门：{ready}。", "", "## 通过条件", "", "每个案例必须有 manifest、capture.bin/HDF5、manifest 指定的 CFG，且 CFG SHA-256 与 manifest 一致；同时 samples/chirp、chirps/frame、RX 和 TDM TX 顺序必须为当前 CFG 提取值。", "", "## 结果解释", "", "基础文件齐全不等于 ADC IQ 有效；满足解码门也不等于通道顺序、TI 校准和硬件 AoA 已验证。", "", f"当前下一步：{summary['next_action']}。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--plan", type=Path, required=True); parser.add_argument("--capture-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.plan.resolve(), args.capture_root.resolve(), args.output.resolve()), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
