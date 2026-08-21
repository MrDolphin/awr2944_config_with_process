"""Audit IQ-like files and classify their provenance without altering them."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def classify(path: Path) -> dict:
    suffix = path.suffix.lower()
    attrs = {}
    if suffix in {".h5", ".hdf5"}:
        try:
            import h5py
            with h5py.File(path, "r") as handle:
                attrs = {str(key): str(value.item() if hasattr(value, "item") else value) for key, value in handle.attrs.items()}
        except Exception as exc:
            attrs = {"read_error": f"{type(exc).__name__}: {exc}"}
    text = (str(path) + " " + " ".join(attrs.values())).lower()
    if any(token in text for token in ("synthetic", "fixture", "sea_clutter", "replay")):
        provenance = "synthetic_or_replay"
    elif any(token in text for token in ("hardware", "dca1000", "measured", "capture")):
        provenance = "hardware_candidate_requires_manifest"
    else:
        provenance = "unknown_requires_manifest"
    return {"path": str(path.resolve()), "suffix": suffix, "size_bytes": path.stat().st_size, "provenance": provenance, "attrs": json.dumps(attrs, ensure_ascii=False, sort_keys=True)}


def run(roots: list[Path], output: Path) -> dict:
    paths = []
    for root in roots:
        if root.is_dir():
            paths.extend(path for path in root.rglob("*") if path.is_file() and path.suffix.lower() in {".bin", ".h5", ".hdf5"})
    rows = [classify(path) for path in sorted(set(paths))]
    output.mkdir(parents=True, exist_ok=True)
    with (output / "data_source_audit.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["path", "suffix", "size_bytes", "provenance", "attrs"]); writer.writeheader(); writer.writerows(rows)
    counts = {name: sum(row["provenance"] == name for row in rows) for name in ("synthetic_or_replay", "hardware_candidate_requires_manifest", "unknown_requires_manifest")}
    summary = {"status": "completed_data_source_audit", "roots": [str(root.resolve()) for root in roots], "file_count": len(rows), "provenance_counts": counts, "hardware_aoa_validated": False}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text("# V0.4.97 IQ 数据来源审计\n\n本审计只读扫描 `.bin/.h5/.hdf5`，不修改数据。分类依据是路径和 HDF5 属性，最终硬件证据仍必须由 V0.90 manifest、CFG、采集元数据和校准记录确认。\n\n" + f"扫描文件：{len(rows)}；合成/回放：{counts['synthetic_or_replay']}；硬件候选：{counts['hardware_candidate_requires_manifest']}；未知：{counts['unknown_requires_manifest']}。\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--root", type=Path, action="append", required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run([path.resolve() for path in args.root], args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
