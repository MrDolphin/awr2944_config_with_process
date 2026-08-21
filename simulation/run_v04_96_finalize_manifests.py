"""Finalize capture manifests with hashes without overwriting measurement fields."""

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


def run(root: Path, output: Path) -> dict:
    manifests = sorted(root.rglob("manifest.json")) if root.is_dir() else []
    rows = []
    for manifest_path in manifests:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        capture_name = str(payload.get("capture_file", "capture.bin"))
        capture_path = manifest_path.parent / capture_name
        if not capture_path.is_file():
            for candidate in ("capture.bin", "capture.h5", "capture.hdf5"):
                if (manifest_path.parent / candidate).is_file():
                    capture_path = manifest_path.parent / candidate
                    payload["capture_file"] = candidate
                    break
        cfg_path = manifest_path.parent / str(payload.get("capture", {}).get("cfg_file", "profile.cfg"))
        capture_found = capture_path.is_file()
        cfg_found = cfg_path.is_file()
        if capture_found:
            payload["capture_sha256"] = sha256(capture_path)
        if cfg_found:
            payload.setdefault("capture", {})["cfg_sha256"] = sha256(cfg_path)
        if capture_found and cfg_found:
            payload["evidence_status"] = "captured_pending_validation"
        manifest_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        rows.append({"case_id": payload.get("capture_id", manifest_path.parent.name), "manifest": str(manifest_path.resolve()), "capture_found": capture_found, "cfg_found": cfg_found, "status": payload.get("evidence_status", "")})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "manifest_finalize.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["case_id", "manifest", "capture_found", "cfg_found", "status"]); writer.writeheader(); writer.writerows(rows)
    summary = {"status": "completed_manifest_finalize", "manifest_count": len(rows), "capture_found_count": sum(row["capture_found"] for row in rows), "cfg_found_count": sum(row["cfg_found"] for row in rows), "both_found_count": sum(row["capture_found"] and row["cfg_found"] for row in rows), "metadata_fields_overwritten": ["capture_file_if_detected", "capture_sha256_if_file_exists", "capture.cfg_sha256_if_file_exists", "evidence_status_if_both_files_exist"], "hardware_aoa_validated": False}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text("# V0.4.96 采集 manifest 自动收口\n\n本工具只补写真实文件的 SHA-256 和采集状态，不覆盖目标真值、机械姿态、通道顺序或校准结论。\n\n" + f"发现 manifest：{len(rows)}；原始 IQ：{summary['capture_found_count']}；CFG：{summary['cfg_found_count']}；两者同时存在：{summary['both_found_count']}。\n\n两类文件同时存在的工况会标记为 `captured_pending_validation`，下一步仍需运行 V0.94 和 V0.92。\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.root.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
