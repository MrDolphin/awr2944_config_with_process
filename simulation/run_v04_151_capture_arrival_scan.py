"""Scan known-angle cases and gate any newly arrived captures safely."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from simulation.run_v04_142_center_iq_decode_gate import run as decode_gate
from simulation.run_v04_149_raw_capture_geometry_audit import run as geometry_audit


def run(cases_root: Path, output: Path, decode_ready: bool = False) -> dict:
    cases_root = cases_root.resolve(); output.mkdir(parents=True, exist_ok=True)
    rows = []
    for case in sorted(cases_root.glob("lv*")):
        if not case.is_dir() or not (case / "manifest.json").is_file():
            continue
        case_out = output / case.name
        geometry = geometry_audit(case, case_out / "geometry")
        row = {"case_id": case.name, "capture_present": bool(geometry["capture_bytes"]), "geometry": geometry, "decode_started": False}
        if decode_ready and geometry["ready_for_decode"]:
            decoded = decode_gate(case, case_out / "decode")
            row["decode_started"] = bool(decoded.get("decode_started")); row["decode"] = decoded
        rows.append(row)
    ready = sum(1 for row in rows if row["geometry"]["ready_for_decode"])
    result = {
        "status": "capture_arrival_scan_completed",
        "cases_root": str(cases_root), "output": str(output.resolve()),
        "case_count": len(rows), "geometry_ready_count": ready,
        "decode_requested": decode_ready,
        "rows": rows,
        "hardware_commands_executed": False,
        "hardware_aoa_validated": False,
        "next_action": "review geometry output; add --decode-ready only after operator approval" if not decode_ready else "inspect decoded HDF5 and verify channel order",
    }
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.151 采集到达扫描\n\n"
        f"扫描案例：`{len(rows)}`；几何审计通过：`{ready}`；请求解码：`{decode_ready}`。\n\n"
        "默认模式只运行 V0.149 几何审计，不启动解码。`--decode-ready` 也不会执行硬件命令，只会对已通过几何审计的案例调用 V0.142。\n\n"
        "硬件 AoA 仍需通道顺序、TI 校准和已知角度证据，不能由本扫描器自动确认。\n",
        encoding="utf-8",
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--cases-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--decode-ready", action="store_true"); args = parser.parse_args(); print(json.dumps(run(args.cases_root, args.output, args.decode_ready), ensure_ascii=False, indent=2)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
