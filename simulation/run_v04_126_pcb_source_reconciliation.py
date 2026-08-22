"""Reconcile the newly supplied SPRR440/SPRR441 hardware source package.

This stage inventories the source files and re-runs the ASCII PCB extraction
on the supplied board.  It deliberately keeps package-pad/board coordinates
separate from antenna phase-center coordinates.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _classify(path: Path) -> str:
    suffix = path.suffix.lower()
    if suffix in {".pcbdoc", ".prjpcb", ".schdoc", ".dat", ".outjob"}:
        return "altium_source"
    if suffix in {".step", ".stp"}:
        return "mechanical_cad"
    if suffix in {".pdf"}:
        return "document"
    if suffix in {".xls", ".xlsx"}:
        return "bom"
    return "other"


def _ascii_summary(path: Path) -> dict:
    from simulation.run_v04_102_pcb_asset_audit import records

    text = path.read_text(errors="ignore")
    components = list(records(text, "Component"))
    nets = {row.get("ID"): row.get("NAME", "") for row in records(text, "Net")}
    pads = list(records(text, "Pad"))
    chip = next((item for item in components if item.get("SOURCEDESCRIPTION") == "XA2944BGALT"), None)
    target_names = {"TX1", "TX2", "TX3", "TX4", "RX1", "RX2", "RX3", "RX4"}
    target_pads = [pad for pad in pads if nets.get(pad.get("NET")) in target_names]
    return {
        "format": "Altium ASCII PcbDoc",
        "component_count": len(components),
        "net_count": len(nets),
        "target_rf_pad_count": len(target_pads),
        "target_rf_nets": sorted({nets.get(pad.get("NET"), "") for pad in target_pads}),
        "chip": {key: chip.get(key, "") for key in ("SOURCEDESIGNATOR", "SOURCEDESCRIPTION", "X", "Y", "ROTATION")} if chip else None,
    }


def reconcile(sprr440: Path, sprr441: Path, output: Path) -> dict:
    files = []
    for root in (sprr440, sprr441):
        for path in sorted(root.rglob("*")):
            if path.is_file():
                item = {"source_root": root.name, "relative_path": str(path.relative_to(root)), "suffix": path.suffix.lower(), "bytes": path.stat().st_size, "kind": _classify(path), "sha256": _sha256(path)}
                if path.name.lower() == "pro113d_ascii.pcbdoc" or path.name.lower() == "proc113d_ascii.pcbdoc":
                    item["ascii_pcb"] = _ascii_summary(path)
                files.append(item)
    ascii_items = [item for item in files if "ascii_pcb" in item]
    output.mkdir(parents=True, exist_ok=True)
    with (output / "source_inventory.csv").open("w", encoding="utf-8", newline="") as handle:
        fields = ["source_root", "relative_path", "suffix", "bytes", "kind", "sha256"]
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows([{key: item.get(key, "") for key in fields} for item in files])
    result = {
        "status": "completed_pcb_source_reconciliation",
        "source_roots": [str(sprr440.resolve()), str(sprr441.resolve())],
        "file_count": len(files),
        "kind_counts": {kind: sum(1 for item in files if item["kind"] == kind) for kind in sorted({item["kind"] for item in files})},
        "ascii_pcb_candidates": ascii_items,
        "phase_center_ready": False,
        "evidence_boundary": [
            "ASCII PCB gives board/package/RF-net coordinates, not chip antenna phase centers",
            "STEP and assembly drawings support mechanical orientation and mounting evidence",
            "BOM supports component identity/version but does not provide array phase centers",
            "measured or EM-derived TX/RX patterns and calibration remain required",
        ],
    }
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    report = [
        "# V0.4.126 新增 SPRR440/SPRR441 PCB/CAD 资料对账",
        "",
        f"本阶段扫描两个资料根目录，共发现 **{len(files)}** 个文件。完整清单与 SHA-256 见 `source_inventory.csv`。",
        "",
        "## 可直接用于仿真的证据",
        "",
        "- `PROC113D_ASCII.PcbDoc` 可解析板级坐标、AWR2944 封装位置、TX1–TX4/RX1–RX4 网络和封装焊盘坐标。",
        "- `PROC113D_BRD.step`、装配图和层叠图可用于确认板面朝向、机械安装和介质层信息。",
        "- 原理图、BOM 和 Altium 工程文件可用于确认器件、网络和资料版本。",
        "",
        "## 重要边界",
        "",
        "当前自动提取到的是板级/封装级 RF 几何线索，不能直接当作 AWR2944 芯片内部天线阵元的电气相位中心。把 TX/RX 封装焊盘坐标直接用于 AoA，会把封装连接位置误当成辐射中心。",
        "",
        "## 对下一步阵面仿真的影响",
        "",
        "下一步应建立三个模型并同时保留：理想半波长阵列、PCB/封装几何近似阵列、实测或 EM 相位中心阵列。只有第三种模型与已知角度 DCA1000 IQ 和 TI 校准结果闭环后，才能用于硬件 AoA 精度结论。",
        "",
        "## 当前结论",
        "",
        "资料包足以继续做硬件证据追踪和候选阵列敏感性分析，但目前仍不能从 PCB 文件单独证明真实方向图、相位中心、互耦或 AoA 性能。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(report), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sprr440", type=Path, required=True)
    parser.add_argument("--sprr441", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(reconcile(args.sprr440.resolve(), args.sprr441.resolve(), args.output.resolve()), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
