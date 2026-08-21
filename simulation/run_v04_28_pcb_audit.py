"""Audit AWR2944 EVM PCB/CAD sources and extract traceable RF candidates.

The extractor deliberately reports candidates rather than claiming antenna phase
centres.  Altium ASCII PCB records are parsed without requiring Altium Designer.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from pathlib import Path


RECORD_RE = re.compile(r"\|RECORD=(?P<record>[^|]+)(?P<body>.*)")
FIELD_RE = re.compile(r"\|(?P<key>[^=|]+)=(?P<value>[^|]*)")
LENGTH_RE = re.compile(r"^\s*([-+]?\d+(?:\.\d+)?)mil\s*$", re.IGNORECASE)
RF_RE = re.compile(r"(?:^|[_+\-])(?:TX|RX|ANT|RF)(?:$|[_+\-\d])|AWR2944|MMW", re.IGNORECASE)
RADAR_CHANNEL_RE = re.compile(r"^(?:TX|RX)[0-4](?:_[PN])?$", re.IGNORECASE)
DIGITAL_RE = re.compile(r"(?:LVDS|UART|CAN|CSI|CLK|CTRL|DV|ER|TRACE|RGMII)", re.IGNORECASE)


def fields(line: str) -> tuple[str, dict[str, str]] | None:
    match = RECORD_RE.search(line)
    if not match:
        return None
    values = {item.group("key"): item.group("value") for item in FIELD_RE.finditer(match.group("body"))}
    return match.group("record"), values


def read_records(path: Path):
    with path.open(encoding="utf-8", errors="replace") as handle:
        for line in handle:
            parsed = fields(line.rstrip("\r\n"))
            if parsed:
                yield parsed


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def source_manifest(root: Path) -> list[dict]:
    rows = []
    for path in sorted(p for p in root.rglob("*") if p.is_file()):
        rows.append({
            "relative_path": str(path.relative_to(root)),
            "absolute_path": str(path.resolve()),
            "extension": path.suffix.lower(),
            "size_bytes": path.stat().st_size,
            "sha256": sha256(path),
            "role": role_for(path),
        })
    return rows


def role_for(path: Path) -> str:
    name = path.name.lower()
    if path.suffix.lower() == ".pcbdoc":
        return "altium_pcb_ascii_or_binary"
    if path.suffix.lower() == ".step":
        return "mechanical_3d_cad"
    if "bom" in name:
        return "bill_of_materials"
    if "layer" in name:
        return "pcb_layer_stack_document"
    if path.suffix.lower() in {".schdoc", ".pdf"}:
        return "schematic_or_assembly_document"
    if path.suffix.lower() in {".outjob", ".dat", ".prjpcb"}:
        return "altium_project_or_fabrication_metadata"
    return "other_source"


def classify_net(name: str) -> str:
    if RADAR_CHANNEL_RE.fullmatch(name.strip()):
        return "radar_channel_name_candidate"
    if DIGITAL_RE.search(name):
        return "digital_interface_name_candidate"
    return "other_rf_name_candidate"


def write_csv(path: Path, rows: list[dict], fieldnames: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_ascii_pcb(path: Path) -> dict:
    counts: dict[str, int] = {}
    components: list[dict] = []
    nets: list[dict] = []
    pads: list[dict] = []
    layers: list[dict] = []
    board_header: dict[str, str] = {}
    for record, values in read_records(path):
        counts[record] = counts.get(record, 0) + 1
        if record == "Board":
            if not board_header:
                board_header = {key: values[key] for key in ("FILENAME", "KIND", "VERSION", "DATE", "TIME", "ORIGINX", "ORIGINY") if key in values}
            for key, value in values.items():
                match = re.fullmatch(r"V9_STACK_LAYER(\d+)_NAME", key)
                if match:
                    index = int(match.group(1))
                    layer = {"index": index, "name": value, "status": "confirmed_from_ascii_pcb"}
                    for suffix, output in (("DIELCONST", "dielectric_constant"), ("DIELHEIGHT", "dielectric_height"), ("DIELMATERIAL", "dielectric_material"), ("COPTHICK", "copper_thickness"), ("LAYERID", "layer_id")):
                        source_key = f"V9_STACK_LAYER{index}_{suffix}"
                        if source_key in values:
                            layer[output] = values[source_key]
                    layers.append(layer)
        elif record == "Component":
            row = {key: values.get(key, "") for key in ("ID", "X", "Y", "ROTATION", "PATTERN", "SOURCEDESIGNATOR", "SOURCEDESCRIPTION", "SOURCEFOOTPRINTLIBRARY", "SOURCELIBREFERENCE")}
            row["rf_name_candidate"] = bool(RF_RE.search(" ".join(row.values())))
            row["status"] = "candidate_name_only" if row["rf_name_candidate"] else "inventory_only"
            components.append(row)
        elif record == "Net":
            name = values.get("NAME", "")
            is_candidate = bool(RF_RE.search(name))
            nets.append({"id": values.get("ID", ""), "name": name, "rf_name_candidate": is_candidate, "classification": classify_net(name) if is_candidate else "not_applicable", "status": "candidate_name_only" if is_candidate else "inventory_only"})
        elif record == "Pad":
            pad = {key: values.get(key, "") for key in ("INDEXFORSAVE", "LAYER", "LAYER_V7", "NAME", "X", "Y", "XSIZE", "YSIZE", "SHAPE", "UNIQUEID")}
            pad["status"] = "geometry_only_unconfirmed"
            pads.append(pad)
    unique_layers = {layer["index"]: layer for layer in layers}
    return {"board_header": board_header, "record_counts": counts, "components": components, "nets": nets, "pads": pads, "layers": [unique_layers[index] for index in sorted(unique_layers)]}


def run(source_roots: list[Path], output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    manifest_rows: list[dict] = []
    for root in source_roots:
        manifest_rows.extend(source_manifest(root.resolve()))
    write_csv(output / "source_manifest.csv", manifest_rows, ["relative_path", "absolute_path", "extension", "size_bytes", "sha256", "role"])

    pcb_paths = [Path(row["absolute_path"]) for row in manifest_rows if row["extension"] == ".pcbdoc" and "ASCII" in Path(row["absolute_path"]).name]
    parsed = parse_ascii_pcb(pcb_paths[0]) if pcb_paths else {"board_header": {}, "record_counts": {}, "components": [], "nets": [], "pads": [], "layers": []}
    write_csv(output / "pcb_component_inventory.csv", parsed["components"], ["ID", "X", "Y", "ROTATION", "PATTERN", "SOURCEDESIGNATOR", "SOURCEDESCRIPTION", "SOURCEFOOTPRINTLIBRARY", "SOURCELIBREFERENCE", "rf_name_candidate", "status"])
    write_csv(output / "rf_net_inventory.csv", [row for row in parsed["nets"] if row["rf_name_candidate"]], ["id", "name", "rf_name_candidate", "classification", "status"])
    write_csv(output / "rf_pad_candidates.csv", parsed["pads"], ["INDEXFORSAVE", "LAYER", "LAYER_V7", "NAME", "X", "Y", "XSIZE", "YSIZE", "SHAPE", "UNIQUEID", "status"])
    write_csv(output / "layer_stack.csv", parsed["layers"], sorted({key for row in parsed["layers"] for key in row}) or ["index", "name", "status"])
    summary = {"status": "completed_pcb_source_audit", "source_roots": [str(root.resolve()) for root in source_roots], "ascii_pcb": str(pcb_paths[0].resolve()) if pcb_paths else None, "record_counts": parsed["record_counts"], "component_count": len(parsed["components"]), "net_count": len(parsed["nets"]), "rf_net_candidate_count": sum(row["rf_name_candidate"] for row in parsed["nets"]), "pad_count": len(parsed["pads"]), "layer_count": len(parsed["layers"]), "phase_center_coordinates_confirmed": False}
    (output / "audit_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.28 PCB/CAD 资料审计", "", f"- 来源根目录数：{len(source_roots)}", f"- ASCII PCB：{summary['ascii_pcb'] or '未找到'}", f"- Component 记录：{summary['component_count']}", f"- Net 记录：{summary['net_count']}", f"- RF/TX/RX 名称候选网络：{summary['rf_net_candidate_count']}", f"- Pad 记录：{summary['pad_count']}", f"- 层叠记录：{summary['layer_count']}", "", "## 结论边界", "", "RF 网络和焊盘候选仅基于名称、坐标和 PCB 记录筛选，尚未确认天线相位中心、TX/RX 通道映射或真实阵列坐标。", "需要结合 TI 天线封装/用户指南、原理图网络、封装库和实测校准继续确认。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", action="append", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.source_root, args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
