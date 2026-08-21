"""Extract Altium layer-stack fields and RF-net primitive layer usage."""

from __future__ import annotations

import argparse
import csv
import re
from collections import Counter
from pathlib import Path

from simulation.extract_pcb_rf_ports import NET_NAMES, value


def stack_fields(text: str) -> list[dict[str, str]]:
    indexes = sorted({int(index) for index, _ in re.findall(r"V9_STACK_LAYER(\d+)_NAME=([^|]+)", text)})
    rows = []
    for index in indexes:
        prefix = f"V9_STACK_LAYER{index}_"
        row = {"stack_index": str(index)}
        for key in ("NAME", "LAYERID", "DIELCONST", "DIELHEIGHT", "DIELMATERIAL", "COPTHICK", "COMPONENTPLACEMENT"):
            match = re.search(re.escape(prefix + key) + r"=([^|]+)", text)
            row[key.lower()] = match.group(1).strip() if match else ""
        rows.append(row)
    return rows


def run(pcbdoc: Path, output: Path) -> None:
    text = pcbdoc.read_text(errors="ignore")
    layers = stack_fields(text)
    rf_counts: Counter[tuple[str, str]] = Counter()
    for line in text.splitlines():
        record, net, layer = value(line, "RECORD"), value(line, "NET"), value(line, "LAYER")
        if record and net and net.isdigit() and int(net) in NET_NAMES:
            rf_counts[(record, layer or "")] += 1
    output.mkdir(parents=True, exist_ok=True)
    with (output / "layer_stack.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(layers[0]))
        writer.writeheader(); writer.writerows(layers)
    rf_rows = [{"record": record, "layer": layer, "count": count}
               for (record, layer), count in sorted(rf_counts.items())]
    with (output / "rf_object_layers.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["record", "layer", "count"])
        writer.writeheader(); writer.writerows(rf_rows)
    top_only = bool(rf_counts) and set(layer for (_record, layer) in rf_counts) == {"TOP"}
    (output / "output_analysis.md").write_text(
        "# V0.4.16 PCB 层叠与 RF 对象层分析\n\n"
        f"层叠记录数：{len(layers)}；RF 网络对象是否全部位于 TOP：{top_only}。\n\n"
        "`layer_stack.csv` 来自 ASCII PCB 的 `V9_STACK_LAYER*` 字段；`rf_object_layers.csv` 统计 8 个 TX/RX 网络的对象层。"
        "这些数据可作为后续传输线/电磁仿真的输入，但尚未计算相位延迟。\n",
        encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcbdoc", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.pcbdoc.resolve(), args.output.resolve())
    print(f"Analyzed PCB layer stack into {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
