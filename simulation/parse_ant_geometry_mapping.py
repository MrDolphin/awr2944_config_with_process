"""Expand antGeometryCfg into SDK Tx/Rx row-column mapping."""

from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path


def parse_cfg(cfg: Path, output: Path) -> None:
    text = cfg.read_text(encoding="utf-8")
    line = re.search(r"^antGeometryCfg\s+(.+)$", text, re.MULTILINE)
    if not line:
        raise ValueError("antGeometryCfg not found")
    tokens = [float(value) for value in line.group(1).split()]
    if len(tokens) != 34:
        raise ValueError(f"expected 34 antGeometryCfg values, found {len(tokens)}")
    rows = []
    for index in range(16):
        tx, rx = divmod(index, 4)
        rows.append({
            "virtual_input_index": index,
            "tx_index": tx, "tx_name": f"TX{tx + 1}",
            "rx_index": rx, "rx_name": f"RX{rx + 1}",
            "row": int(tokens[2 * index]), "column": int(tokens[2 * index + 1]),
            "azimuth_spacing_lambda": tokens[-2], "elevation_spacing_lambda": tokens[-1],
            "source": str(cfg.resolve()),
            "sdk_semantics": "Tx0Rx0,Tx0Rx1,...,Tx3Rx3; row/elevation then column/azimuth",
        })
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    parse_cfg(args.cfg.resolve(), args.output.resolve())
    print(f"Wrote 16 antGeometryCfg mappings to {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
