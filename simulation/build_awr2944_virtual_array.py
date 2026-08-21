"""Build a CAD-derived virtual-array table from extracted TX/RX regions."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def build(input_csv: Path, output_csv: Path, frequency_ghz: float = 77.0) -> None:
    rows = list(csv.DictReader(input_csv.open(encoding="utf-8", newline="")))
    tx = {row["antenna"]: row for row in rows if row["antenna"].startswith("TX")}
    rx = {row["antenna"]: row for row in rows if row["antenna"].startswith("RX")}
    origin_x = float(tx["TX1"]["center_x_mm"]) + float(rx["RX1"]["center_x_mm"])
    origin_y = float(tx["TX1"]["center_y_mm"]) + float(rx["RX1"]["center_y_mm"])
    wavelength_mm = 299.792458 / frequency_ghz
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    fields = ["virtual_channel", "tx", "rx", "x_mm", "y_mm", "x_lambda", "y_lambda",
              "coordinate_status", "source"]
    with output_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        index = 0
        for tx_name in sorted(tx):
            for rx_name in sorted(rx):
                # Approximate virtual phase center as TX/RX centroid sum relative to RX1.
                x = (float(tx[tx_name]["center_x_mm"]) + float(rx[rx_name]["center_x_mm"]) - origin_x)
                y = (float(tx[tx_name]["center_y_mm"]) + float(rx[rx_name]["center_y_mm"]) - origin_y)
                writer.writerow({"virtual_channel": index, "tx": tx_name, "rx": rx_name,
                    "x_mm": f"{x:.6f}", "y_mm": f"{y:.6f}",
                    "x_lambda": f"{x / wavelength_mm:.6f}", "y_lambda": f"{y / wavelength_mm:.6f}",
                    "coordinate_status": "cad_copper_centroid_sum_approximation_not_phase_center",
                    "source": str(input_csv.resolve())})
                index += 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    build(args.input.resolve(), args.output.resolve())
    print(f"Built 16 CAD-derived virtual channels into {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
