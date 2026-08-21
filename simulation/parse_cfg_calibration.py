"""Parse TI compRangeBiasAndRxChanPhase into the project 4x4 contract."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np


def parse(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    match = re.search(r"^compRangeBiasAndRxChanPhase\s+(.+)$", text, re.MULTILINE)
    if not match:
        raise ValueError("compRangeBiasAndRxChanPhase not found")
    tokens = [float(token) for token in match.group(1).split()]
    if len(tokens) != 33:
        raise ValueError(f"expected range bias + 32 real/imag values, found {len(tokens)}")
    range_bias_m = tokens[0]
    amplitude = np.zeros((4, 4)); phase_deg = np.zeros((4, 4))
    complex_values = []
    for index in range(16):
        real, imag = tokens[1 + 2 * index], tokens[1 + 2 * index + 1]
        value = complex(real, imag)
        rx, tx = index % 4, index // 4
        amplitude[rx, tx] = abs(value)
        phase_deg[rx, tx] = np.rad2deg(np.angle(value))
        complex_values.append({"tx_index": tx, "rx_index": rx, "real": real, "imag": imag})
    identity = bool(np.allclose(amplitude, 1.0) and np.allclose(phase_deg, 0.0))
    return {
        "schema_version": "awr2944p-calibration-v0.1",
        "source_cfg": str(path.resolve()),
        "channel_order": "Tx0Rx0,Tx0Rx1,...,Tx3Rx3; output matrices indexed [rx][tx]",
        "range_bias_m": range_bias_m,
        "amplitude": amplitude.tolist(), "phase_deg": phase_deg.tolist(),
        "raw_tokens": tokens, "complex_values_tx_major": complex_values,
        "calibration_status": "cfg_identity_not_measured" if identity else "cfg_values_present_measurement_provenance_required",
        "interpretation_status": "assumed_direct_complex_correction_values_verify_against_ti_sdk",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    payload = parse(args.cfg.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Parsed CFG calibration into {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
