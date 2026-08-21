"""Create a traceable AWR2944P CFG snapshot for V0.4."""

from __future__ import annotations

import argparse
from pathlib import Path

from simulation.hardware_cfg import write_snapshot


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    write_snapshot(args.cfg.resolve(), args.output.resolve())
    print(f"Wrote AWR2944P CFG snapshot to {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
