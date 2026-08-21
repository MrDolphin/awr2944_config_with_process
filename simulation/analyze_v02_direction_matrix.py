"""Analyze the four V0.2 direction-matrix MATLAB runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile

from simulation.run_v02 import analyze_matlab_run


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--run-prefix", default="v02b_dir")
    args = parser.parse_args()
    config_path = Path(__file__).with_name("configs") / "sea_states_0_to_3.json"
    config_value = json.loads(config_path.read_text(encoding="utf-8"))
    repo_root = Path(__file__).resolve().parents[1]
    radar_cfg = (repo_root / "Config" / "profile_3d_3Azim_1ElevTx_awr2944P.cfg").resolve()
    for direction in (0, 90, 180):
        direction_config = dict(config_value)
        direction_config["sea_surface"] = dict(config_value["sea_surface"])
        direction_config["sea_surface"]["wind_direction_deg"] = direction
        direction_config["radar"] = dict(config_value["radar"])
        direction_config["radar"]["cfg_path"] = str(radar_cfg)
        direction_config["output"] = dict(config_value["output"])
        direction_config["output"]["directory"] = str(args.results_root.resolve())
        source = args.results_root / "matlab" / f"{args.run_prefix}{direction:03d}_seed101"
        output_id = f"{args.run_prefix}{direction:03d}_seed101_analysis"
        with tempfile.TemporaryDirectory(prefix="awr2944_v02_dir_") as temp_dir:
            direction_config_path = Path(temp_dir) / "sea_states_direction.json"
            direction_config_path.write_text(
                json.dumps(direction_config, indent=2, ensure_ascii=False),
                encoding="utf-8",
            )
            analyze_matlab_run(
                input_run=source,
                config_path=direction_config_path,
                run_id=output_id,
                render_plots=True,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
