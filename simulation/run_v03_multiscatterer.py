"""Run the V0.3.2 controlled multi-scatterer sea-surface echo loop."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil

import h5py
import numpy as np

from simulation.artifacts import create_run_directory
from simulation.run_v03 import _plot_range_doppler
from simulation.v03 import FmcwConfig, generate_multi_scatterer_iq, process_range_doppler


def run_multi_scatterer(
    *, input_truth: Path, results_root: Path, run_id: str, stride: int = 20,
    max_scatterers: int = 32, time_index: int = 0,
) -> Path:
    with h5py.File(input_truth, "r") as handle:
        ranges = np.asarray(handle["/truth/slant_range_m"][time_index], dtype=float)
        rates = np.asarray(
            handle["/truth/slant_range_rate_mps"][time_index], dtype=float
        )
        case_id = str(handle.attrs["case_id"])
        source_producer = str(handle.attrs["producer"])
    config = FmcwConfig()
    scatterers: list[tuple[float, float, float]] = []
    for y_index in range(0, ranges.shape[0], max(1, stride)):
        for x_index in range(0, ranges.shape[1], max(1, stride)):
            if len(scatterers) >= max_scatterers:
                break
            range_m = float(ranges[y_index, x_index])
            rate_mps = float(rates[y_index, x_index])
            if range_m < config.max_range_m:
                scatterers.append((range_m, rate_mps, 1.0))
        if len(scatterers) >= max_scatterers:
            break
    iq = generate_multi_scatterer_iq(config, scatterers)
    result = process_range_doppler(iq, config)
    output_run = create_run_directory(
        results_root, producer="python", stage_id="v03_complex_echo_range_doppler", run_id=run_id
    )
    source_run = input_truth.parent.parent
    for name in ("run_config.json", "radar_profile.cfg"):
        source = source_run / name
        if source.is_file():
            shutil.copy2(source, output_run / name)
    with h5py.File(output_run / "data" / "multi_scatterer.h5", "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-complex-echo-v0.3"
        handle.attrs["producer"] = "python"
        handle.attrs["source_truth"] = str(input_truth.resolve())
        handle.create_dataset("/radar/iq", data=iq, compression="gzip")
        handle.create_dataset("/range_doppler/power_linear", data=result.power_linear)
        handle.create_dataset("/axes/range_m", data=result.range_axis_m)
        handle.create_dataset("/axes/velocity_mps", data=result.velocity_axis_mps)
        handle.create_dataset("/truth/scatterers", data=np.asarray(scatterers))
    _plot_range_doppler(
        result,
        output_run / "figures" / "range_doppler.png",
        title="V0.3.2 multi-scatterer Range-Doppler",
    )
    summary = {
        "case_id": case_id,
        "source_producer": source_producer,
        "source_truth": str(input_truth.resolve()),
        "scatterer_count": len(scatterers),
        "scatterer_range_min_m": min(item[0] for item in scatterers),
        "scatterer_range_max_m": max(item[0] for item in scatterers),
        "scatterer_rate_min_mps": min(item[1] for item in scatterers),
        "scatterer_rate_max_mps": max(item[1] for item in scatterers),
        "peak_range_m": result.peak_range_m,
        "peak_velocity_mps": result.peak_velocity_mps,
        "fmcw": {
            "range_resolution_m": config.range_resolution_m,
            "velocity_resolution_mps": config.velocity_resolution_mps,
            "tx_count": config.tx_count,
            "rx_count": config.rx_count,
        },
    }
    (output_run / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    (output_run / "output_analysis.md").write_text(
        "# V0.3.2 输出分析\n\n"
        f"- 来源 V0.2 truth：`{input_truth.resolve()}`\n"
        f"- 选取散射微元数：{len(scatterers)}\n"
        f"- 距离范围：{summary['scatterer_range_min_m']:.3f}～{summary['scatterer_range_max_m']:.3f} m\n"
        f"- 斜距变化率范围：{summary['scatterer_rate_min_mps']:.6f}～{summary['scatterer_rate_max_mps']:.6f} m/s\n\n"
        "`figures/range_doppler.png` 的亮区不再对应单个微元，而是多个微元的相干叠加。"
        "分析时先看距离方向是否出现展宽或多个峰，再看速度方向是否出现展宽；当前每个"
        "微元使用相同幅度和同相阵列响应，因此结果是受控回归，不是经过海面反射率校准的"
        "绝对海杂波功率。\n\n"
        f"最强峰：{result.peak_range_m:.3f} m，{result.peak_velocity_mps:.6f} m/s。\n\n"
        "下一步是加入局部掠射角、可见性、距离衰减和散射权重，并比较不同 Hs 工况的"
        "Range-Doppler 展宽。\n",
        encoding="utf-8",
    )
    return output_run


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-truth", type=Path, required=True)
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--stride", type=int, default=20)
    parser.add_argument("--max-scatterers", type=int, default=32)
    parser.add_argument("--time-index", type=int, default=0)
    args = parser.parse_args()
    output = run_multi_scatterer(
        input_truth=args.input_truth.resolve(),
        results_root=args.results_root.resolve(),
        run_id=args.run_id,
        stride=args.stride,
        max_scatterers=args.max_scatterers,
        time_index=args.time_index,
    )
    print(f"Generated V0.3.2 multi-scatterer run into {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
