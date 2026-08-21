"""Run V0.3.3 weighted multi-scatterer sea-surface echo regression."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil

import h5py
import numpy as np

from simulation.artifacts import create_run_directory
from simulation.run_v03 import _plot_range_doppler
from simulation.v03 import (
    FmcwConfig,
    compute_controlled_scatterer_weights,
    generate_multi_scatterer_iq,
    process_range_doppler,
)


def run_weighted_multi_scatterer(*, input_truth: Path, results_root: Path,
                                 run_id: str, stride: int = 20,
                                 max_scatterers: int = 64, time_index: int = 0,
                                 front_face_only: bool = True) -> Path:
    with h5py.File(input_truth, "r") as handle:
        ranges = np.asarray(handle["/truth/slant_range_m"][time_index], dtype=float)
        rates = np.asarray(handle["/truth/slant_range_rate_mps"][time_index], dtype=float)
        grazing = np.asarray(handle["/truth/grazing_angle_deg"][time_index], dtype=float)
        case_id = str(handle.attrs["case_id"])
        source_producer = str(handle.attrs["producer"])
    config = FmcwConfig()
    selected: list[tuple[int, int]] = []
    step = max(1, stride)
    for y_index in range(0, ranges.shape[0], step):
        for x_index in range(0, ranges.shape[1], step):
            if len(selected) >= max_scatterers:
                break
            if 0.0 < ranges[y_index, x_index] < config.max_range_m:
                selected.append((y_index, x_index))
        if len(selected) >= max_scatterers:
            break
    if not selected:
        raise ValueError("no scatterers fall inside the configured unambiguous range")
    r = np.asarray([ranges[y, x] for y, x in selected])
    v = np.asarray([rates[y, x] for y, x in selected])
    g = np.asarray([grazing[y, x] for y, x in selected])
    weights = compute_controlled_scatterer_weights(r, g, front_face_only=front_face_only)
    scatterers = list(zip(r.tolist(), v.tolist(), weights.tolist()))
    iq = generate_multi_scatterer_iq(config, scatterers)
    result = process_range_doppler(iq, config)
    output_run = create_run_directory(results_root, producer="python",
        stage_id="v03_complex_echo_range_doppler", run_id=run_id)
    source_run = input_truth.parent.parent
    for name in ("run_config.json", "radar_profile.cfg"):
        source = source_run / name
        if source.is_file():
            shutil.copy2(source, output_run / name)
    scatterer_table = np.column_stack((r, v, g, weights))
    with h5py.File(output_run / "data" / "weighted_multi_scatterer.h5", "w") as handle:
        handle.attrs["schema_version"] = "awr2944p-complex-echo-v0.3.3"
        handle.attrs["producer"] = "python"
        handle.attrs["weight_model"] = "normalized_inverse_range_squared_front_face_proxy"
        handle.attrs["global_ray_occlusion"] = False
        handle.attrs["source_truth"] = str(input_truth.resolve())
        handle.create_dataset("/radar/iq", data=iq, compression="gzip")
        handle.create_dataset("/range_doppler/power_linear", data=result.power_linear)
        handle.create_dataset("/axes/range_m", data=result.range_axis_m)
        handle.create_dataset("/axes/velocity_mps", data=result.velocity_axis_mps)
        handle.create_dataset("/truth/scatterers", data=scatterer_table,
                              dtype="f8", compression="gzip")
    _plot_range_doppler(result, output_run / "figures" / "range_doppler.png",
                        title="V0.3.3 weighted sea-facet Range-Doppler")
    summary = {
        "case_id": case_id, "source_producer": source_producer,
        "source_truth": str(input_truth.resolve()), "scatterer_count": len(selected),
        "front_face_only": front_face_only, "global_ray_occlusion": False,
        "weight_model": "normalized_inverse_range_squared_front_face_proxy",
        "positive_weight_fraction": float(np.mean(weights > 0.0)),
        "range_m": {"min": float(np.min(r)), "max": float(np.max(r))},
        "grazing_angle_deg": {"min": float(np.min(g)), "max": float(np.max(g))},
        "peak_range_m": result.peak_range_m, "peak_velocity_mps": result.peak_velocity_mps,
        "range_resolution_m": config.range_resolution_m,
        "velocity_resolution_mps": config.velocity_resolution_mps,
    }
    (output_run / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output_run / "output_analysis.md").write_text(
        "# V0.3.3 输出分析\n\n"
        f"- 输入：V0.2 truth `{input_truth.resolve()}`，工况 `{case_id}`，时间索引 `{time_index}`。\n"
        f"- 散射微元数：{len(selected)}；距离范围：{np.min(r):.3f}～{np.max(r):.3f} m。\n"
        f"- 局部掠射角范围：{np.min(g):.3f}～{np.max(g):.3f}°；正面照射微元比例：{np.mean(weights > 0):.1%}。\n\n"
        "## 图和数据怎么读\n\n"
        "`figures/range_doppler.png` 横轴是斜距、纵轴是径向速度，颜色越亮表示该距离-速度单元的相对功率越高。"
        "先看亮区是否由单一亮点变成多个距离峰或距离展宽，再看速度方向是否出现展宽；峰值只能说明离散 FFT 网格中的最强单元。\n\n"
        "本版本的幅度是归一化 `1/R²`，并在 `grazing_angle_deg <= 0` 时将局部背向微元权重置零。"
        "这是局部正面照射代理，不是全局射线遮挡，也不是经过海水介电常数、极化和粗糙面散射校准的 RCS。"
        "因此可用于比较权重模型前后的相对变化，不可直接作为 AWR2944P 实测杂波功率或探测距离。\n\n"
        f"最强峰：{result.peak_range_m:.3f} m，{result.peak_velocity_mps:.6f} m/s。"
        f"距离分辨率 {config.range_resolution_m:.4f} m，速度分辨率 {config.velocity_resolution_mps:.4f} m/s。\n\n"
        "下一步验收：对 Hs=0.30/0.85/1.00 m 重复同一权重模型，比较距离展宽、速度展宽和正面微元比例；"
        "再接入实测 DCA1000 IQ 前，需增加散射系数、极化和平台姿态标定。\n",
        encoding="utf-8")
    return output_run


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-truth", type=Path, required=True)
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--stride", type=int, default=20)
    parser.add_argument("--max-scatterers", type=int, default=64)
    parser.add_argument("--time-index", type=int, default=0)
    parser.add_argument("--include-back-face", action="store_true")
    args = parser.parse_args()
    output = run_weighted_multi_scatterer(
        input_truth=args.input_truth.resolve(), results_root=args.results_root.resolve(),
        run_id=args.run_id, stride=args.stride, max_scatterers=args.max_scatterers,
        time_index=args.time_index, front_face_only=not args.include_back_face)
    print(f"Generated V0.3.3 weighted multi-scatterer run into {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
