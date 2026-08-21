"""Process one known-angle capture against PCB coordinate-transform candidates."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_50_dca_reference_calibration import load_capture, range_doppler
from simulation.run_v04_86_geometry_comparison import _estimate, _load_candidate
from simulation.run_v04_89_coordinate_transform_stress import transform_candidates
from simulation.run_v04_90_known_angle_capture_contract import validate_manifest
from simulation.v03 import FmcwConfig


def process(manifest_path: Path, candidate_csv: Path, output: Path) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    root = manifest_path.parent
    capture_path = root / str(manifest["capture_file"])
    capture_meta = manifest["capture"]
    chirps = int(capture_meta.get("chirps", 64))
    samples = int(capture_meta.get("samples_per_chirp", 128))
    rx_count = int(capture_meta.get("rx_count", 4))
    tx_sequence = tuple(int(value) for value in capture_meta.get("tdm_tx_sequence", [0, 1, 2, 3]))
    expected_range = float(manifest["target"]["range_m"])
    expected_velocity = float(manifest["target"].get("radial_velocity_mps", 0.0))
    truth_az = float(manifest["target"]["azimuth_deg"])
    truth_el = float(manifest["target"]["elevation_deg"])

    iq, source = load_capture(capture_path, chirps=chirps, samples_per_chirp=samples, rx_count=rx_count)
    config = FmcwConfig(samples_per_chirp=samples, chirps_per_frame=max(1, iq.shape[0] // len(tx_sequence)))
    spectrum, power, ranges, velocities, _ = range_doppler(iq, config, tx_sequence)
    range_index = int(np.argmin(np.abs(ranges - expected_range)))
    velocity_index = int(np.argmin(np.abs(velocities - expected_velocity)))
    channel = spectrum[velocity_index, range_index, :, :]
    candidate_positions = transform_candidates(_load_candidate(candidate_csv))
    solver_az = np.arange(-60.0, 60.01, 1.0)
    solver_el = np.arange(-20.0, 20.01, 1.0)
    rows = []
    for name, positions in candidate_positions.items():
        estimated_az, estimated_el, score = _estimate(config, channel, positions[0], positions[1], solver_az, solver_el)
        rows.append({
            "transform": name,
            "truth_azimuth_deg": truth_az,
            "truth_elevation_deg": truth_el,
            "estimated_azimuth_deg": estimated_az,
            "estimated_elevation_deg": estimated_el,
            "azimuth_error_deg": estimated_az - truth_az,
            "elevation_error_deg": estimated_el - truth_el,
            "combined_error_deg": float(np.hypot(estimated_az - truth_az, estimated_el - truth_el)),
            "aoa_score": score,
        })
    best = min(rows, key=lambda row: row["combined_error_deg"])
    readiness = validate_manifest(manifest_path, root)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "known_angle_transform_results.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    summary = {
        "status": "completed_known_angle_capture_processing",
        "manifest": str(manifest_path.resolve()),
        "capture": str(capture_path.resolve()),
        "source_format": source["source_format"],
        "decoded_iq_shape": list(iq.shape),
        "target_range_m": expected_range,
        "selected_range_m": float(ranges[range_index]),
        "target_velocity_mps": expected_velocity,
        "selected_velocity_mps": float(velocities[velocity_index]),
        "selected_range_index": range_index,
        "selected_velocity_index": velocity_index,
        "best_transform_by_known_angle": best["transform"],
        "best_combined_error_deg": best["combined_error_deg"],
        "results": rows,
        "manifest_readiness": readiness,
        "hardware_aoa_validated": False,
        "evidence_status": "real_capture_candidate_only_until_channel_order_calibration_and_phase_center_are_verified",
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.91 已知角 DCA1000 IQ 坐标候选处理", "", f"输入文件：`{capture_path}`", f"解码 IQ 形状：`{list(iq.shape)}`", f"目标真值：方位 `{truth_az:.3f}°`，俯仰 `{truth_el:.3f}°`，距离 `{expected_range:.3f} m`，速度 `{expected_velocity:.3f} m/s`。", "", "## 目标 Range-Doppler 单元", "", f"选择距离 `{ranges[range_index]:.6f} m`（索引 {range_index}），速度 `{velocities[velocity_index]:.6f} m/s`（索引 {velocity_index}）。", "", "## 四种坐标候选", "", "| 候选 | 估计方位(°) | 估计俯仰(°) | 方位误差(°) | 俯仰误差(°) | 综合误差(°) |", "|---|---:|---:|---:|---:|---:|"]
    lines.extend(f"| {row['transform']} | {row['estimated_azimuth_deg']:.3f} | {row['estimated_elevation_deg']:.3f} | {row['azimuth_error_deg']:.3f} | {row['elevation_error_deg']:.3f} | {row['combined_error_deg']:.3f} |" for row in rows)
    lines += ["", f"当前已知角数据下误差最小候选为 `{best['transform']}`。", "", "## 证据边界", "", "这一步只说明候选坐标对当前采集文件的解释差异。必须同时确认 DCA1000 wire order、TDM TX 顺序、通道顺序、TI 校准矩阵、PCB 安装姿态和电气相位中心，才能将其升级为真实硬件 AoA 结论。", "", f"manifest 就绪性状态：`{readiness['status']}`。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--candidate-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(process(args.manifest.resolve(), args.candidate_csv.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
