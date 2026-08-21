"""Calibrated-power contract and target/sea-clutter Doppler-overlap sweep."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_38_pcb_array_comparison import model_positions
from simulation.run_v04_48_physical_target_model import local_cfar_noise, run_case
from simulation.v03 import FmcwConfig


DEFAULT_CALIBRATION = {
    "schema_version": "awr2944p-power-calibration-v0.4.49",
    "status": "awaiting_dca1000_reference_capture",
    "reference_range_m": 20.0,
    "reference_rcs_m2": 1.0,
    "reference_power_linear": None,
    "reference_adc_rms": None,
    "reference_target_description": "待用已知距离、已知RCS角反射器或标准目标采集DCA1000原始IQ",
    "required_evidence": ["raw_adc_iq_file", "chirp_profile_cfg", "target_range_truth_m", "target_rcs_m2", "window_and_fft_definition"],
}


def load_calibration(path: Path | None) -> dict:
    if path is None:
        return dict(DEFAULT_CALIBRATION)
    data = json.loads(path.read_text(encoding="utf-8")); merged = dict(DEFAULT_CALIBRATION); merged.update(data); return merged


def overlap_metrics(path: Path, target_range_m: float, target_velocity_mps: float) -> dict:
    with h5py.File(path, "r") as handle:
        power = handle["/range_doppler/power_linear"][...]; ranges = handle["/axes/range_m"][...]; velocities = handle["/axes/velocity_mps"][...]
    ri = int(np.argmin(np.abs(ranges - target_range_m))); di = int(np.argmin(np.abs(velocities - target_velocity_mps)))
    ratios = []
    for frame in power:
        noise = local_cfar_noise(frame, di, ri); ratios.append(float(10.0 * np.log10(max(frame[di, ri], 1e-30) / max(noise, 1e-30))))
    return {"target_range_bin_m": float(ranges[ri]), "target_velocity_bin_mps": float(velocities[di]), "mean_clutter_to_training_db": float(np.mean(ratios)), "max_clutter_to_training_db": float(np.max(ratios))}


def run(input_root: Path, output: Path, calibration: dict) -> dict:
    config = FmcwConfig(samples_per_chirp=128, chirps_per_frame=64); models = model_positions(Path("simulation/hardware/awr2944pev/antgeometry_mapping.csv"), Path("simulation/hardware/awr2944pev/pcb_antenna_regions.csv"), config); positions = models["pcb_centroid_candidate"]
    velocities = (-0.6, -0.3, 0.0, 0.3, 0.6, 1.0); scenarios = [{"scenario_id": f"v{velocity:g}_snr{snr:g}_pfa{pfa:g}", "target_snr_db": snr, "pfa": pfa, "training": (2, 2), "guard": (1, 1)} for velocity in velocities for snr in (0.0, 5.0, 10.0, 15.0) for pfa in (1e-2, 1e-3)]; rows = []
    for case in ("ss2_normal", "ss3_upper"):
        path = input_root / f"{case}_range_doppler.h5"
        for velocity in velocities:
            overlap = overlap_metrics(path, 20.0, velocity)
            for snr in (0.0, 5.0, 10.0, 15.0):
                for pfa in (1e-2, 1e-3):
                    scenario = {"scenario_id": f"v{velocity:g}_snr{snr:g}_pfa{pfa:g}", "target_snr_db": snr, "pfa": pfa, "training": (2, 2), "guard": (1, 1)}
                    result = run_case(path, scenario, positions, config, {"range_m": 20.0, "velocity_mps": velocity, "azimuth_deg": 10.0, "elevation_deg": 2.0, "rcs_m2": 1.0}); result.update({"case_id": case, "target_velocity_truth_mps": velocity, "calibration_status": calibration["status"], **overlap}); rows.append(result)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "doppler_overlap_sweep.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    (output / "calibration_schema.json").write_text(json.dumps(calibration, indent=2, ensure_ascii=False), encoding="utf-8")
    summary = {"status": "completed_calibration_contract_and_doppler_overlap", "input_status": "synthetic_sea_spectrum_not_measured_iq", "hardware_validated": False, "calibration": calibration, "row_count": len(rows), "rows": rows}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.49 校准接口与目标-海杂波 Doppler 重叠", "", "本阶段在 20 m、方位 10°、俯仰 2° 的目标条件下，将目标速度设置为 -0.6/-0.3/0/0.3/0.6/1.0 m/s，比较目标注入后的检测概率、虚警点和目标 AoA；同时记录未注入海杂波在目标 Doppler 单元相对 CFAR 训练噪声的强度。", "", f"校准状态：`{calibration['status']}`。当前没有把任意合成谱单位伪装成 dBm；待 DCA1000 已知目标采集后，填写 calibration_schema.json 中的 reference_power_linear 或 reference_adc_rms。", "", "| 海况 | 目标速度 | SNR | Pfa | 检测概率 | 虚警点/帧 | 海杂波相对训练噪声(dB) |", "|---|---:|---:|---:|---:|---:|---:|"]
    lines.extend(f"| {r['case_id']} | {r['target_velocity_truth_mps']:.1f} | {r['target_snr_db']:.0f} | {r['pfa']:g} | {r['detection_probability']:.3f} | {r['mean_false_alarms_per_frame']:.3f} | {r['mean_clutter_to_training_db']:.2f} |" for r in rows)
    lines += ["", "## 解释", "", "速度接近海面 Doppler 主瓣时，目标所在训练窗的海杂波功率会升高，目标检测概率可能下降或虚警点增加。若不同速度的结果没有变化，说明当前海杂波谱与目标注入尚未产生足够重叠，不能据此断言真实海杂波不影响目标。", "", "## 校准下一步", "", "使用 DCA1000 保存未经 CFAR 的 ADC/IQ，在已知距离和 RCS 目标下测量同一 FFT 窗函数、采样数和通道合成方式的参考功率，再更新 calibration_schema.json；没有这一步，所有功率仍是相对量。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--input-root", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); parser.add_argument("--calibration", type=Path); args = parser.parse_args(); print(json.dumps(run(args.input_root.resolve(), args.output.resolve(), load_calibration(args.calibration)), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
