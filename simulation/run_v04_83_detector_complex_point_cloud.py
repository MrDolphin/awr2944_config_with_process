"""Formal V0.4.83 detector-to-complex-bin-AoA point-cloud stage."""

from __future__ import annotations

import argparse, json
from pathlib import Path

from simulation.run_v04_72_detector_point_cloud import run as run_v072


def run(input_root: Path, geometry: Path, output: Path, max_points_per_group: int = 256) -> dict:
    summary = run_v072(input_root, geometry, output, max_points_per_group)
    summary.update({"status": "completed_detector_complex_bin_point_cloud", "aoa_source": "per_detection_complex_range_doppler_bin", "point_cloud_schema": "case/scenario/detector/frame/range/velocity/azimuth/elevation/x/y/z/power/noise/threshold", "hardware_validated": False})
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.83 检测器→复数 bin AoA→海杂波三维点云\n\n"
        "本阶段把每个检测器保留的距离-多普勒单元直接映射到对应的 4×4 复数通道矩阵，逐点估计方位/俯仰，再计算雷达坐标系 XYZ。输出字段包含检测器、CFAR 场景、距离、速度、角度、功率、噪声、门限和三维坐标。\n\n"
        f"海况数量：{summary['case_count']}；CFAR 场景数量：{summary['scenario_count']}；检测器：{summary['detectors']}；每组最多保存 {summary['max_points_per_group']} 点。\n\n"
        "## 解释方法\n\n"
        "先比较 `detection_count` 判断检测规则保留了多少距离-多普勒单元，再比较 `azimuth_deg/elevation_deg` 的均值和标准差判断点云空间偏置，最后查看 `x_m/y_m/z_m` 的距离投影。`stored_point_count` 只是功率最高点的保存上限，不等于总检测数。\n\n"
        "## 证据边界\n\n"
        "输入是 MATLAB 合成 HDF5，阵列坐标是候选 mapping，OS-CFAR 是探索性实现，通道顺序和 TI 校准未验证；因此本阶段是软件管线验证，不是实测 AoA 或真实海杂波虚警率。\n", encoding="utf-8")
    return summary


def main():
    p=argparse.ArgumentParser(); p.add_argument("--input-root",type=Path,required=True); p.add_argument("--geometry",type=Path,required=True); p.add_argument("--output",type=Path,required=True); p.add_argument("--max-points-per-group",type=int,default=256); a=p.parse_args(); print(json.dumps(run(a.input_root.resolve(),a.geometry.resolve(),a.output.resolve(),a.max_points_per_group),indent=2,ensure_ascii=False))


if __name__ == "__main__": main()
