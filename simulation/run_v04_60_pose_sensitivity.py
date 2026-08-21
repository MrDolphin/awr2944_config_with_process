"""Scan installation-pose uncertainty around a board-to-radar candidate."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_36_pose_transform import rotation_matrix


def _read_regions(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _pose_points(regions: list[dict[str, str]], angles: tuple[float, float, float], translation: np.ndarray) -> dict[str, np.ndarray]:
    rotation = rotation_matrix(*angles)
    points: dict[str, np.ndarray] = {}
    for row in regions:
        board = np.array([float(row["center_x_relative_to_origin_mm"]), float(row["center_y_relative_to_origin_mm"]), 0.0])
        points[row["antenna"]] = rotation @ board + translation
    return points


def run(regions_path: Path, pose_path: Path, output: Path, roll_offsets: tuple[float, ...] = (-2.0, 0.0, 2.0), pitch_offsets: tuple[float, ...] = (-2.0, 0.0, 2.0), yaw_offsets: tuple[float, ...] = (-5.0, 0.0, 5.0)) -> dict:
    regions = _read_regions(regions_path)
    pose = json.loads(pose_path.read_text(encoding="utf-8"))
    nominal = tuple(float(pose[key]) for key in ("roll_deg", "pitch_deg", "yaw_deg"))
    translation = np.asarray(pose.get("translation_mm", [0.0, 0.0, 0.0]), dtype=float)
    nominal_points = _pose_points(regions, nominal, translation)
    nominal_normal = rotation_matrix(*nominal) @ np.array([0.0, 0.0, 1.0])
    nominal_vertical_angle = float(np.rad2deg(np.arccos(np.clip(abs(nominal_normal[2]), -1.0, 1.0))))
    sensitivity_rows: list[dict] = []
    displacement_rows: list[dict] = []
    for dr in roll_offsets:
        for dp in pitch_offsets:
            for dy in yaw_offsets:
                angles = (nominal[0] + dr, nominal[1] + dp, nominal[2] + dy)
                rotation = rotation_matrix(*angles)
                normal = rotation @ np.array([0.0, 0.0, 1.0])
                vertical_angle = float(np.rad2deg(np.arccos(np.clip(abs(normal[2]), -1.0, 1.0))))
                points = _pose_points(regions, angles, translation)
                displacements = {name: float(np.linalg.norm(points[name] - nominal_points[name])) for name in points}
                sensitivity_rows.append({
                    "roll_offset_deg": dr,
                    "pitch_offset_deg": dp,
                    "yaw_offset_deg": dy,
                    "roll_deg": angles[0],
                    "pitch_deg": angles[1],
                    "yaw_deg": angles[2],
                    "normal_x": normal[0],
                    "normal_y": normal[1],
                    "normal_z": normal[2],
                    "normal_to_vertical_abs_angle_deg": vertical_angle,
                    "max_region_displacement_mm": max(displacements.values()),
                    "mean_region_displacement_mm": float(np.mean(list(displacements.values()))),
                })
                for name, distance in displacements.items():
                    displacement_rows.append({"roll_offset_deg": dr, "pitch_offset_deg": dp, "yaw_offset_deg": dy, "antenna": name, "displacement_from_nominal_mm": distance})
    output.mkdir(parents=True, exist_ok=True)
    _write_csv(output / "pose_sensitivity.csv", sensitivity_rows, list(sensitivity_rows[0]))
    _write_csv(output / "region_displacement.csv", displacement_rows, list(displacement_rows[0]))
    summary = {
        "status": "completed_pose_sensitivity_scan",
        "scenario_count": len(sensitivity_rows),
        "region_count": len(regions),
        "nominal_rpy_deg": list(nominal),
        "nominal_board_normal_radar": nominal_normal.tolist(),
        "nominal_normal_to_vertical_abs_angle_deg": nominal_vertical_angle,
        "roll_offsets_deg": list(roll_offsets),
        "pitch_offsets_deg": list(pitch_offsets),
        "yaw_offsets_deg": list(yaw_offsets),
        "max_displacement_over_scan_mm": max(row["max_region_displacement_mm"] for row in sensitivity_rows),
        "pose_is_measured": bool(pose.get("installation_pose_confirmed", False)),
        "phase_center_confirmed": False,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    worst = max(sensitivity_rows, key=lambda row: row["max_region_displacement_mm"])
    lines = [
        "# V0.4.60 安装姿态敏感性扫描",
        "",
        f"扫描场景数：{len(sensitivity_rows)}；RF 区域数：{len(regions)}。",
        f"名义 RPY：{nominal[0]:.3f}°, {nominal[1]:.3f}°, {nominal[2]:.3f}°；名义板面法向与竖直夹角：{nominal_vertical_angle:.3f}°。",
        f"扫描范围：roll {min(roll_offsets):.1f}°～{max(roll_offsets):.1f}°，pitch {min(pitch_offsets):.1f}°～{max(pitch_offsets):.1f}°，yaw {min(yaw_offsets):.1f}°～{max(yaw_offsets):.1f}°。",
        "",
        "## 结果解释",
        "",
        "每一行对应一个姿态误差组合。`max_region_displacement_mm` 表示该姿态下任一 RF 铜区候选相对于名义姿态的最大坐标移动；`normal_to_vertical_abs_angle_deg` 表示板面法向偏离竖直方向的角度。刚体旋转不会改变阵元间距离，但会改变它们在雷达坐标系的 x/y/z 分量。",
        "",
        f"本次扫描中最大候选坐标移动为 {worst['max_region_displacement_mm']:.4f} mm，出现在 roll/pitch/yaw 偏置为 ({worst['roll_offset_deg']:.1f}°, {worst['pitch_offset_deg']:.1f}°, {worst['yaw_offset_deg']:.1f}°) 的场景。",
        "",
        "## 证据边界",
        "",
        "扫描偏置是工程敏感性假设，不是 IMU 或机械测量结果；RF 区域仍是铜区几何中心候选，不是电气相位中心。后续应将实测 IMU 外参替换当前扫描范围，并用已知角度目标验证 AoA 偏差。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--regions", type=Path, required=True)
    parser.add_argument("--pose", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.regions.resolve(), args.pose.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
