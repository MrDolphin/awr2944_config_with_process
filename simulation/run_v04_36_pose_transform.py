"""Apply candidate board-to-radar/world poses to PCB RF-region coordinates."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    roll, pitch, yaw = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return rz @ ry @ rx


POSES = {
    "identity": (0.0, 0.0, 0.0),
    "board_vertical_pitch_plus90": (0.0, 90.0, 0.0),
    "board_vertical_pitch_minus90": (0.0, -90.0, 0.0),
    "board_vertical_roll_plus90": (90.0, 0.0, 0.0),
    "board_vertical_roll_minus90": (-90.0, 0.0, 0.0),
    "yaw_plus90": (0.0, 0.0, 90.0),
}


def run(regions_csv: Path, output: Path, translation_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> dict:
    with regions_csv.open(encoding="utf-8", newline="") as handle:
        regions = list(csv.DictReader(handle))
    output.mkdir(parents=True, exist_ok=True)
    transformed_rows = []
    pose_rows = []
    vertical = np.array([0.0, 0.0, 1.0])
    for pose_name, angles in POSES.items():
        rotation = rotation_matrix(*angles)
        normal = rotation @ vertical
        normal_angle = float(np.rad2deg(np.arccos(np.clip(abs(normal[2]), -1.0, 1.0))))
        pose_rows.append({"pose_name": pose_name, "roll_deg": angles[0], "pitch_deg": angles[1], "yaw_deg": angles[2], "normal_x": normal[0], "normal_y": normal[1], "normal_z": normal[2], "normal_to_vertical_abs_angle_deg": normal_angle, "pose_status": "pose_candidate_not_measured"})
        for region in regions:
            point = np.array([float(region["center_x_relative_to_origin_mm"]), float(region["center_y_relative_to_origin_mm"]), 0.0])
            transformed = rotation @ point + np.asarray(translation_mm)
            transformed_rows.append({"pose_name": pose_name, "antenna": region["antenna"], "board_x_mm": point[0], "board_y_mm": point[1], "radar_or_world_x_mm": transformed[0], "radar_or_world_y_mm": transformed[1], "radar_or_world_z_mm": transformed[2], "coordinate_status": "board_relative_transformed_pose_candidate"})
    with (output / "pose_candidates.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(pose_rows[0])); writer.writeheader(); writer.writerows(pose_rows)
    with (output / "rf_regions_pose_candidates.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(transformed_rows[0])); writer.writeheader(); writer.writerows(transformed_rows)
    schema = {"coordinate_chain": ["pcb_board_relative_mm", "board_to_radar_rotation", "board_to_radar_translation_mm", "radar_or_world_candidate_mm"], "rotation_convention": "Rz(yaw) @ Ry(pitch) @ Rx(roll)", "translation_mm": list(translation_mm), "pose_measurement_required": True, "phase_center_confirmed": False}
    (output / "pose_transform_schema.json").write_text(json.dumps(schema, indent=2, ensure_ascii=False), encoding="utf-8")
    summary = {"status": "completed_pose_transform_candidates", "region_count": len(regions), "pose_count": len(POSES), "translation_mm": list(translation_mm), "installation_pose_confirmed": False, "phase_center_confirmed": False, "vertical_board_pose_is_candidate_only": True}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.36 板框到雷达/船体姿态候选", "", f"- RF 区域数：{len(regions)}", f"- 姿态候选数：{len(POSES)}", "- 实测安装姿态：未提供", "", "## 角度含义", "", "`normal_to_vertical_abs_angle_deg=0°` 表示板面法向与竖直方向平行；这意味着板面近似水平。`90°` 表示板面法向近似水平；这意味着板面近似垂直。这里仅描述候选姿态，不代表实际安装。", "", "## 边界", "", "没有机械安装测量、船体坐标基准或 IMU 姿态时，不能从 PCB 文件推出雷达相对地面 90°、俯仰角或方位角。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--regions", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--translation-mm", nargs=3, type=float, default=(0.0, 0.0, 0.0))
    args = parser.parse_args()
    print(json.dumps(run(args.regions.resolve(), args.output.resolve(), tuple(args.translation_mm)), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
