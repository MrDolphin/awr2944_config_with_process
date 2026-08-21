"""Transform PCB/CAD coordinates into an explicit radar-frame pose candidate.

This stage is deliberately a pose *candidate* tool.  It does not infer the
physical phase centre or installation attitude from a PCB file.
"""

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


def _read_outline(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _point_from_region(row: dict[str, str]) -> np.ndarray:
    return np.array(
        [
            float(row["center_x_relative_to_origin_mm"]),
            float(row["center_y_relative_to_origin_mm"]),
            0.0,
        ],
        dtype=float,
    )


def _point_from_outline(row: dict[str, str]) -> np.ndarray:
    return np.array([float(row["x_relative_to_origin_mm"]), float(row["y_relative_to_origin_mm"]), 0.0], dtype=float)


def _transform(point_mm: np.ndarray, rotation: np.ndarray, translation_mm: np.ndarray) -> np.ndarray:
    return rotation @ point_mm + translation_mm


def _write_csv(path: Path, rows: list[dict], empty_fields: list[str] | None = None) -> None:
    if not rows:
        if empty_fields:
            path.write_text(",".join(empty_fields) + "\n", encoding="utf-8")
        else:
            path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run(regions_path: Path, outline_path: Path, pose_path: Path, output: Path) -> dict:
    regions = _read_regions(regions_path)
    outline = _read_outline(outline_path)
    pose = json.loads(pose_path.read_text(encoding="utf-8"))
    angles = tuple(float(pose[key]) for key in ("roll_deg", "pitch_deg", "yaw_deg"))
    translation = np.asarray(pose.get("translation_mm", [0.0, 0.0, 0.0]), dtype=float)
    if translation.shape != (3,):
        raise ValueError("translation_mm must contain exactly three values")
    rotation = rotation_matrix(*angles)
    board_normal = rotation @ np.array([0.0, 0.0, 1.0])
    vertical_angle = float(np.rad2deg(np.arccos(np.clip(abs(board_normal[2]), -1.0, 1.0))))

    rf_rows: list[dict] = []
    for row in regions:
        board_point = _point_from_region(row)
        radar_point = _transform(board_point, rotation, translation)
        rf_rows.append(
            {
                "antenna": row["antenna"],
                "board_x_mm": board_point[0],
                "board_y_mm": board_point[1],
                "board_z_mm": board_point[2],
                "radar_x_mm": radar_point[0],
                "radar_y_mm": radar_point[1],
                "radar_z_mm": radar_point[2],
                "coordinate_status": "board_relative_transformed_pose_candidate",
            }
        )

    outline_rows: list[dict] = []
    for row in outline:
        board_point = _point_from_outline(row)
        radar_point = _transform(board_point, rotation, translation)
        outline_rows.append(
            {
                "vertex_index": row["vertex_index"],
                "radar_x_mm": radar_point[0],
                "radar_y_mm": radar_point[1],
                "radar_z_mm": radar_point[2],
            }
        )

    # Pairwise spacing is useful for a geometry sanity check, but is not an
    # antenna phase-centre or measured radiation-pattern result.
    baselines: list[dict] = []
    points = {row["antenna"]: np.array([row["radar_x_mm"], row["radar_y_mm"], row["radar_z_mm"]], dtype=float) for row in rf_rows}
    names = sorted(points)
    for index, left in enumerate(names):
        for right in names[index + 1 :]:
            delta = points[right] - points[left]
            baselines.append(
                {
                    "antenna_a": left,
                    "antenna_b": right,
                    "dx_mm": delta[0],
                    "dy_mm": delta[1],
                    "dz_mm": delta[2],
                    "distance_mm": float(np.linalg.norm(delta)),
                }
            )

    output.mkdir(parents=True, exist_ok=True)
    _write_csv(output / "rf_regions_radar_coordinates.csv", rf_rows)
    _write_csv(output / "board_outline_radar_coordinates.csv", outline_rows)
    _write_csv(output / "array_baseline_metrics.csv", baselines, ["antenna_a", "antenna_b", "dx_mm", "dy_mm", "dz_mm", "distance_mm"])
    schema = {
        "coordinate_frames": {
            "board": "x=PCB local x, y=PCB local y, z=board normal candidate",
            "radar": "x=forward, y=left, z=up",
        },
        "rotation_convention": "Rz(yaw) @ Ry(pitch) @ Rx(roll)",
        "pose_source": pose.get("pose_source", "candidate_not_measured"),
        "installation_pose_confirmed": bool(pose.get("installation_pose_confirmed", False)),
        "phase_center_confirmed": False,
        "measurement_required": True,
    }
    (output / "coordinate_schema.json").write_text(json.dumps(schema, indent=2, ensure_ascii=False), encoding="utf-8")
    summary = {
        "status": "completed_board_to_radar_pose_candidate",
        "region_count": len(rf_rows),
        "outline_vertex_count": len(outline_rows),
        "roll_deg": angles[0],
        "pitch_deg": angles[1],
        "yaw_deg": angles[2],
        "translation_mm": translation.tolist(),
        "board_normal_radar": board_normal.tolist(),
        "board_normal_to_vertical_abs_angle_deg": vertical_angle,
        "pose_source": pose.get("pose_source", "candidate_not_measured"),
        "installation_pose_confirmed": bool(pose.get("installation_pose_confirmed", False)),
        "phase_center_confirmed": False,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# V0.4.59 PCB 板坐标到雷达坐标姿态候选",
        "",
        f"- RF 区域数：{len(rf_rows)}",
        f"- 板框顶点数：{len(outline_rows)}",
        f"- RPY：{angles[0]:.3f}°, {angles[1]:.3f}°, {angles[2]:.3f}°",
        f"- 平移：({translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}) mm",
        f"- 板面法向（雷达系）：({board_normal[0]:.6f}, {board_normal[1]:.6f}, {board_normal[2]:.6f})",
        f"- 法向与竖直夹角：{vertical_angle:.3f}°",
        "",
        "## 如何解释",
        "",
        "本阶段把 PCB 相对坐标按显式 RPY 和平移变换到雷达坐标系，并输出三维坐标。法向与竖直夹角接近 90° 表示板面近似垂直；这只是输入 pose 的几何结果，不是从 PCB 文件自动推断出的真实安装角。",
        "",
        "## 证据边界",
        "",
        "RF 区域仍是铜区几何中心候选，不是电气相位中心；安装姿态、板面法向、相位中心和真实 AoA 仍需机械测量、装配基准或角反射器/已知角度实测确认。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--regions", type=Path, required=True)
    parser.add_argument("--outline", type=Path, required=True)
    parser.add_argument("--pose", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.regions.resolve(), args.outline.resolve(), args.pose.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
