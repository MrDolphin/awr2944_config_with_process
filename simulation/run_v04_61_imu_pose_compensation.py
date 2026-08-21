"""Compare fixed-pose and time-varying IMU pose point-cloud projections."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from simulation.run_v04_36_pose_transform import rotation_matrix


def _read(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _angle(point: np.ndarray) -> tuple[float, float, float]:
    radius = float(np.linalg.norm(point))
    azimuth = float(np.rad2deg(np.arctan2(point[0], point[1])))
    elevation = float(np.rad2deg(np.arctan2(point[2], np.hypot(point[0], point[1]))))
    return radius, azimuth, elevation


def _write(path: Path, rows: list[dict]) -> None:
    fields = list(rows[0]) if rows else ["frame"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def run(point_cloud_path: Path, imu_path: Path, output: Path, nominal_pose_path: Path | None = None) -> dict:
    points = _read(point_cloud_path)
    imu_rows = _read(imu_path)
    imu = {int(row["frame"]): row for row in imu_rows}
    if not points:
        raise ValueError("point cloud is empty")
    if any(int(row["frame"]) not in imu for row in points):
        raise ValueError("IMU trace must contain every point-cloud frame")
    if nominal_pose_path:
        nominal_payload = json.loads(nominal_pose_path.read_text(encoding="utf-8"))
        nominal = tuple(float(nominal_payload[key]) for key in ("roll_deg", "pitch_deg", "yaw_deg"))
    else:
        nominal = (90.0, 0.0, 0.0)
    nominal_rotation = rotation_matrix(*nominal)
    rows: list[dict] = []
    for row in points:
        frame = int(row["frame"])
        radar_point = np.array([float(row["x_m"]), float(row["y_m"]), float(row["z_m"])])
        imu_row = imu[frame]
        measured_angles = tuple(float(imu_row[key]) for key in ("roll_deg", "pitch_deg", "yaw_deg"))
        compensated_point = rotation_matrix(*measured_angles) @ radar_point
        fixed_point = nominal_rotation @ radar_point
        fixed_range, fixed_az, fixed_el = _angle(fixed_point)
        compensated_range, compensated_az, compensated_el = _angle(compensated_point)
        rows.append({
            "case_id": row.get("case_id", ""), "frame": frame, "time_s": float(imu_row["time_s"]),
            "roll_deg": measured_angles[0], "pitch_deg": measured_angles[1], "yaw_deg": measured_angles[2],
            "radar_x_m": radar_point[0], "radar_y_m": radar_point[1], "radar_z_m": radar_point[2],
            "fixed_x_m": fixed_point[0], "fixed_y_m": fixed_point[1], "fixed_z_m": fixed_point[2],
            "compensated_x_m": compensated_point[0], "compensated_y_m": compensated_point[1], "compensated_z_m": compensated_point[2],
            "fixed_azimuth_deg": fixed_az, "fixed_elevation_deg": fixed_el,
            "compensated_azimuth_deg": compensated_az, "compensated_elevation_deg": compensated_el,
            "azimuth_change_deg": compensated_az - fixed_az,
            "elevation_change_deg": compensated_el - fixed_el,
            "position_change_m": float(np.linalg.norm(compensated_point - fixed_point)),
            "range_difference_m": compensated_range - fixed_range,
        })
    output.mkdir(parents=True, exist_ok=True)
    _write(output / "point_cloud_pose_comparison.csv", rows)
    max_row = max(rows, key=lambda item: item["position_change_m"])
    summary = {
        "status": "completed_imu_pose_compensation_comparison",
        "point_count": len(rows),
        "frame_count": len({row["frame"] for row in rows}),
        "nominal_pose_deg": list(nominal),
        "imu_frames_used": sorted(imu),
        "max_position_change_m": max_row["position_change_m"],
        "max_abs_azimuth_change_deg": max(abs(row["azimuth_change_deg"]) for row in rows),
        "max_abs_elevation_change_deg": max(abs(row["elevation_change_deg"]) for row in rows),
        "source_is_hardware_imu": False,
        "point_cloud_source_is_hardware": False,
        "phase_center_confirmed": False,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "# V0.4.61 IMU 姿态补偿前后点云比较",
        "",
        f"点数：{len(rows)}；帧数：{summary['frame_count']}；使用的 IMU 帧：{summary['imu_frames_used']}。",
        f"固定姿态 RPY：{nominal[0]:.3f}°, {nominal[1]:.3f}°, {nominal[2]:.3f}°。",
        f"最大坐标变化：{summary['max_position_change_m']:.6f} m；最大方位变化：{summary['max_abs_azimuth_change_deg']:.6f}°；最大俯仰变化：{summary['max_abs_elevation_change_deg']:.6f}°。",
        "",
        "## 如何解释",
        "",
        "固定姿态结果把所有帧都按同一个安装姿态投影；补偿结果按对应 frame 的 IMU RPY 旋转。两者之差表示船体姿态变化对点云坐标和角度的影响，不是目标真实运动，也不是雷达 AoA 算法误差。",
        "",
        "## 证据边界",
        "",
        "本次 IMU 文件和点云均为合成/历史仿真输入，不能作为船上实测姿态补偿性能。接入真实 IMU 时必须确认时间戳同步、坐标轴方向、角度单位和初始安装外参。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--point-cloud", type=Path, required=True)
    parser.add_argument("--imu", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--nominal-pose", type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.point_cloud.resolve(), args.imu.resolve(), args.output.resolve(), args.nominal_pose.resolve() if args.nominal_pose else None), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
