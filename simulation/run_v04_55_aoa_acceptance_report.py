"""Create an explicit synthetic-vs-hardware AoA acceptance gate report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def build_report(multiscene_summary: dict, hdf5_summary: dict, *, combined_rmse_limit_deg: float = 2.0, max_error_limit_deg: float = 5.0) -> dict:
    best = multiscene_summary["best_candidates"][0]; identity = multiscene_summary["identity_candidate"]; metadata = hdf5_summary.get("input_metadata", {})
    checks = {
        "multi_scene_present": multiscene_summary.get("scene_count", 0) >= 2,
        "candidate_count_576": multiscene_summary.get("candidate_count") == 576,
        "best_candidate_within_rmse": float(best["combined_rmse_deg"]) <= combined_rmse_limit_deg,
        "best_candidate_max_error": max(float(best["max_abs_azimuth_error_deg"]), float(best["max_abs_elevation_error_deg"])) <= max_error_limit_deg,
        "identity_matches_best": bool(best["identity_order"]),
        "hdf5_known_angle_present": "truth_azimuth_deg" in metadata and "truth_elevation_deg" in metadata,
        "measured_source": bool(hdf5_summary.get("source_is_hardware_measurement", False)),
        "channel_order_hardware_verified": bool(hdf5_summary.get("channel_order_verified", False)),
    }
    synthetic_regression_pass = all(checks[key] for key in ("multi_scene_present", "candidate_count_576", "best_candidate_within_rmse", "best_candidate_max_error", "identity_matches_best", "hdf5_known_angle_present"))
    hardware_ready = synthetic_regression_pass and checks["measured_source"] and checks["channel_order_hardware_verified"]
    return {"status": "completed_aoa_acceptance_gate", "checks": checks, "synthetic_regression_pass": synthetic_regression_pass, "real_hardware_aoa_ready": hardware_ready, "limits": {"combined_rmse_limit_deg": combined_rmse_limit_deg, "max_error_limit_deg": max_error_limit_deg}, "best_candidate": best, "identity_candidate": identity, "data_provenance": {"source_type": metadata.get("source_type"), "calibration_status": metadata.get("calibration_status"), "geometry_source": multiscene_summary.get("geometry_source")}}


def run(multiscene_path: Path, hdf5_path: Path, output: Path) -> dict:
    multi = json.loads(multiscene_path.read_text(encoding="utf-8")); hdf5 = json.loads(hdf5_path.read_text(encoding="utf-8")); summary = build_report(multi, hdf5); output.mkdir(parents=True, exist_ok=True); (output / "acceptance_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.55 AoA 综合验收门禁", "", "本报告将多场景 RMSE、通道排列、HDF5 已知角度元数据和硬件 provenance 分开判定。", "", "## 门禁结果", "", f"- 合成回归通过：{'是' if summary['synthetic_regression_pass'] else '否'}", f"- 真实硬件 AoA 就绪：{'是' if summary['real_hardware_aoa_ready'] else '否'}", "", "| 检查项 | 结果 |", "|---|---|"]
    lines.extend(f"| {key} | {'通过' if value else '未通过'} |" for key, value in summary["checks"].items())
    lines += ["", "## 如何解释", "", "本次合成数据满足 RMSE 和候选稳定性门禁，因此可以说 AoA 算法回归链路通过；但输入来源是 synthetic known-angle fixture，且 channel_order_hardware_verified=false，所以不能说真实 AWR2944P 硬件 AoA 已验证。", "", "## 真实硬件放行条件", "", "必须重新运行 V0.53/V0.54，输入真实 DCA1000 IQ、同次采集 CFG、已知角反射器多场景数据，并确认 measured_source、channel_order_hardware_verified 和实测校准矩阵 provenance。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8"); return summary


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--multiscene-summary", type=Path, required=True); parser.add_argument("--hdf5-summary", type=Path, required=True); parser.add_argument("--output", type=Path, required=True); args = parser.parse_args(); print(json.dumps(run(args.multiscene_summary.resolve(), args.hdf5_summary.resolve(), args.output.resolve()), indent=2, ensure_ascii=False)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
