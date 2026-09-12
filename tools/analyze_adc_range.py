#!/usr/bin/env python3
"""Range-domain analysis for AWR2944P real-only DCA1000 ADC captures.

Validated input order for this project is AWR2944 4-RX, 2-LVDS-lane,
real-only, non-interleaved data: [frame, chirp, RX, ADC sample].  The tool
does not attempt TDM-MIMO AoA; it generates traceable range-domain evidence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

try:
    from tools.dca1000_capture import parse_radar_cfg
except ModuleNotFoundError:  # Direct ``python tools/analyze_adc_range.py``.
    from dca1000_capture import parse_radar_cfg


SPEED_OF_LIGHT_MPS = 299_792_458.0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bin", required=True, dest="bin_path")
    parser.add_argument("--cfg", required=True)
    parser.add_argument("--output-dir", help="Defaults to capture directory/range_analysis.")
    parser.add_argument("--max-range-m", type=float, default=15.0)
    parser.add_argument("--remove-mean", action="store_true", help="Remove each chirp/RX time-domain mean before FFT.")
    return parser.parse_args(argv)


def load_cube(bin_path: Path, cfg_path: Path) -> tuple[np.ndarray, int, dict]:
    cfg = parse_radar_cfg(str(cfg_path))
    samples = int(cfg.get("num_adc_samples", 0) or 0)
    rx = int(cfg.get("num_rx", 0) or 0)
    chirps = int(cfg.get("num_chirps_per_frame", 0) or 0)
    if not samples or not rx or not chirps:
        raise ValueError("CFG must provide num_adc_samples, num_rx and num_chirps_per_frame")
    words_per_frame = samples * rx * chirps
    raw = np.fromfile(bin_path, dtype="<i2")
    full_frames, trailing_words = divmod(raw.size, words_per_frame)
    if full_frames < 1:
        raise ValueError("capture does not contain one complete frame")
    cube = raw[: full_frames * words_per_frame].reshape(full_frames, chirps, rx, samples)
    return cube, trailing_words * 2, cfg


def range_axis_m(samples: int, sample_rate_ksps: float, slope_mhz_per_us: float) -> np.ndarray:
    sample_rate_hz = sample_rate_ksps * 1e3
    slope_hz_per_s = slope_mhz_per_us * 1e12
    beat_hz = np.fft.rfftfreq(samples, d=1.0 / sample_rate_hz)
    return SPEED_OF_LIGHT_MPS * beat_hz / (2.0 * slope_hz_per_s)


def range_power(cube: np.ndarray, remove_mean: bool = False) -> np.ndarray:
    signal = cube.astype(np.float32)
    if remove_mean:
        signal = signal - signal.mean(axis=-1, keepdims=True)
    window = np.hanning(signal.shape[-1]).astype(np.float32)
    spectrum = np.fft.rfft(signal * window, axis=-1)
    return np.abs(spectrum) ** 2


def db(power: np.ndarray, floor_db: float = -120.0) -> np.ndarray:
    peak = float(np.max(power)) if power.size else 0.0
    if peak <= 0:
        return np.full(power.shape, floor_db)
    return np.maximum(10.0 * np.log10(np.maximum(power, np.finfo(float).tiny) / peak), floor_db)


def markdown_report(metadata: dict, cfg: dict) -> str:
    peak = metadata["strongest_mean_range_peak"]
    cube_shape = metadata["cube_shape"]
    return "\n".join(
        [
            "# AWR2944P 原始 ADC 距离域分析",
            "",
            "## 处理依据与数据排列",
            "",
            "本分析按 AWR2944P 的实数、非交织 LVDS 输出解释数据：`[frame, chirp, RX, ADC sample]`，每个样本为一个 16-bit 有符号整数。",
            "",
            "## 本次处理结果",
            "",
            f"- 完整帧：`{metadata['full_frames']}`；数据立方体：`{cube_shape}`。",
            f"- 忽略末尾非完整数据：`{metadata['trailing_bytes_ignored']}` bytes。",
            f"- 配置：ADC samples=`{cfg['num_adc_samples']}`，RX=`{cfg['num_rx']}`，chirps/frame=`{cfg['num_chirps_per_frame']}`，frame period=`{cfg['frame_period_ms']}` ms。",
            f"- 理论距离 bin 间隔：`{metadata['range_bin_spacing_m']:.6f}` m。",
            f"- 绘图最大距离：`{metadata['max_range_plotted_m']:.3f}` m。",
            f"- 平均距离谱的最强峰：bin `{peak['range_bin']}`，约 `{peak['range_m']:.3f}` m。",
            f"- 每 chirp/RX 去均值：`{metadata['remove_time_domain_mean']}`。",
            "",
            "### 0.3 m 以外的候选稳定反射峰",
            "",
            "| 距离 (m) | 相对功率 (dB) | 帧间波动标准差 (dB) |",
            "|---:|---:|---:|",
            *[
                f"| {candidate['range_m']:.3f} | {candidate['relative_power_db']:.2f} | {candidate['temporal_std_db']:.2f} |"
                for candidate in metadata["candidate_static_peaks"]
            ],
            "",
            "## 图像如何阅读",
            "",
            "- `mean_range_profile.png`：所有帧、chirp 和 RX 平均后的距离谱。红点仅标记 0.3 m 以外、按平均功率排序的局部候选稳定峰；标签为距离和相对功率，不是检测结果。靠近 0 m 的强峰仍可能是直流/近距离泄漏。",
            "- `per_rx_range_profiles.png`：各 RX 独立距离谱；同一稳定反射峰在各 RX 位置一致，是后续阵列相位/AoA 处理的前提之一。幅度不同不等同于相位已校准。",
            "- `range_time_intensity.png`：对 chirp 与 RX 平均后，各帧的距离强度。白色虚线复用平均谱中排名靠前的候选峰距离，方便与上图逐一对应；沿时间方向延伸的亮带表示稳定距离反射，随时间倾斜或移动的亮带才可能对应距离变化目标。",
            "- `diagnostic_dashboard.png`：单 chirp 时域、带候选峰标签的平均 1D 距离谱、单 TX 组诊断性 2D Range-Doppler、带相同距离参考线的 Range-Time 总览。它用于快速判断原始数据是否具有合理结构，不能当作已校准速度、AoA 或点云。",
            "- `range_doppler_diagnostic_frame0_txgroup0_rx0.png`：只选 frame 0、RX 0 和一个 TX chirp 组做慢时间 FFT，避免把相邻 TDM-MIMO chirp 混成伪 Doppler；速度轴仍只是依据 CFG 时序推算的诊断坐标。",
            "",
            "## 结论边界",
            "",
            "本阶段已能验证原始采样在距离域中具有可分析结构；尚未完成 RX/TX 虚拟阵列重排、通道相位校准、TDM 慢时间重排或 AoA。因此，不能把该报告的峰值直接解释为方位角、俯仰角或真实点云目标。应以已知距离的角反射器/静止目标进行下一步绝对距离验证。",
            "",
        ]
    )


def candidate_static_peaks(
    ranges: np.ndarray, mean_range_power: np.ndarray, range_time_power: np.ndarray, minimum_range_m: float = 0.3, count: int = 8
) -> list[dict]:
    """Return strongest local peaks outside the direct-leakage guard range."""
    valid = np.flatnonzero((ranges >= minimum_range_m) & (np.arange(ranges.size) > 0) & (np.arange(ranges.size) < ranges.size - 1))
    local = valid[(mean_range_power[valid] >= mean_range_power[valid - 1]) & (mean_range_power[valid] >= mean_range_power[valid + 1])]
    ordered = local[np.argsort(mean_range_power[local])[::-1]][:count]
    reference = float(np.max(mean_range_power))
    records = []
    for index in ordered:
        temporal_db = 10.0 * np.log10(np.maximum(range_time_power[:, index], np.finfo(float).tiny) / reference)
        records.append(
            {
                "range_bin": int(index),
                "range_m": float(ranges[index]),
                "relative_power_db": float(10.0 * np.log10(mean_range_power[index] / reference)),
                "temporal_std_db": float(np.std(temporal_db)),
            }
        )
    return records


def annotate_candidate_peaks(axis, candidates: list[dict], *, maximum: int = 6) -> None:
    """Mark the strongest stable range candidates without calling them detections."""
    for ordinal, candidate in enumerate(candidates[:maximum]):
        range_m = candidate["range_m"]
        relative_power_db = candidate["relative_power_db"]
        axis.plot(range_m, relative_power_db, marker="o", color="crimson", markersize=4, zorder=4)
        axis.annotate(
            f"P{ordinal + 1}",
            xy=(range_m, relative_power_db),
            xytext=(2, 5),
            textcoords="offset points",
            ha="left",
            va="bottom",
            fontsize=6.5,
            color="crimson",
        )


def candidate_peak_table(candidates: list[dict], *, maximum: int = 6) -> str:
    """Return a compact legend that remains readable when close peaks cluster."""
    rows = ["Candidate stable peaks (>=0.3 m)", "ID   range     power     frame std"]
    for ordinal, candidate in enumerate(candidates[:maximum]):
        rows.append(
            f"P{ordinal + 1:<2}  {candidate['range_m']:>6.1f} m  {candidate['relative_power_db']:>6.1f} dB  "
            f"{candidate['temporal_std_db']:>5.2f} dB"
        )
    return "\n".join(rows)


def draw_candidate_peak_table(axis, candidates: list[dict], *, maximum: int = 6) -> None:
    if not candidates:
        return
    axis.text(
        0.98,
        0.97,
        candidate_peak_table(candidates, maximum=maximum),
        transform=axis.transAxes,
        ha="right",
        va="top",
        fontsize=6.5,
        family="monospace",
        color="black",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "crimson", "alpha": 0.88},
        zorder=5,
    )


def annotate_range_time_candidates(axis, candidates: list[dict], *, maximum: int = 6) -> None:
    """Use the same peak positions as the mean spectrum for cross-panel reading."""
    for candidate in candidates[:maximum]:
        range_m = candidate["range_m"]
        axis.axvline(range_m, color="white", linewidth=0.7, linestyle="--", alpha=0.7)
    draw_candidate_peak_table(axis, candidates, maximum=maximum)


def diagnostic_products(cube: np.ndarray, ranges: np.ndarray, cfg: dict) -> dict:
    """Create WaveStudio-like diagnostic views without claiming calibrated Doppler.

    The 2D transform deliberately selects chirp indices 0, N, 2N ... from one
    TX group.  Mixing adjacent TDM-MIMO chirps would produce a visually neat
    but physically misleading Doppler image.
    """
    frame_index = 0
    rx_index = 0
    chirp_index = 0
    time_domain = cube[frame_index, chirp_index, rx_index].astype(np.float32)
    range_spectrum = np.fft.rfft((time_domain - time_domain.mean()) * np.hanning(time_domain.size))
    chirps_per_loop = int(cfg.get("num_chirps_per_loop", 0) or 0)
    if chirps_per_loop < 1 or chirps_per_loop > cube.shape[1]:
        chirps_per_loop = cube.shape[1]
    chirp_indices = np.arange(0, cube.shape[1], chirps_per_loop, dtype=int)
    slow_time = cube[frame_index, chirp_indices, rx_index].astype(np.float32)
    slow_time -= slow_time.mean(axis=-1, keepdims=True)
    range_fft = np.fft.rfft(slow_time * np.hanning(slow_time.shape[-1]), axis=-1)
    range_doppler = np.fft.fftshift(np.fft.fft(range_fft * np.hanning(range_fft.shape[0])[:, None], axis=0), axes=0)

    chirp_period_s = (
        float(cfg.get("idle_time_us", 0.0) or 0.0) + float(cfg.get("ramp_end_time_us", 0.0) or 0.0)
    ) * 1e-6
    slow_time_period_s = chirp_period_s * chirps_per_loop
    if slow_time_period_s > 0 and cfg.get("start_freq_ghz"):
        wavelength_m = SPEED_OF_LIGHT_MPS / (float(cfg["start_freq_ghz"]) * 1e9)
        velocity_mps = np.fft.fftshift(np.fft.fftfreq(len(chirp_indices), d=slow_time_period_s)) * wavelength_m / 2.0
    else:
        velocity_mps = np.arange(len(chirp_indices), dtype=float)

    range_bins = len(ranges)
    return {
        "time_domain": time_domain,
        "single_chirp_range_power": (np.abs(range_spectrum) ** 2)[:range_bins],
        "range_doppler_power": (np.abs(range_doppler) ** 2)[:, :range_bins],
        "velocity_mps": velocity_mps,
        "metadata": {
            "frame_index": frame_index,
            "rx_index": rx_index,
            "chirp_indices_within_frame": [int(index) for index in chirp_indices],
            "slow_time_chirps": int(len(chirp_indices)),
            "chirps_per_loop": chirps_per_loop,
            "nominal_slow_time_period_s": slow_time_period_s if slow_time_period_s > 0 else None,
            "velocity_axis_is_diagnostic_only": True,
            "velocity_axis_reason": "Uses profileCfg timing and one TX chirp group; final TDM ordering and Doppler calibration are not validated.",
        },
    }


def write_diagnostic_plots(
    products: dict,
    ranges: np.ndarray,
    range_time_power: np.ndarray,
    time_s: np.ndarray,
    output_dir: Path,
    *,
    mean_range_power: np.ndarray | None = None,
    candidate_peaks: list[dict] | None = None,
) -> dict:
    """Write a compact four-panel overview with consistent stable-peak labels."""
    output_dir.mkdir(parents=True, exist_ok=True)
    time_domain = products["time_domain"]
    single_chirp_power = products["single_chirp_range_power"]
    range_doppler_power = products["range_doppler_power"]
    velocity_mps = products["velocity_mps"]
    rx_index = products["metadata"]["rx_index"]
    chirp_index = products["metadata"]["chirp_indices_within_frame"][0]
    candidates = candidate_peaks or []
    if mean_range_power is None:
        mean_range_power = single_chirp_power

    paths = {
        "time_domain": output_dir / "time_domain_frame0_chirp0_rx0.png",
        "single_chirp_range": output_dir / "single_chirp_range_profile_frame0_chirp0_rx0.png",
        "mean_range_annotated": output_dir / "mean_range_profile.png",
        "range_doppler": output_dir / "range_doppler_diagnostic_frame0_txgroup0_rx0.png",
        "dashboard": output_dir / "diagnostic_dashboard.png",
    }

    def draw_time_domain(axis):
        axis.plot(np.arange(time_domain.size), time_domain, linewidth=0.8)
        axis.set_title(f"Time domain: frame 0, chirp {chirp_index}, RX {rx_index}")
        axis.set_xlabel("ADC sample")
        axis.set_ylabel("ADC code")
        axis.grid(True, alpha=0.3)

    def draw_single_chirp_range(axis):
        axis.plot(ranges, db(single_chirp_power), linewidth=0.9)
        axis.set_title(f"1D range FFT: frame 0, chirp {chirp_index}, RX {rx_index}")
        axis.set_xlabel("Range (m)")
        axis.set_ylabel("Relative power (dB)")
        axis.grid(True, alpha=0.3)

    def draw_mean_range(axis):
        axis.plot(ranges, db(mean_range_power), linewidth=0.9, label="Mean across all frames/chirps/RX")
        annotate_candidate_peaks(axis, candidates)
        draw_candidate_peak_table(axis, candidates)
        axis.set_title("Mean 1D range spectrum: candidate stable peaks (not detections)")
        axis.set_xlabel("Range (m)")
        axis.set_ylabel("Relative power (dB)")
        axis.grid(True, alpha=0.3)
        if candidates:
            axis.legend(loc="lower right", fontsize=7)

    def draw_range_doppler(axis):
        image = db(range_doppler_power).T
        extent = [float(velocity_mps[0]), float(velocity_mps[-1]), float(ranges[-1]), float(ranges[0])]
        result = axis.imshow(image, aspect="auto", extent=extent, cmap="viridis", vmin=-50, vmax=0)
        axis.set_title("Diagnostic 2D Range-Doppler: frame 0, TX group 0, RX 0")
        axis.set_xlabel("Nominal velocity (m/s; diagnostic only)")
        axis.set_ylabel("Range (m)")
        return result

    def draw_range_time(axis):
        image = db(range_time_power)
        extent = [float(ranges[0]), float(ranges[-1]), float(time_s[-1]) if len(time_s) else 0.0, 0.0]
        result = axis.imshow(image, aspect="auto", extent=extent, cmap="viridis", vmin=-50, vmax=0)
        axis.set_title("Range-Time: mean across chirps and RX")
        axis.set_xlabel("Range (m)")
        axis.set_ylabel("Frame time (s)")
        annotate_range_time_candidates(axis, candidates)
        return result

    for name, draw in (("time_domain", draw_time_domain), ("single_chirp_range", draw_single_chirp_range)):
        figure, axis = plt.subplots(figsize=(10, 5))
        draw(axis)
        figure.tight_layout()
        figure.savefig(paths[name], dpi=160)
        plt.close(figure)

    figure, axis = plt.subplots(figsize=(12, 6))
    draw_mean_range(axis)
    figure.tight_layout()
    figure.savefig(paths["mean_range_annotated"], dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(10, 6))
    image = draw_range_doppler(axis)
    figure.colorbar(image, ax=axis, label="Relative power (dB)")
    figure.tight_layout()
    figure.savefig(paths["range_doppler"], dpi=160)
    plt.close(figure)

    figure, axes = plt.subplots(2, 2, figsize=(16, 10))
    draw_time_domain(axes[0, 0])
    draw_mean_range(axes[0, 1])
    range_doppler_image = draw_range_doppler(axes[1, 0])
    range_time_image = draw_range_time(axes[1, 1])
    figure.colorbar(range_doppler_image, ax=axes[1, 0], label="Relative power (dB)")
    figure.colorbar(range_time_image, ax=axes[1, 1], label="Relative power (dB)")
    figure.suptitle("AWR2944P Raw ADC diagnostic overview (not AoA / calibrated velocity)", fontsize=14)
    figure.tight_layout()
    figure.savefig(paths["dashboard"], dpi=160)
    plt.close(figure)

    return {name: str(path) for name, path in paths.items()}


def write_outputs(cube: np.ndarray, trailing_bytes: int, cfg: dict, output_dir: Path, max_range_m: float, remove_mean: bool) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    power = range_power(cube, remove_mean=remove_mean)
    ranges = range_axis_m(cube.shape[-1], float(cfg["sample_rate_ksps"]), float(cfg["freq_slope_mhz_per_us"]))
    select = ranges <= max_range_m
    ranges = ranges[select]
    power = power[..., select]

    frame_period_s = float(cfg.get("frame_period_ms", 0.0)) / 1e3
    time_s = np.arange(cube.shape[0]) * frame_period_s
    mean_range_power = power.mean(axis=(0, 1, 2))
    rx_range_power = power.mean(axis=(0, 1))
    range_time_power = power.mean(axis=(1, 2))
    strongest_bin = int(np.argmax(mean_range_power))
    strongest = {
        "range_bin": strongest_bin,
        "range_m": float(ranges[strongest_bin]),
        "relative_power_db": 0.0,
    }
    candidates = candidate_static_peaks(ranges, mean_range_power, range_time_power)
    diagnostics = diagnostic_products(cube, ranges, cfg)
    diagnostic_paths = write_diagnostic_plots(
        diagnostics,
        ranges,
        range_time_power,
        time_s,
        output_dir,
        mean_range_power=mean_range_power,
        candidate_peaks=candidates,
    )
    metadata = {
        "input_format": "AWR2944P real-only, non-interleaved [frame, chirp, rx, sample], int16",
        "full_frames": int(cube.shape[0]),
        "trailing_bytes_ignored": int(trailing_bytes),
        "cube_shape": [int(value) for value in cube.shape],
        "range_bin_spacing_m": float(ranges[1] - ranges[0]) if len(ranges) > 1 else None,
        "max_range_plotted_m": float(ranges[-1]),
        "remove_time_domain_mean": bool(remove_mean),
        "strongest_mean_range_peak": strongest,
        "candidate_static_peaks": candidates,
        "diagnostic_visualizations": {
            **diagnostics["metadata"],
            "paths": diagnostic_paths,
        },
        "limitations": [
            "Range magnitude is valid for the documented AWR2944 real-only non-interleaved order.",
            "RX/TX virtual-array mapping, calibration and AoA are not performed here.",
            "The strongest bin may be direct leakage or a nearby static reflector; use a controlled target to validate absolute range.",
            "The 2D Range-Doppler visualization is diagnostic only, not a validated velocity product or TDM-MIMO AoA input.",
        ],
    }

    np.savez_compressed(
        output_dir / "range_fft_products.npz",
        range_m=ranges,
        time_s=time_s,
        mean_range_power=mean_range_power,
        rx_range_power=rx_range_power,
        range_time_power=range_time_power,
    )
    (output_dir / "range_fft_analysis.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    (output_dir / "output_analysis.md").write_text(markdown_report(metadata, cfg), encoding="utf-8")

    plt.figure(figsize=(10, 6))
    for rx_index, values in enumerate(rx_range_power):
        plt.plot(ranges, db(values), label=f"RX{rx_index}")
    plt.xlabel("Range (m)")
    plt.ylabel("Relative power per RX (dB)")
    plt.title("Per-RX mean range spectra")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_dir / "per_rx_range_profiles.png", dpi=160)
    plt.close()

    plt.figure(figsize=(10, 6))
    image = db(range_time_power)
    extent = [float(ranges[0]), float(ranges[-1]), float(time_s[-1]) if len(time_s) else 0.0, 0.0]
    plt.imshow(image, aspect="auto", extent=extent, cmap="viridis", vmin=-50, vmax=0)
    plt.colorbar(label="Relative power (dB)")
    plt.xlabel("Range (m)")
    plt.ylabel("Frame time (s)")
    plt.title("Range-Time intensity (mean across chirps and RX)")
    plt.tight_layout()
    plt.savefig(output_dir / "range_time_intensity.png", dpi=160)
    plt.close()

    return metadata


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    bin_path = Path(args.bin_path)
    cfg_path = Path(args.cfg)
    cube, trailing_bytes, cfg = load_cube(bin_path, cfg_path)
    output_dir = Path(args.output_dir) if args.output_dir else bin_path.parent / "range_analysis"
    report = write_outputs(cube, trailing_bytes, cfg, output_dir, args.max_range_m, args.remove_mean)
    print(f"[DONE] frames={report['full_frames']} trailing_bytes={report['trailing_bytes_ignored']}")
    print(f"[DONE] strongest_mean_range_peak={report['strongest_mean_range_peak']['range_m']:.3f} m")
    print(f"[DONE] diagnostic_dashboard={report['diagnostic_visualizations']['paths']['dashboard']}")
    print("[DONE] diagnostic_range_doppler=generated (not a calibrated velocity or AoA result)")
    print(f"[DONE] output={output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
