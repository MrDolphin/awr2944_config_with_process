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
            "- `mean_range_profile.png`：所有帧、chirp 和 RX 平均后的距离谱。靠近 0 m 的强峰可能是直流/近距离泄漏，不能直接当作目标。",
            "- `per_rx_range_profiles.png`：各 RX 独立距离谱；同一稳定反射峰在各 RX 位置一致，是后续阵列相位/AoA 处理的前提之一。幅度不同不等同于相位已校准。",
            "- `range_time_intensity.png`：对 chirp 与 RX 平均后，各帧的距离强度。横向持续亮线表示稳定距离反射；随时间移动的亮线才可能对应距离变化目标。",
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
        "limitations": [
            "Range magnitude is valid for the documented AWR2944 real-only non-interleaved order.",
            "RX/TX virtual-array mapping, calibration and AoA are not performed here.",
            "The strongest bin may be direct leakage or a nearby static reflector; use a controlled target to validate absolute range.",
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

    plt.figure(figsize=(10, 5))
    plt.plot(ranges, db(mean_range_power))
    plt.xlabel("Range (m)")
    plt.ylabel("Relative power (dB)")
    plt.title("AWR2944P mean range spectrum (all frames/chirps/RX)")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_dir / "mean_range_profile.png", dpi=160)
    plt.close()

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
    print(f"[DONE] output={output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
