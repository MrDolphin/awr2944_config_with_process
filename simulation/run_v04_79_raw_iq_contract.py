"""Validate DCA1000/LVDS raw-IQ files against the V0.4.79 contract."""

from __future__ import annotations

import argparse, json
from pathlib import Path
import h5py
import numpy as np

SCHEMA = "awr2944p-raw-iq-contract-v0.4.79"


def _load(path: Path):
    metadata = {}
    if path.suffix.lower() == ".npz":
        archive = np.load(path, allow_pickle=False)
        key = "iq" if "iq" in archive else ("raw_iq" if "raw_iq" in archive else None)
        if key is None:
            raise ValueError("NPZ must contain iq or raw_iq")
        return np.asarray(archive[key]), metadata, key
    with h5py.File(path, "r") as handle:
        key = "/radar/iq" if "/radar/iq" in handle else ("/recovered/virtual_iq" if "/recovered/virtual_iq" in handle else None)
        if key is None:
            raise ValueError("HDF5 must contain /radar/iq or /recovered/virtual_iq")
        data = handle[key][...]
        metadata = {str(k): (v.decode() if isinstance(v, bytes) else v.item() if hasattr(v, "item") else v) for k, v in handle.attrs.items()}
        return np.asarray(data), metadata, key


def validate(path: Path) -> dict:
    data, metadata, dataset = _load(path)
    issues = []
    if data.ndim not in (3, 4):
        issues.append(f"expected 3-D raw or 4-D virtual IQ, got ndim={data.ndim}")
    if not np.iscomplexobj(data):
        issues.append("IQ dataset is not complex-valued")
    if data.ndim >= 3 and data.shape[-1 if data.ndim == 4 else -1] != 4:
        issues.append("TX/RX terminal dimension is not 4")
    if data.ndim == 3 and data.shape[2] != 4:
        issues.append("raw IQ RX dimension is not 4")
    if data.ndim == 4 and data.shape[2:] != (4, 4):
        issues.append("virtual IQ trailing dimensions must be (RX=4, TX=4)")
    if not np.isfinite(data.real).all() or not np.isfinite(data.imag).all():
        issues.append("IQ contains non-finite values")
    required_metadata = ["raw_dtype", "wire_order_assumption", "channel_order_verified", "cfg_snapshot", "tdm_tx_sequence", "ti_calibration_status", "capture_timestamp", "imu_or_platform_pose_reference"]
    missing_metadata = [key for key in required_metadata if key not in metadata]
    status = "valid_shape_metadata_incomplete" if not issues and missing_metadata else ("valid_contract" if not issues else "invalid_contract")
    return {"status": status, "schema_version": SCHEMA, "input": str(path.resolve()), "dataset": dataset,
            "dtype": str(data.dtype), "shape": list(data.shape), "kind": "raw_adc_iq" if data.ndim == 3 else "virtual_iq",
            "issues": issues, "missing_metadata": missing_metadata, "metadata": metadata,
            "channel_order_verified": bool(metadata.get("channel_order_verified", False))}


def run(input_path: Path, output: Path) -> dict:
    result = validate(input_path)
    output.mkdir(parents=True, exist_ok=True)
    (output / "validation.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.79 DCA1000/LVDS 原始 IQ 接入契约\n\n"
        f"输入：`{input_path.resolve()}`\n\n状态：`{result['status']}`\n\n"
        f"数据类型：`{result['kind']}`；形状：`{result['shape']}`；数据集：`{result['dataset']}`。\n\n"
        "## 必须记录的采集元数据\n\n"
        "- ADC 原始类型和端序；\n- LVDS wire order；\n- RX 数量、样本数、chirp 数；\n- TDM TX 序列和 frame 边界；\n- CFG 文件快照；\n- 通道顺序是否已验证；\n- TI 校准状态、校准命令和幅相矩阵；\n- 时间戳、船体姿态和安装角度。\n\n"
        f"当前缺失元数据：`{result['missing_metadata']}`。\n\n"
        "契约验证通过不代表 AoA 已经经过硬件验证；`channel_order_verified=false` 时，数据只能进入排列敏感性分析，不能作为实板角度精度结论。\n", encoding="utf-8")
    return result


def main():
    p=argparse.ArgumentParser(); p.add_argument("--input",type=Path,required=True); p.add_argument("--output",type=Path,required=True); a=p.parse_args(); print(json.dumps(run(a.input.resolve(),a.output.resolve()),indent=2,ensure_ascii=False))


if __name__ == "__main__": main()
