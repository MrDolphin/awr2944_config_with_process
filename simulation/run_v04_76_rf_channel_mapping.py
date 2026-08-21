"""Build an auditable PCB RF-net to CFG virtual-channel mapping candidate."""

from __future__ import annotations

import argparse, csv, json, re
from pathlib import Path

PCB_NET = {"TX1": 110, "TX2": 109, "TX3": 108, "TX4": 107,
           "RX1": 120, "RX2": 119, "RX3": 118, "RX4": 117}


def parse_cfg(path: Path):
    text = path.read_text(encoding="utf-8")
    chirps = {}
    for match in re.finditer(r"^chirpCfg\s+(\d+)\s+(\d+)\s+0\s+0\s+0\s+0\s+0\s+(\d+)", text, re.MULTILINE):
        chirps[int(match.group(1))] = int(match.group(3))
    geometry = re.search(r"^antGeometryCfg\s+(.+)$", text, re.MULTILINE)
    if not geometry:
        raise ValueError("antGeometryCfg missing")
    tokens = geometry.group(1).split()
    if len(tokens) != 34:
        raise ValueError(f"expected 34 antGeometryCfg values, found {len(tokens)}")
    return chirps, tokens


def run(pcb_pads: Path, cfg: Path, output: Path) -> dict:
    with pcb_pads.open(encoding="utf-8", newline="") as handle:
        pads = list(csv.DictReader(handle))
    pad_by_name = {row["antenna"]: row for row in pads}
    chirps, geometry_tokens = parse_cfg(cfg)
    rows = []
    for tx_index in range(4):
        tx_name = f"TX{tx_index+1}"
        for rx_index in range(4):
            rx_name = f"RX{rx_index+1}"
            index = tx_index * 4 + rx_index
            row = {
                "virtual_input_index": index, "tx_index": tx_index, "rx_index": rx_index,
                "tx_name": tx_name, "rx_name": rx_name, "chirp_index": tx_index,
                "tx_enable_bit": chirps.get(tx_index, ""), "tx_net_id": PCB_NET[tx_name],
                "rx_net_id": PCB_NET[rx_name], "tx_pad_name": pad_by_name.get(tx_name, {}).get("pad_name", ""),
                "rx_pad_name": pad_by_name.get(rx_name, {}).get("pad_name", ""),
                "cfg_row": geometry_tokens[2*index], "cfg_column": geometry_tokens[2*index+1],
                "mapping_status": "name_based_candidate_not_channel_calibrated",
                "source_status": "PCB_net_name_plus_CFG_chirp_order",
            }
            rows.append(row)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "rf_channel_mapping_candidate.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    unique_inputs = len({row["virtual_input_index"] for row in rows})
    summary = {"status": "completed_rf_channel_mapping_candidate", "mapping_count": len(rows), "unique_virtual_inputs": unique_inputs, "unique_tx_nets": len({row["tx_net_id"] for row in rows}), "unique_rx_nets": len({row["rx_net_id"] for row in rows}), "chirp_cfg": chirps, "mapping_validated": False, "channel_phase_calibrated": False}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.76 PCB RF 网络到 CFG 虚拟通道映射\n\n"
        "本阶段把 PCB ASCII 中的 TX1–TX4/RX1–RX4 网络名，与 CFG 的 chirpCfg 发射顺序和 antGeometryCfg 虚拟输入索引对齐。当前配置的 chirp 顺序是 TX1、TX2、TX3、TX4，每个 TX 与 RX1–RX4 组合形成 4 个虚拟输入。\n\n"
        f"共生成 {len(rows)} 个映射候选，虚拟输入唯一数为 {unique_inputs}。\n\n"
        "## 重要边界\n\n"
        "该表是基于网络名和 CFG 顺序的工程候选，不等价于 TI 固件内部通道重排，也没有证明 PCB Pad 是天线电气相位中心。必须用 DCA1000 原始数据、已知角度目标和 TI 校准幅相矩阵验证通道顺序。\n", encoding="utf-8")
    return summary


def main():
    p=argparse.ArgumentParser(); p.add_argument("--pcb-pads", type=Path, required=True); p.add_argument("--cfg", type=Path, required=True); p.add_argument("--output", type=Path, required=True); a=p.parse_args(); print(json.dumps(run(a.pcb_pads.resolve(), a.cfg.resolve(), a.output.resolve()), indent=2, ensure_ascii=False))


if __name__ == "__main__": main()
