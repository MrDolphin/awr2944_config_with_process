"""Infer RF-network geometric endpoint candidates without calling them phase centers."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path


FIELD_RE = re.compile(r"([^|=]+)=([^|]*)")
TARGETS = {"TX1", "TX2", "TX3", "TX4", "RX1", "RX2", "RX3", "RX4"}


def _records(text: str, kind: str):
    for chunk in text.split(f"|RECORD={kind}")[1:]:
        chunk = chunk.split("|RECORD=", 1)[0]
        yield dict(FIELD_RE.findall(chunk[:6000]))


def _mil(value: str) -> float:
    return float(value.removesuffix("mil"))


def _key(x: float, y: float, tolerance: float = 1.0) -> tuple[int, int]:
    return round(x / tolerance), round(y / tolerance)


def trace(pcbdoc: Path, output: Path) -> dict:
    text = pcbdoc.read_text(errors="ignore")
    board = next(iter(_records(text, "Board")), {})
    nets = {row.get("ID"): row.get("NAME", "") for row in _records(text, "Net") if row.get("NAME") in TARGETS}
    pads = defaultdict(list)
    for row in _records(text, "Pad"):
        name = nets.get(row.get("NET"))
        if name and row.get("COMPONENT") == "727":
            pads[name].append((_mil(row["X"]), _mil(row["Y"])))

    edges = defaultdict(list)
    for row in _records(text, "Track"):
        name = nets.get(row.get("NET"))
        if not name:
            continue
        p1 = (_mil(row["X1"]), _mil(row["Y1"]))
        p2 = (_mil(row["X2"]), _mil(row["Y2"]))
        edges[name].append((p1, p2, "Track"))
    for row in _records(text, "Arc"):
        name = nets.get(row.get("NET"))
        if not name:
            continue
        cx, cy = _mil(row["LOCATION.X"]), _mil(row["LOCATION.Y"])
        radius = _mil(row["RADIUS"])
        start = math.radians(float(row["STARTANGLE"]))
        end = math.radians(float(row["ENDANGLE"]))
        p1 = (cx + radius * math.cos(start), cy + radius * math.sin(start))
        p2 = (cx + radius * math.cos(end), cy + radius * math.sin(end))
        edges[name].append((p1, p2, "Arc"))

    rows = []
    candidates = {}
    for name in sorted(TARGETS):
        degree = Counter()
        positions = {}
        for p1, p2, kind in edges[name]:
            for p in (p1, p2):
                k = _key(*p)
                degree[k] += 1
                positions[k] = p
        chip = pads[name][0] if pads[name] else (float("nan"), float("nan"))
        endpoints = []
        for k, count in degree.items():
            if count != 1:
                continue
            x, y = positions[k]
            distance = math.hypot(x - chip[0], y - chip[1]) if all(math.isfinite(v) for v in chip) else float("nan")
            endpoints.append((distance, x, y, count))
        endpoints.sort(reverse=True)
        candidates[name] = [{"x_mil": x, "y_mil": y, "distance_from_chip_pad_mil": d, "topology_degree": count} for d, x, y, count in endpoints]
        for rank, candidate in enumerate(candidates[name], start=1):
            rows.append({"net": name, "rank": rank, **candidate, "x_mm": candidate["x_mil"] * 0.0254, "y_mm": candidate["y_mil"] * 0.0254, "phase_center": False})

    output.mkdir(parents=True, exist_ok=True)
    with (output / "rf_endpoint_candidates.csv").open("w", newline="", encoding="utf-8") as handle:
        fields = ["net", "rank", "x_mil", "y_mil", "x_mm", "y_mm", "distance_from_chip_pad_mil", "topology_degree", "phase_center"]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    result = {
        "status": "completed_rf_endpoint_candidate_inference",
        "source": str(pcbdoc),
        "candidate_count": len(rows),
        "candidates_by_net": candidates,
        "phase_centers_validated": False,
        "method": "degree-one endpoints of target-net Track/Arc graph, ranked by distance from U29 package pad",
        "warning": "Candidates are geometric open-end clues only; they are not antenna phase centers or AoA truth.",
    }
    (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    try:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 6), dpi=160)
        vertex_indices = [i for i in range(32) if board.get(f"VX{i}") and board.get(f"VY{i}")]
        if vertex_indices:
            bx = [_mil(board[f"VX{i}"]) for i in vertex_indices] + [_mil(board[f"VX{vertex_indices[0]}"])]
            by = [_mil(board[f"VY{i}"]) for i in vertex_indices] + [_mil(board[f"VY{vertex_indices[0]}"])]
            ax.plot(bx, by, "k-", linewidth=0.8, label="PCB outline")
        for name in sorted(TARGETS):
            points = candidates[name]
            if points:
                p = points[0]
                ax.scatter(p["x_mil"], p["y_mil"], s=28, label=f"{name} endpoint")
                ax.annotate(name, (p["x_mil"], p["y_mil"]), xytext=(3, 3), textcoords="offset points", fontsize=7)
        ax.set_title("AWR2944P RF network geometric endpoint candidates")
        ax.set_xlabel("PCB X (mil)")
        ax.set_ylabel("PCB Y (mil)")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=6, loc="best")
        fig.tight_layout()
        fig.savefig(output / "rf_endpoint_candidates.png")
        plt.close(fig)
    except Exception as exc:  # plotting is supplementary; preserve numeric audit
        result["plot_warning"] = str(exc)
        (output / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    (output / "output_analysis.md").write_text(
        "# V0.4.105 RF 端点候选\n\n"
        f"共生成 {len(rows)} 个端点候选。候选来自 TX/RX Track/Arc 图元的拓扑度为 1 的端点，并按距 U29 封装焊盘距离排序。\n\n"
        "这些点只能作为开放端微带或馈电区域的几何线索，不能直接作为天线相位中心、虚拟阵元坐标或 AoA 真值。必须与天线层图、EM 模型或已知角度实测对齐后才能进入最终阵列模型。\n",
        encoding="utf-8",
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcbdoc", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = trace(args.pcbdoc, args.output)
    print(json.dumps({k: result[k] for k in ("status", "candidate_count", "phase_centers_validated")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
