"""Build a tolerant geometric connectivity graph for RF nets in ASCII PcbDoc."""

from __future__ import annotations

import argparse
import csv
import heapq
import math
import re
from collections import Counter
from pathlib import Path


NET_NAMES = {107: "TX4", 108: "TX3", 109: "TX2", 110: "TX1",
             117: "RX4", 118: "RX3", 119: "RX2", 120: "RX1"}
MIL_TO_MM = 0.0254


def value(line: str, key: str) -> str | None:
    match = re.search(rf"(?:^|\|){re.escape(key)}=([^|]+)", line)
    return match.group(1).strip() if match else None


def point(x: str, y: str) -> tuple[float, float]:
    return float(x.replace("mil", "")), float(y.replace("mil", ""))


def arc_endpoints(line: str) -> tuple[tuple[float, float], tuple[float, float]] | None:
    x, y, radius = value(line, "LOCATION.X"), value(line, "LOCATION.Y"), value(line, "RADIUS")
    start, end = value(line, "STARTANGLE"), value(line, "ENDANGLE")
    if not all((x, y, radius, start, end)):
        return None
    cx, cy = point(x, y)
    r = float(radius.replace("mil", ""))
    a0, a1 = math.radians(float(start)), math.radians(float(end))
    return ((cx + r * math.cos(a0), cy + r * math.sin(a0)),
            (cx + r * math.cos(a1), cy + r * math.sin(a1)))


def extract_primitives(path: Path):
    edges: dict[int, list[tuple[tuple[float, float], tuple[float, float], float, str]]] = {net: [] for net in NET_NAMES}
    pads: dict[int, list[tuple[float, float]]] = {net: [] for net in NET_NAMES}
    regions: dict[int, list[list[tuple[float, float]]]] = {net: [] for net in NET_NAMES}
    counts: Counter[str] = Counter()
    for line in path.read_text(errors="ignore").splitlines():
        record = value(line, "RECORD")
        net_text = value(line, "NET")
        if not record or not net_text or not net_text.isdigit() or int(net_text) not in NET_NAMES:
            continue
        net = int(net_text)
        counts[record] += 1
        if record == "Pad":
            x, y = value(line, "X"), value(line, "Y")
            if x and y:
                pads[net].append(point(x, y))
        elif record == "Track":
            x1, y1, x2, y2 = (value(line, key) for key in ("X1", "Y1", "X2", "Y2"))
            if all((x1, y1, x2, y2)):
                p0, p1 = point(x1, y1), point(x2, y2)
                edges[net].append((p0, p1, math.dist(p0, p1), "Track"))
        elif record == "Arc":
            endpoints = arc_endpoints(line)
            if endpoints:
                p0, p1 = endpoints
                edges[net].append((p0, p1, math.dist(p0, p1), "Arc"))
        elif record == "Region":
            vertices = [(float(x.replace("mil", "")), float(y.replace("mil", "")))
                        for _, x, y in re.findall(r"VX(\d+)=([^|]+)\|VY\1=([^|]+)", line)]
            regions[net].append(vertices)
            for p0, p1 in zip(vertices, vertices[1:] + vertices[:1]):
                edges[net].append((p0, p1, math.dist(p0, p1), "Region"))
    return edges, pads, regions, counts


def point_in_polygon(p: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    x, y = p
    inside = False
    for p0, p1 in zip(polygon, polygon[1:] + polygon[:1]):
        if (p0[1] > y) != (p1[1] > y):
            at_x = (p1[0] - p0[0]) * (y - p0[1]) / (p1[1] - p0[1]) + p0[0]
            if x < at_x:
                inside = not inside
    return inside


def graph_for(edges, pads, regions, tolerance_mil: float = 1.0):
    def key(p):
        return (round(p[0] / tolerance_mil), round(p[1] / tolerance_mil))

    coords = {}
    adjacency: dict[tuple[int, int], list[tuple[tuple[int, int], float]]] = {}
    edge_count = 0
    def add_node(p):
        k = key(p)
        coords.setdefault(k, p)
        adjacency.setdefault(k, [])
        return k
    def add_edge(p0, p1, length):
        k0, k1 = add_node(p0), add_node(p1)
        adjacency[k0].append((k1, length))
        adjacency[k1].append((k0, length))
        return 1
    for p0, p1, length, _kind in edges:
        edge_count += add_edge(p0, p1, length)
    pad_nodes = [add_node(p) for p in pads]
    region_target = ("region",)
    adjacency.setdefault(region_target, [])
    for p0, p1, _length, kind in edges:
        if kind == "Region":
            continue
        for candidate in (p0, p1):
            if any(point_in_polygon(candidate, polygon) for polygon in regions):
                node = add_node(candidate)
                adjacency[node].append((region_target, 0.0))
                adjacency[region_target].append((node, 0.0))
    return coords, adjacency, pad_nodes, edge_count, {region_target}


def shortest_to_region(adjacency, start, region_nodes):
    distances = {start: 0.0}
    queue = [(0.0, start)]
    while queue:
        distance, node = heapq.heappop(queue)
        if distance != distances[node]:
            continue
        if node in region_nodes:
            return distance, node
        for neighbor, weight in adjacency.get(node, []):
            candidate = distance + weight
            if candidate < distances.get(neighbor, float("inf")):
                distances[neighbor] = candidate
                heapq.heappush(queue, (candidate, neighbor))
    return None, None


def run(pcbdoc: Path, output: Path) -> None:
    edges, pads, regions, counts = extract_primitives(pcbdoc)
    rows = []
    for net, name in NET_NAMES.items():
        coords, adjacency, pad_nodes, edge_count, region_keys = graph_for(edges[net], pads[net], regions[net])
        path_mil, endpoint = shortest_to_region(adjacency, pad_nodes[0] if pad_nodes else None, region_keys) if pad_nodes else (None, None)
        rows.append({"antenna": name, "net_id": net, "pad_count": len(pads[net]),
                     "graph_nodes": len(coords), "graph_edges": edge_count,
                     "region_edge_count": sum(kind == "Region" for *_rest, kind in edges[net]),
                     "track_edge_count": sum(kind == "Track" for *_rest, kind in edges[net]),
                     "arc_edge_count": sum(kind == "Arc" for *_rest, kind in edges[net]),
                     "pad_to_region_connected": bool(endpoint),
                     "shortest_pad_to_region_mil": "" if path_mil is None else path_mil,
                     "shortest_pad_to_region_mm": "" if path_mil is None else path_mil * MIL_TO_MM,
                     "status": "geometric_connectivity_not_rf_phase_delay"})
    output.mkdir(parents=True, exist_ok=True)
    with (output / "rf_connectivity_summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    (output / "output_analysis.md").write_text(
        "# V0.4.15 RF 网络几何连通性\n\n"
        f"识别对象：{dict(counts)}。每条 RF 网络使用 1 mil 节点量化容差建立几何图。\n\n"
        "`pad_to_region_connected` 只表示 PCB ASCII 几何对象在容差内可连通，不表示 RF 电气连续性、阻抗匹配或相位中心。"
        "`shortest_pad_to_region_mm` 是 Track/Arc/Region 的几何路径长度，不能直接换算成 77 GHz 相位延迟。\n",
        encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcbdoc", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.pcbdoc.resolve(), args.output.resolve())
    print(f"Built RF connectivity graph in {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
