"""Inventory AWR2944P PCB/CAD release files and classify extractable AoA data."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def classify(path: Path) -> tuple[str, str]:
    suffix = path.suffix.lower()
    head = path.read_bytes()[:32]
    if suffix == ".pcbdoc" and head.startswith(b"|RECORD="):
        return "altium_ascii_pcb", "可解析铜区/网络/层；可做几何中心近似"
    if suffix == ".pcbdoc" and head.startswith(bytes.fromhex("d0cf11e0a1b11ae1")):
        return "altium_ole_pcb", "需 Altium 导出 ASCII 或专用 OLE 解析器；当前不直接解析"
    if suffix == ".step":
        return "step_mechanical_cad", "可提取机械实体/包络；通常没有 RF 相位中心"
    if suffix == ".pdf":
        return "pdf_release", "用于装配方向、层叠、标注和人工复核"
    if suffix in {".xls", ".xlsx"}:
        return "bom", "用于器件/版本追溯，不提供天线相位中心"
    if suffix in {".schdoc", ".prjpcb", ".outjob", ".dat"}:
        return "altium_project", "用于设计版本和连接关系追溯"
    return "other", "未定义自动提取规则"


def inventory(roots: list[Path], output: Path) -> None:
    rows: list[tuple[str, int, str, str, str]] = []
    for root in roots:
        for path in sorted(p for p in root.rglob("*") if p.is_file()):
            kind, use = classify(path)
            rows.append((str(path.resolve()), path.stat().st_size, kind, use, sha256(path)))
    output.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# AWR2944P PCB/CAD 资料包审计",
        "",
        "本文件由 `simulation/inspect_pcb_package.py` 生成。哈希用于确认后续分析引用的是同一份资料。",
        "",
        "## 文件清单",
        "",
        "| 文件 | 字节数 | 类型 | 可提取内容 | SHA-256 |",
        "|---|---:|---|---|---|",
    ]
    for path, size, kind, use, digest in rows:
        lines.append(f"| `{path}` | {size} | {kind} | {use} | `{digest}` |")
    lines += [
        "",
        "## 当前判断",
        "",
        "- `PROC113D_ASCII.PcbDoc` 是当前最有价值的自动化输入：已能按 RF 网络提取 8 个 TX/RX 铜区的顶点、包围盒和几何中心。",
        "- `PROC113D_BRD.PcbDoc` 是 Altium OLE 二进制文件。它可能包含更完整的层和对象信息，但在未导出 ASCII 或未使用 Altium 官方解析链之前，不能声称已经提取成功。",
        "- `PROC113D_BRD.step` 适合验证板框、安装方向和机械包络，不足以单独给出天线电气相位中心。",
        "- 装配图、层叠图和原理图可用于确认 TX/RX 区域的朝向、层号、馈电关系；BOM 只做版本追溯。",
        "- 所有由 PCB 铜区几何中心得到的阵列坐标都必须标记为 `not_electrical_phase_center`，进入 AoA 前还需要仿真或角反射器实测校准。",
        "",
        "## 进入 AoA 管线所需的最小数据",
        "",
        "1. 每个 TX/RX 天线的唯一编号、网络名、层号和馈电点坐标；",
        "2. PCB 坐标原点、单位、X/Y 轴方向以及板面朝向；",
        "3. 从 TX/RX 编号到 CFG `Tx0Rx0...Tx3Rx3` 的映射；",
        "4. 真实天线相位中心或经过角反射器标定得到的等效坐标；",
        "5. 与坐标对应的校准幅相矩阵和采集 CFG。",
    ]
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", action="append", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    inventory([root.resolve() for root in args.root], args.output.resolve())
    print(f"Inventoried PCB/CAD package into {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
