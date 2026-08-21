"""Extract the mechanical frame and source traceability from SPRR440 release files."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _step_board_bbox(path: Path) -> dict:
    text = path.read_text(encoding="ascii", errors="ignore")
    records = {int(m.group(1)): m.group(0) for m in re.finditer(r"#(\d+)\s*=.*?;", text, flags=re.DOTALL)}
    # In this TI release the PCB product's shape representation is #2809 and its BREP is #2810.
    # Follow references from that BREP instead of using all component coordinates.
    pending = [2810]
    visited: set[int] = set()
    points: list[tuple[float, float, float]] = []
    while pending:
        identifier = pending.pop()
        if identifier in visited or identifier not in records:
            continue
        visited.add(identifier)
        record = records[identifier]
        if "CARTESIAN_POINT" in record:
            match = re.search(r"CARTESIAN_POINT\([^;]*?\(([-+0-9.Ee]+),([-+0-9.Ee]+),([-+0-9.Ee]+)\)", record, flags=re.DOTALL)
            if match:
                points.append(tuple(float(value) for value in match.groups()))
        for reference in re.findall(r"#(\d+)", record):
            pending.append(int(reference))
    if not points:
        raise ValueError("could not follow PCB BREP from STEP #2810")
    axes = list(zip(*points))
    return {
        "step_brep_root": 2810,
        "referenced_entity_count": len(visited),
        "cartesian_point_count": len(points),
        "bbox_mm": {axis: {"min": min(values), "max": max(values), "size": max(values) - min(values)} for axis, values in zip(("x", "y", "z"), axes)},
        "top_surface_z_mm": max(axes[2]),
        "bottom_surface_z_mm": min(axes[2]),
    }


def _pdf_evidence(path: Path) -> dict:
    from pypdf import PdfReader

    reader = PdfReader(str(path))
    text = "\n".join(page.extract_text() or "" for page in reader.pages)
    upper = text.upper()
    keywords = {keyword: upper.count(keyword) for keyword in ("TX1", "TX2", "TX3", "TX4", "RX1", "RX2", "RX3", "RX4", "GCPW", "ANTENNA", "PCB", "TOP", "BOTTOM")}
    return {"page_count": len(reader.pages), "extracted_text_chars": len(text), "keyword_counts": keywords}


def run(step: Path, assembly_pdf: Path, schematic_pdf: Path, output: Path) -> dict:
    step_bbox = _step_board_bbox(step)
    result = {
        "status": "completed_sprr440_mechanical_frame_extraction",
        "sources": {
            "step": {"path": str(step.resolve()), "sha256": _sha256(step)},
            "assembly_pdf": {"path": str(assembly_pdf.resolve()), "sha256": _sha256(assembly_pdf), **_pdf_evidence(assembly_pdf)},
            "schematic_pdf": {"path": str(schematic_pdf.resolve()), "sha256": _sha256(schematic_pdf), **_pdf_evidence(schematic_pdf)},
        },
        "step_board": step_bbox,
        "coordinate_status": "pcb_local_frame_confirmed_only",
        "radar_boresight_transform_status": "missing_mechanical_datum",
        "phase_center_status": "missing_electromagnetic_measurement",
        "hardware_validated": False,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "mechanical_frame.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    (output / "source_traceability.md").write_text(
        "# V0.4.88 SPRR440 机械坐标证据追踪\n\n"
        f"- STEP：`{step}`\n- STEP SHA-256：`{result['sources']['step']['sha256']}`\n"
        f"- 装配图：`{assembly_pdf}`\n- 装配图 SHA-256：`{result['sources']['assembly_pdf']['sha256']}`\n"
        f"- 原理图：`{schematic_pdf}`\n- 原理图 SHA-256：`{result['sources']['schematic_pdf']['sha256']}`\n\n"
        "## 已确认\n\n"
        f"STEP PCB BREP 的坐标包围盒为 `{step_bbox['bbox_mm']['x']['size']:.6f} × {step_bbox['bbox_mm']['y']['size']:.6f} × {step_bbox['bbox_mm']['z']['size']:.6f} mm`；顶面 z="
        f"{step_bbox['top_surface_z_mm']:.6f} mm，底面 z={step_bbox['bottom_surface_z_mm']:.6f} mm。STEP 使用毫米单位。\n\n"
        "## 尚未确认\n\n"
        "装配文件没有提供可直接读取的雷达前视基准、板面法向标注或安装旋转矩阵。因而当前只能冻结 PCB 局部坐标，不能把它直接当作雷达坐标；天线铜区几何中心也不能等同于电气相位中心。\n",
        encoding="utf-8",
    )
    report = [
        "# V0.4.88 SPRR440 机械坐标与装配资料分析",
        "",
        "## 结论",
        "",
        f"STEP 中 PCB 板体尺寸约为 {step_bbox['bbox_mm']['x']['size']:.3f} mm × {step_bbox['bbox_mm']['y']['size']:.3f} mm，厚度约 {step_bbox['bbox_mm']['z']['size']:.6f} mm。板体顶面为 z=0，底面为 z={step_bbox['bottom_surface_z_mm']:.6f} mm。",
        "",
        "这一步确认的是 PCB 自身的局部机械坐标，不是雷达坐标。当前仍缺少 PCB 安装到雷达/船体后的机械基准，因此不能直接决定方位正方向、俯仰正方向或镜像关系。",
        "",
        "## 对仿真的影响",
        "",
        "V0.4.85 的阵元位置可以继续作为 PCB 局部坐标候选；V0.4.86/V0.4.87 的结果应保留为‘局部坐标敏感性分析’。下一步必须对候选变换 `identity`、`mirror_x`、`mirror_y`、`rotate_180` 分别做已知角测试，再用角反射器实测选择唯一变换。",
        "",
        "## 边界",
        "",
        "STEP、装配 PDF 和原理图 PDF 没有提供电磁相位中心、天线方向图、互耦矩阵或 TI 校准矩阵；hardware_validated 保持为 false。",
        "",
    ]
    (output / "output_analysis.md").write_text("\n".join(report), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--step", type=Path, required=True)
    parser.add_argument("--assembly-pdf", type=Path, required=True)
    parser.add_argument("--schematic-pdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run(args.step.resolve(), args.assembly_pdf.resolve(), args.schematic_pdf.resolve(), args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
