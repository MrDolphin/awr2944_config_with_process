"""Extract document evidence for RF channel mapping without claiming phase centres."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

try:
    from pypdf import PdfReader
except ImportError:  # pragma: no cover - environment diagnostic
    PdfReader = None


RF_RE = re.compile(r"\b(?:TX|RX)[0-4](?:[_\-][PN])?\b|\b(?:ANT|RF|AWR2944)\w*", re.IGNORECASE)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def extract_pdf(path: Path) -> dict:
    result = {"path": str(path.resolve()), "file_sha256": sha256(path), "format": "pdf", "pages": None, "text_status": "unavailable", "rf_hits": []}
    if PdfReader is None:
        result["text_status"] = "pypdf_not_installed"
        return result
    try:
        reader = PdfReader(str(path))
        result["pages"] = len(reader.pages)
        hits = []
        for index, page in enumerate(reader.pages, start=1):
            text = page.extract_text() or ""
            matches = sorted(set(match.group(0) for match in RF_RE.finditer(text)))
            if matches:
                lines = [line.strip() for line in text.splitlines() if line.strip()]
                snippets = [line[:300] for line in lines if RF_RE.search(line)][:8]
                hits.append({"page": index, "tokens": matches, "snippets": snippets})
        result["text_status"] = "text_extracted"
        result["rf_hits"] = hits
    except Exception as exc:  # keep audit reproducible even for scanned/protected PDFs
        result["text_status"] = f"extract_error:{type(exc).__name__}"
    return result


def extract_schdoc(path: Path) -> dict:
    data = path.read_bytes()
    # Altium SchDoc is binary, but designators/net labels often remain in UTF-8
    # or UTF-16LE strings.  This is evidence discovery only, not a full parser.
    text = data.decode("utf-8", errors="ignore") + "\n" + data.decode("utf-16le", errors="ignore")
    tokens = sorted(set(match.group(0) for match in RF_RE.finditer(text)))
    return {"path": str(path.resolve()), "file_sha256": sha256(path), "format": "schdoc", "text_status": "binary_token_scan", "rf_hits": [{"tokens": tokens, "snippets": []}] if tokens else []}


def run(source_roots: list[Path], output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    docs = []
    for root in source_roots:
        for path in sorted(root.rglob("*")):
            if not path.is_file():
                continue
            if path.suffix.lower() == ".pdf":
                docs.append(extract_pdf(path))
            elif path.suffix.lower() == ".schdoc":
                docs.append(extract_schdoc(path))
    (output / "document_evidence.json").write_text(json.dumps({"documents": docs}, indent=2, ensure_ascii=False), encoding="utf-8")
    rows = []
    for doc in docs:
        for hit in doc["rf_hits"]:
            rows.append({"path": doc["path"], "format": doc["format"], "text_status": doc["text_status"], "page": hit.get("page", ""), "tokens": ",".join(hit.get("tokens", [])), "snippets": " || ".join(hit.get("snippets", [])), "evidence_status": "pdf_text_context" if doc["format"] == "pdf" else "binary_token_candidate"})
    with (output / "document_rf_token_candidates.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["path", "format", "text_status", "page", "tokens", "snippets", "evidence_status"])
        writer.writeheader(); writer.writerows(rows)
    summary = {"status": "completed_document_evidence_audit", "document_count": len(docs), "pdf_count": sum(doc["format"] == "pdf" for doc in docs), "schdoc_count": sum(doc["format"] == "schdoc" for doc in docs), "documents_with_rf_tokens": sum(bool(doc["rf_hits"]) for doc in docs), "phase_center_confirmed": False, "mapping_confirmed": False}
    (output / "document_evidence_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = ["# V0.4.30 原理图/装配/层叠文档证据审计", "", f"- 文档数：{summary['document_count']}", f"- PDF 数：{summary['pdf_count']}", f"- SchDoc 数：{summary['schdoc_count']}", f"- 包含 TX/RX/RF/AWR2944 字符串的文档：{summary['documents_with_rf_tokens']}", "", "## 证据边界", "", "PDF 文本提取和 SchDoc 字符串扫描只证明文档中出现了相关标识，不能证明网络到天线相位中心的映射。SchDoc 仍需要 Altium 原生解析或导出 ASCII/网表后才能进行引脚级连通性确认。", "当前没有把任何文档 token 升级为 confirmed phase centre 或 confirmed channel mapping。", ""]
    (output / "output_analysis.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", action="append", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.source_root, args.output.resolve()), indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
