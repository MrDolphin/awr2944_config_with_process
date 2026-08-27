#!/usr/bin/env python3
"""Read-only deployment preflight for the Raspberry Pi radar service."""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shutil
import socket
import tempfile
from pathlib import Path


def check(name, ok, level="pass", detail=""):
    return {"name": name, "ok": bool(ok), "level": level if not ok else "pass", "detail": detail}


def run_preflight(args):
    results = []
    for package in ("serial", "websockets"):
        results.append(check(f"python:{package}", importlib.util.find_spec(package) is not None, "fail", "install the missing Python package"))
    cfg = Path(args.config).expanduser()
    results.append(check("config", cfg.is_file() and cfg.suffix == ".cfg", "fail", str(cfg)))
    root = Path(args.capture_root).expanduser()
    try:
        root.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=root, delete=True):
            pass
        results.append(check("capture_root", True, detail=str(root)))
    except OSError as exc:
        results.append(check("capture_root", False, "fail", str(exc)))
    free_mb = shutil.disk_usage(root if root.exists() else Path.cwd()).free // (1024 * 1024)
    results.append(check("disk_free", free_mb >= args.min_free_mb, "fail", f"{free_mb} MiB free; minimum {args.min_free_mb} MiB"))
    for label, port in (("cfg_port", args.cfg_port), ("data_port", args.data_port)):
        exists = Path(port).exists()
        results.append(check(label, exists, "fail" if args.require_ports else "warn", port))
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(("0.0.0.0", args.ws_port))
        results.append(check("ws_port", True, detail=str(args.ws_port)))
    except OSError as exc:
        results.append(check("ws_port", False, "warn", f"{args.ws_port}: {exc}"))
    finally:
        sock.close()
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--cfg-port", default="/dev/ttyACM0")
    parser.add_argument("--data-port", default="/dev/ttyACM1")
    parser.add_argument("--capture-root", default="captures/pointcloud_logs")
    parser.add_argument("--ws-port", type=int, default=8765)
    parser.add_argument("--min-free-mb", type=int, default=1024)
    parser.add_argument("--require-ports", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    results = run_preflight(args)
    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
    else:
        for result in results:
            print(f"[{result['level'].upper()}] {result['name']}: {result['detail']}")
    raise SystemExit(2 if any(not item["ok"] and item["level"] == "fail" for item in results) else 0)


if __name__ == "__main__":
    main()
