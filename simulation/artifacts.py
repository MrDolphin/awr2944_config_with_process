"""Create non-overwriting, comparable simulation run directories."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from importlib.metadata import PackageNotFoundError, version
import platform
from pathlib import Path
import re
import subprocess
import sys
from uuid import uuid4


_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def _validate_identifier(name: str, value: str) -> str:
    if not _IDENTIFIER.fullmatch(value):
        raise ValueError(f"{name} must contain only letters, digits, dot, dash or underscore")
    return value


def _default_run_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    return f"{timestamp}_{uuid4().hex[:8]}"


def _git_commit() -> str | None:
    completed = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        capture_output=True,
        check=False,
        text=True,
    )
    return completed.stdout.strip() if completed.returncode == 0 else None


def _package_versions() -> dict[str, str]:
    versions: dict[str, str] = {}
    for package in ("numpy", "h5py", "matplotlib", "playwright"):
        try:
            versions[package] = version(package)
        except PackageNotFoundError:
            continue
    return versions


def create_run_directory(
    results_root: Path,
    *,
    producer: str,
    stage_id: str,
    run_id: str | None = None,
) -> Path:
    """Create a new stage/producer/run directory and refuse collisions."""

    producer = _validate_identifier("producer", producer)
    stage_id = _validate_identifier("stage_id", stage_id)
    resolved_run_id = _validate_identifier("run_id", run_id or _default_run_id())
    run_directory = Path(results_root).resolve() / producer / resolved_run_id
    run_directory.mkdir(parents=True, exist_ok=False)
    (run_directory / "data").mkdir()
    (run_directory / "figures").mkdir()

    created_utc = datetime.now(timezone.utc).isoformat()
    environment = {
        "stage_id": stage_id,
        "producer": producer,
        "run_id": resolved_run_id,
        "created_utc": created_utc,
        "git_commit": _git_commit(),
        "python_version": sys.version,
        "python_packages": _package_versions(),
        "platform": platform.platform(),
    }
    (run_directory / "environment.json").write_text(
        json.dumps(environment, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    (run_directory / "design_snapshot.md").write_text(
        f"# {stage_id} run {resolved_run_id}\n\n"
        f"- Producer: `{producer}`\n"
        f"- Created UTC: `{created_utc}`\n"
        "- Design source: see the stage README and copied run configuration.\n",
        encoding="utf-8",
    )
    (run_directory / "validation.md").write_text(
        "# Validation\n\n- [ ] Generation completed\n- [ ] Automatic checks passed\n"
        "- [ ] Manual review recorded\n",
        encoding="utf-8",
    )
    return run_directory
