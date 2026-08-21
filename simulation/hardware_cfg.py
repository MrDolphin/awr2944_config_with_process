"""Parse the AWR2944P CLI fields needed by the V0.4 AoA model."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
import re


@dataclass(frozen=True)
class Awr2944CfgSnapshot:
    source_cfg: str
    platform: str
    tx_mask: int
    rx_mask: int
    ant_geometry_tokens: list[float]
    azimuth_fov_deg: tuple[float, float]
    elevation_fov_deg: tuple[float, float]
    range_bias_m: float
    calibration_measure_enabled: bool
    calibration_compensation_is_identity: bool
    coordinate_status: str = "diagram_derived_or_pending_measurement"
    calibration_status: str = "not_measured"


def parse_cfg(path: Path) -> Awr2944CfgSnapshot:
    text = path.read_text(encoding="utf-8")
    platform_match = re.search(r"^% Platform:(.+)$", text, flags=re.MULTILINE)
    channel = re.search(r"^channelCfg\s+(\d+)\s+(\d+)", text, flags=re.MULTILINE)
    geometry = re.search(r"^antGeometryCfg\s+(.+)$", text, flags=re.MULTILINE)
    fov = re.search(r"^aoaFovCfg\s+-?\d+\s+([-+\d.]+)\s+([-+\d.]+)\s+([-+\d.]+)\s+([-+\d.]+)", text, flags=re.MULTILINE)
    compensation = re.search(r"^compRangeBiasAndRxChanPhase\s+(.+)$", text, flags=re.MULTILINE)
    measurement = re.search(r"^measureRangeBiasAndRxChanPhase\s+(\d+)\s+", text, flags=re.MULTILINE)
    if not all((platform_match, channel, geometry, fov, compensation, measurement)):
        raise ValueError(f"missing required AWR2944P CLI fields in {path}")
    comp_tokens = compensation.group(1).split()
    identity = len(comp_tokens) >= 3 and float(comp_tokens[0]) == 0.0 and all(
        float(value) in (0.0, 1.0) for value in comp_tokens[1:]
    )
    return Awr2944CfgSnapshot(
        source_cfg=str(path.resolve()), platform=platform_match.group(1).strip(),
        tx_mask=int(channel.group(1)), rx_mask=int(channel.group(2)),
        ant_geometry_tokens=[float(value) for value in geometry.group(1).split()],
        azimuth_fov_deg=(float(fov.group(1)), float(fov.group(2))),
        elevation_fov_deg=(float(fov.group(3)), float(fov.group(4))),
        range_bias_m=float(comp_tokens[0]),
        calibration_measure_enabled=measurement.group(1) == "1",
        calibration_compensation_is_identity=identity,
    )


def write_snapshot(cfg_path: Path, output_path: Path) -> None:
    snapshot = parse_cfg(cfg_path)
    payload = asdict(snapshot)
    payload["azimuth_fov_deg"] = list(snapshot.azimuth_fov_deg)
    payload["elevation_fov_deg"] = list(snapshot.elevation_fov_deg)
    output_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
