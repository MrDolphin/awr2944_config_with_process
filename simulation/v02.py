"""Validated dynamic-sea truth inputs for AWR2944P V0.2."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Union

import h5py
import numpy as np
from numpy.typing import NDArray


PathLike = Union[str, Path]
FloatArray = NDArray[np.float64]
MAX_SEA_STATE = 3
SCHEMA_VERSION = "awr2944p-dynamic-sea-v0.2"


def classify_sea_state(significant_wave_height_m: float) -> int:
    """Return the WMO 0--3 sea-state code for a wave height.

    Exact shared bounds stay in the lower state: 0.1 m is state 1,
    0.5 m is state 2, and 1.25 m is state 3.
    """

    if significant_wave_height_m < 0.0:
        raise ValueError("significant wave height must be nonnegative")
    for sea_state, upper_bound_m in ((0, 0.0), (1, 0.1), (2, 0.5), (3, 1.25)):
        if significant_wave_height_m <= upper_bound_m:
            return sea_state
    raise ValueError("V0.2 supports sea states 0 through 3 only (Hs <= 1.25 m)")


@dataclass(frozen=True)
class SeaStateCase:
    """One target-controlled sea-state case."""

    case_id: str
    sea_state: int
    target_hs_m: float
    label: str

    def __post_init__(self) -> None:
        if not 0 <= self.sea_state <= MAX_SEA_STATE:
            raise ValueError("sea_state must be between 0 and 3")
        classified = classify_sea_state(self.target_hs_m)
        if classified != self.sea_state:
            raise ValueError(
                f"{self.case_id} target Hs classifies as state {classified}, "
                f"not state {self.sea_state}"
            )


@dataclass(frozen=True)
class V02Config:
    """Public file-backed configuration for the V0.2-A truth sweep."""

    cases: tuple[SeaStateCase, ...]
    raw: dict[str, object]
    source_path: Path


@dataclass(frozen=True)
class DynamicSeaTruth:
    """Derived physical fields for one sampled dynamic sea surface."""

    sea_state: int
    target_hs_m: float
    achieved_hs_m: float
    hs_validation_passed: bool
    min_radar_clearance_m: float
    x_m: FloatArray
    y_m: FloatArray
    time_s: FloatArray
    height_m: FloatArray
    normal_x: FloatArray
    normal_y: FloatArray
    normal_z: FloatArray
    vertical_velocity_mps: FloatArray
    slant_range_m: FloatArray
    azimuth_deg: FloatArray
    elevation_deg: FloatArray
    grazing_angle_deg: FloatArray


@dataclass(frozen=True)
class RawSeaSurface:
    """Sampled sea height and provenance exchanged by MATLAB and Python."""

    case_id: str
    sea_state: int
    target_hs_m: float
    raw_hs_m: float
    amplitude_scale_factor: float
    random_seed: int
    wind_speed_mps: float
    wind_direction_deg: float
    fetch_m: float
    radar_height_m: float
    mounting_pitch_deg: float
    x_m: FloatArray
    y_m: FloatArray
    time_s: FloatArray
    height_m: FloatArray
    producer: str

    def __post_init__(self) -> None:
        if classify_sea_state(self.target_hs_m) != self.sea_state:
            raise ValueError("target_hs_m does not belong to sea_state")
        expected = (self.time_s.size, self.y_m.size, self.x_m.size)
        if self.height_m.shape != expected:
            raise ValueError(f"height_m must have shape {expected}")


def load_config(config_path: PathLike) -> V02Config:
    """Load and validate a V0.2-A JSON configuration."""

    path = Path(config_path).resolve()
    data = json.loads(path.read_text(encoding="utf-8"))
    cases = tuple(
        SeaStateCase(
            case_id=str(item["case_id"]),
            sea_state=int(item["sea_state"]),
            target_hs_m=float(item["target_hs_m"]),
            label=str(item["label"]),
        )
        for item in data["sea_states"]
    )
    if not cases:
        raise ValueError("sea_states must contain at least one case")
    if len({case.case_id for case in cases}) != len(cases):
        raise ValueError("sea-state case_id values must be unique")
    return V02Config(cases=cases, raw=data, source_path=path)


def analyze_height_cube(
    *,
    x_m: FloatArray,
    y_m: FloatArray,
    time_s: FloatArray,
    height_m: FloatArray,
    radar_height_m: float,
    mounting_pitch_deg: float,
    target_hs_m: float,
    hs_relative_tolerance: float,
    flat_hs_absolute_tolerance_m: float = 1e-12,
) -> DynamicSeaTruth:
    """Derive geometry and validation fields from a sampled sea-height cube.

    The height cube uses ``(time, y-forward, x-right)`` ordering. Vessel axes
    are x right, y forward, and z up. Positive mounting pitch points the radar
    boresight downward from horizontal, matching the V0.1 convention.
    """

    x = np.asarray(x_m, dtype=float)
    y = np.asarray(y_m, dtype=float)
    times = np.asarray(time_s, dtype=float)
    heights = np.asarray(height_m, dtype=float)
    expected_shape = (times.size, y.size, x.size)
    if heights.shape != expected_shape:
        raise ValueError(
            f"height_m must have shape (time, y, x)={expected_shape}, "
            f"got {heights.shape}"
        )
    if min(x.size, y.size, times.size) < 2:
        raise ValueError("x_m, y_m and time_s must each contain at least two values")
    if not (
        np.all(np.diff(x) > 0.0)
        and np.all(np.diff(y) > 0.0)
        and np.all(np.diff(times) > 0.0)
    ):
        raise ValueError("x_m, y_m and time_s must be strictly increasing")
    if radar_height_m <= 0.0:
        raise ValueError("radar_height_m must be positive")
    if hs_relative_tolerance < 0.0 or flat_hs_absolute_tolerance_m < 0.0:
        raise ValueError("Hs tolerances must be nonnegative")

    sea_state = classify_sea_state(target_hs_m)
    zero_mean_heights = heights - heights.mean(axis=(1, 2), keepdims=True)
    achieved_hs_m = 4.0 * float(np.std(zero_mean_heights))
    if target_hs_m == 0.0:
        hs_validation_passed = achieved_hs_m <= flat_hs_absolute_tolerance_m
    else:
        relative_error = abs(achieved_hs_m - target_hs_m) / target_hs_m
        hs_validation_passed = relative_error <= hs_relative_tolerance

    slope_y, slope_x = np.gradient(heights, y, x, axis=(1, 2), edge_order=1)
    normal_norm = np.sqrt(slope_x**2 + slope_y**2 + 1.0)
    normal_x = -slope_x / normal_norm
    normal_y = -slope_y / normal_norm
    normal_z = 1.0 / normal_norm
    vertical_velocity_mps = np.gradient(
        heights, times, axis=0, edge_order=1
    )

    x_grid, y_grid = np.meshgrid(x, y, indexing="xy")
    x_cube = np.broadcast_to(x_grid, heights.shape)
    y_cube = np.broadcast_to(y_grid, heights.shape)
    z_relative_m = heights - radar_height_m
    slant_range_m = np.sqrt(x_cube**2 + y_cube**2 + z_relative_m**2)

    pitch_rad = np.deg2rad(mounting_pitch_deg)
    radar_y_m = y_cube * np.cos(pitch_rad) - z_relative_m * np.sin(pitch_rad)
    radar_z_m = y_cube * np.sin(pitch_rad) + z_relative_m * np.cos(pitch_rad)
    azimuth_deg = np.rad2deg(np.arctan2(x_cube, radar_y_m))
    elevation_deg = np.rad2deg(
        np.arctan2(radar_z_m, np.hypot(x_cube, radar_y_m))
    )

    unit_to_radar_x = -x_cube / slant_range_m
    unit_to_radar_y = -y_cube / slant_range_m
    unit_to_radar_z = -z_relative_m / slant_range_m
    normal_projection = (
        normal_x * unit_to_radar_x
        + normal_y * unit_to_radar_y
        + normal_z * unit_to_radar_z
    )
    grazing_angle_deg = np.rad2deg(
        np.arcsin(np.clip(normal_projection, -1.0, 1.0))
    )

    return DynamicSeaTruth(
        sea_state=sea_state,
        target_hs_m=target_hs_m,
        achieved_hs_m=achieved_hs_m,
        hs_validation_passed=hs_validation_passed,
        min_radar_clearance_m=radar_height_m - float(heights.max()),
        x_m=x,
        y_m=y,
        time_s=times,
        height_m=heights,
        normal_x=normal_x,
        normal_y=normal_y,
        normal_z=normal_z,
        vertical_velocity_mps=vertical_velocity_mps,
        slant_range_m=slant_range_m,
        azimuth_deg=azimuth_deg,
        elevation_deg=elevation_deg,
        grazing_angle_deg=grazing_angle_deg,
    )


def normalize_height_cube(
    height_m: FloatArray, *, target_hs_m: float
) -> tuple[FloatArray, float, float]:
    """Zero-center and scale a spectral height cube to a target Hs.

    This controls wave amplitude while preserving the sampled spectral shape
    and phase evolution. The returned scale factor records that the result is
    target-controlled rather than an unmodified wind/fetch equilibrium sea.
    """

    classify_sea_state(target_hs_m)
    heights = np.asarray(height_m, dtype=float)
    if heights.ndim != 3:
        raise ValueError("height_m must have (time, y, x) dimensions")
    centered = heights - heights.mean(axis=(1, 2), keepdims=True)
    raw_hs_m = 4.0 * float(np.std(centered))
    if target_hs_m == 0.0:
        return np.zeros_like(centered), raw_hs_m, 0.0
    if raw_hs_m <= np.finfo(float).eps:
        raise ValueError("a nonzero target Hs requires a varying height cube")
    scale_factor = target_hs_m / raw_hs_m
    return centered * scale_factor, raw_hs_m, scale_factor


def write_raw_hdf5(surface: RawSeaSurface, output_path: PathLike) -> Path:
    """Write the cross-language V0.2 sampled-height contract."""

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(path, "w") as handle:
        handle.attrs["schema_version"] = SCHEMA_VERSION
        handle.attrs["producer"] = surface.producer
        handle.attrs["case_id"] = surface.case_id
        case = handle.create_group("case")
        for name, value in (
            ("sea_state", surface.sea_state),
            ("target_hs_m", surface.target_hs_m),
            ("raw_hs_m", surface.raw_hs_m),
            ("amplitude_scale_factor", surface.amplitude_scale_factor),
            ("random_seed", surface.random_seed),
            ("wind_speed_mps", surface.wind_speed_mps),
            ("wind_direction_deg", surface.wind_direction_deg),
            ("fetch_m", surface.fetch_m),
        ):
            case.create_dataset(name, data=value)
        installation = handle.create_group("installation")
        installation.create_dataset("height_m", data=surface.radar_height_m)
        installation.create_dataset(
            "mounting_pitch_deg", data=surface.mounting_pitch_deg
        )
        axes = handle.create_group("axes")
        axes.create_dataset("x_m", data=surface.x_m)
        axes.create_dataset("y_m", data=surface.y_m)
        axes.create_dataset("time_s", data=surface.time_s)
        truth = handle.create_group("truth")
        truth.create_dataset(
            "height_m",
            data=surface.height_m,
            compression="gzip",
            shuffle=True,
        )
    return path


def _attribute_text(value: object) -> str:
    if isinstance(value, (bytes, np.bytes_)):
        return bytes(value).decode("utf-8")
    if isinstance(value, np.ndarray) and value.size == 1:
        return _attribute_text(value.reshape(-1)[0])
    return str(value)


def _single_value(dataset: h5py.Dataset, dataset_path: str) -> float:
    value = np.asarray(dataset[()])
    if value.size != 1:
        raise ValueError(f"{dataset_path} must contain exactly one value")
    return float(value.reshape(-1)[0])


def read_raw_hdf5(input_path: PathLike) -> RawSeaSurface:
    """Read one MATLAB/Python V0.2 sampled-height file."""

    with h5py.File(input_path, "r") as handle:
        schema = _attribute_text(handle.attrs["schema_version"])
        if schema != SCHEMA_VERSION:
            raise ValueError(f"unsupported schema_version {schema!r}")
        producer = _attribute_text(handle.attrs["producer"])
        case_id = _attribute_text(handle.attrs["case_id"])
        x_m = np.asarray(handle["/axes/x_m"][...], dtype=float).reshape(-1)
        y_m = np.asarray(handle["/axes/y_m"][...], dtype=float).reshape(-1)
        time_s = np.asarray(handle["/axes/time_s"][...], dtype=float).reshape(-1)
        height_m = np.asarray(handle["/truth/height_m"][...], dtype=float)
        expected = (time_s.size, y_m.size, x_m.size)
        matlab_storage = (x_m.size, y_m.size, time_s.size)
        if height_m.shape == matlab_storage and height_m.shape != expected:
            height_m = np.transpose(height_m, (2, 1, 0))
        if height_m.shape != expected:
            raise ValueError(
                f"/truth/height_m must normalize to {expected}, got {height_m.shape}"
            )
        return RawSeaSurface(
            case_id=case_id,
            sea_state=int(_single_value(handle["/case/sea_state"], "/case/sea_state")),
            target_hs_m=_single_value(handle["/case/target_hs_m"], "/case/target_hs_m"),
            raw_hs_m=_single_value(handle["/case/raw_hs_m"], "/case/raw_hs_m"),
            amplitude_scale_factor=_single_value(
                handle["/case/amplitude_scale_factor"],
                "/case/amplitude_scale_factor",
            ),
            random_seed=int(
                _single_value(handle["/case/random_seed"], "/case/random_seed")
            ),
            wind_speed_mps=_single_value(
                handle["/case/wind_speed_mps"], "/case/wind_speed_mps"
            ),
            wind_direction_deg=_single_value(
                handle["/case/wind_direction_deg"],
                "/case/wind_direction_deg",
            ),
            fetch_m=_single_value(handle["/case/fetch_m"], "/case/fetch_m"),
            radar_height_m=_single_value(
                handle["/installation/height_m"], "/installation/height_m"
            ),
            mounting_pitch_deg=_single_value(
                handle["/installation/mounting_pitch_deg"],
                "/installation/mounting_pitch_deg",
            ),
            x_m=x_m,
            y_m=y_m,
            time_s=time_s,
            height_m=height_m,
            producer=producer,
        )


def write_truth_hdf5(
    raw: RawSeaSurface,
    truth: DynamicSeaTruth,
    output_path: PathLike,
) -> Path:
    """Write analyzed V0.2 fields while retaining raw-surface provenance."""

    if raw.height_m.shape != truth.height_m.shape:
        raise ValueError("raw and analyzed height cubes must have the same shape")
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(path, "w") as handle:
        handle.attrs["schema_version"] = SCHEMA_VERSION
        handle.attrs["producer"] = "python"
        handle.attrs["input_producer"] = raw.producer
        handle.attrs["case_id"] = raw.case_id
        axes = handle.create_group("axes")
        axes.create_dataset("x_m", data=truth.x_m)
        axes.create_dataset("y_m", data=truth.y_m)
        axes.create_dataset("time_s", data=truth.time_s)
        case = handle.create_group("case")
        for name, value in (
            ("sea_state", raw.sea_state),
            ("target_hs_m", raw.target_hs_m),
            ("raw_hs_m", raw.raw_hs_m),
            ("amplitude_scale_factor", raw.amplitude_scale_factor),
            ("random_seed", raw.random_seed),
            ("wind_speed_mps", raw.wind_speed_mps),
            ("wind_direction_deg", raw.wind_direction_deg),
            ("fetch_m", raw.fetch_m),
        ):
            case.create_dataset(name, data=value)
        installation = handle.create_group("installation")
        installation.create_dataset("height_m", data=raw.radar_height_m)
        installation.create_dataset(
            "mounting_pitch_deg", data=raw.mounting_pitch_deg
        )
        truth_group = handle.create_group("truth")
        for name in (
            "height_m",
            "normal_x",
            "normal_y",
            "normal_z",
            "vertical_velocity_mps",
            "slant_range_m",
            "azimuth_deg",
            "elevation_deg",
            "grazing_angle_deg",
        ):
            truth_group.create_dataset(
                name,
                data=getattr(truth, name),
                compression="gzip",
                shuffle=True,
            )
        validation = handle.create_group("validation")
        validation.create_dataset("achieved_hs_m", data=truth.achieved_hs_m)
        validation.create_dataset(
            "hs_validation_passed", data=truth.hs_validation_passed
        )
        validation.create_dataset(
            "min_radar_clearance_m", data=truth.min_radar_clearance_m
        )
    return path
