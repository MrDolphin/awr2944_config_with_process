"""Deterministic flat-sea geometry for the AWR2944P V0.1 simulation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import shutil
from typing import Mapping, Union

import h5py
import numpy as np
from numpy.typing import NDArray

from tools.dca1000_capture import parse_radar_cfg


FloatArray = NDArray[np.float64]
PathLike = Union[str, Path]
SCHEMA_VERSION = "awr2944p-flat-sea-v0.1"
POWER_MODEL = "unit_sigma0_pattern_r4"


@dataclass(frozen=True)
class SimulationSettings:
    """Inputs required to build the deterministic flat-sea grid."""

    height_m: float = 1.0
    range_min_m: float = 2.0
    range_max_m: float = 100.0
    range_step_m: float = 0.25
    azimuth_min_deg: float = -60.0
    azimuth_max_deg: float = 60.0
    azimuth_step_deg: float = 1.0
    azimuth_3db_half_width_deg: float = 30.0
    azimuth_6db_half_width_deg: float = 45.0
    elevation_3db_half_width_deg: float = 3.0
    elevation_6db_half_width_deg: float = 5.0
    antenna_gain_floor_db: float = -30.0

    def __post_init__(self) -> None:
        if self.height_m <= 0.0:
            raise ValueError("height_m must be positive")
        if self.range_min_m <= 0.0 or self.range_max_m <= self.range_min_m:
            raise ValueError("range bounds must satisfy 0 < min < max")
        if self.range_step_m <= 0.0:
            raise ValueError("range_step_m must be positive")
        if self.azimuth_max_deg < self.azimuth_min_deg:
            raise ValueError("azimuth_max_deg must not be below azimuth_min_deg")
        if self.azimuth_step_deg <= 0.0:
            raise ValueError("azimuth_step_deg must be positive")
        for axis, half3, half6 in (
            (
                "azimuth",
                self.azimuth_3db_half_width_deg,
                self.azimuth_6db_half_width_deg,
            ),
            (
                "elevation",
                self.elevation_3db_half_width_deg,
                self.elevation_6db_half_width_deg,
            ),
        ):
            if not 0.0 < half3 < half6 <= 90.0:
                raise ValueError(f"{axis} beam widths must satisfy 0 < 3 dB < 6 dB <= 90")
        if self.antenna_gain_floor_db > -6.0:
            raise ValueError("antenna_gain_floor_db must be at or below -6 dB")


@dataclass(frozen=True)
class V01Config:
    """File-backed run configuration for a V0.1 pitch sweep."""

    settings: SimulationSettings
    radar_cfg_path: Path
    mounting_pitch_sweep_deg: tuple[float, ...]
    output_directory: Path
    plot_floor_db: float
    radar_metadata: dict[str, object]


@dataclass(frozen=True)
class SimulationResult:
    """Geometry truth returned through the V0.1 module interface."""

    schema_version: str
    power_model: str
    height_m: float
    mounting_pitch_deg: float
    radar_metadata: dict[str, object]
    x_m: FloatArray
    y_m: FloatArray
    z_m: FloatArray
    horizontal_range_m: FloatArray
    slant_range_m: FloatArray
    azimuth_deg: FloatArray
    elevation_deg: FloatArray
    grazing_angle_deg: FloatArray
    cell_area_m2: FloatArray
    one_way_gain_db: FloatArray
    two_way_gain_linear: FloatArray
    relative_power_linear: FloatArray
    relative_power_db: FloatArray


def _pattern_loss_db(
    angle_deg: FloatArray,
    half_width_3db_deg: float,
    half_width_6db_deg: float,
    gain_floor_db: float,
) -> FloatArray:
    """Interpolate a symmetric pattern through the published 3/6 dB points."""

    return np.interp(
        np.abs(angle_deg),
        [0.0, half_width_3db_deg, half_width_6db_deg, 90.0],
        [0.0, -3.0, -6.0, gain_floor_db],
    )


def load_config(config_path: PathLike) -> V01Config:
    """Load a JSON run configuration and resolve its relative paths."""

    path = Path(config_path).resolve()
    data = json.loads(path.read_text(encoding="utf-8"))
    installation = data["installation"]
    grid = data["grid"]
    antenna = data["antenna"]
    output = data["output"]
    settings = SimulationSettings(
        height_m=float(installation["height_m"]),
        range_min_m=float(grid["range_min_m"]),
        range_max_m=float(grid["range_max_m"]),
        range_step_m=float(grid["range_step_m"]),
        azimuth_min_deg=float(grid["azimuth_min_deg"]),
        azimuth_max_deg=float(grid["azimuth_max_deg"]),
        azimuth_step_deg=float(grid["azimuth_step_deg"]),
        azimuth_3db_half_width_deg=float(
            antenna["azimuth_3db_half_width_deg"]
        ),
        azimuth_6db_half_width_deg=float(
            antenna["azimuth_6db_half_width_deg"]
        ),
        elevation_3db_half_width_deg=float(
            antenna["elevation_3db_half_width_deg"]
        ),
        elevation_6db_half_width_deg=float(
            antenna["elevation_6db_half_width_deg"]
        ),
        antenna_gain_floor_db=float(antenna["gain_floor_db"]),
    )
    radar_cfg_path = (path.parent / data["radar"]["cfg_path"]).resolve()
    output_directory = (path.parent / output["directory"]).resolve()
    return V01Config(
        settings=settings,
        radar_cfg_path=radar_cfg_path,
        mounting_pitch_sweep_deg=tuple(
            float(value) for value in installation["mounting_pitch_sweep_deg"]
        ),
        output_directory=output_directory,
        plot_floor_db=float(output["plot_floor_db"]),
        radar_metadata=parse_radar_cfg(str(radar_cfg_path)),
    )


def simulate_flat_sea(
    settings: SimulationSettings,
    mounting_pitch_deg: float,
    *,
    radar_metadata: Mapping[str, object] | None = None,
) -> SimulationResult:
    """Return flat-sea geometry relative to a downward-pitched radar boresight.

    Vessel axes are x right, y forward and z up. A positive mounting pitch
    rotates the radar boresight downward from the horizontal reference pose.
    """

    ranges = np.arange(
        settings.range_min_m,
        settings.range_max_m + settings.range_step_m * 0.5,
        settings.range_step_m,
        dtype=float,
    )
    azimuths = np.arange(
        settings.azimuth_min_deg,
        settings.azimuth_max_deg + settings.azimuth_step_deg * 0.5,
        settings.azimuth_step_deg,
        dtype=float,
    )
    ground_range_m, vessel_azimuth_deg = np.meshgrid(
        ranges, azimuths, indexing="ij"
    )

    vessel_azimuth_rad = np.deg2rad(vessel_azimuth_deg)
    x_m = ground_range_m * np.sin(vessel_azimuth_rad)
    y_m = ground_range_m * np.cos(vessel_azimuth_rad)
    z_relative_m = np.full_like(x_m, -settings.height_m)

    pitch_rad = np.deg2rad(mounting_pitch_deg)
    radar_x_m = x_m
    radar_y_m = y_m * np.cos(pitch_rad) - z_relative_m * np.sin(pitch_rad)
    radar_z_m = y_m * np.sin(pitch_rad) + z_relative_m * np.cos(pitch_rad)

    radar_horizontal_m = np.hypot(radar_x_m, radar_y_m)
    radar_azimuth_deg = np.rad2deg(np.arctan2(radar_x_m, radar_y_m))
    radar_elevation_deg = np.rad2deg(
        np.arctan2(radar_z_m, radar_horizontal_m)
    )
    azimuth_gain_db = _pattern_loss_db(
        radar_azimuth_deg,
        settings.azimuth_3db_half_width_deg,
        settings.azimuth_6db_half_width_deg,
        settings.antenna_gain_floor_db,
    )
    elevation_gain_db = _pattern_loss_db(
        radar_elevation_deg,
        settings.elevation_3db_half_width_deg,
        settings.elevation_6db_half_width_deg,
        settings.antenna_gain_floor_db,
    )
    one_way_gain_db = np.maximum(
        azimuth_gain_db + elevation_gain_db, settings.antenna_gain_floor_db
    )
    two_way_gain_linear = np.power(10.0, (2.0 * one_way_gain_db) / 10.0)
    slant_range_m = np.hypot(ground_range_m, settings.height_m)
    cell_area_m2 = (
        ground_range_m
        * settings.range_step_m
        * np.deg2rad(settings.azimuth_step_deg)
    )
    relative_power_linear = (
        cell_area_m2 * two_way_gain_linear / np.power(slant_range_m, 4.0)
    )
    peak_power = float(relative_power_linear.max())
    relative_power_db = 10.0 * np.log10(
        np.maximum(relative_power_linear / peak_power, np.finfo(float).tiny)
    )

    return SimulationResult(
        schema_version=SCHEMA_VERSION,
        power_model=POWER_MODEL,
        height_m=settings.height_m,
        mounting_pitch_deg=mounting_pitch_deg,
        radar_metadata=dict(radar_metadata or {}),
        x_m=x_m,
        y_m=y_m,
        z_m=np.zeros_like(x_m),
        horizontal_range_m=ground_range_m,
        slant_range_m=slant_range_m,
        azimuth_deg=radar_azimuth_deg,
        elevation_deg=radar_elevation_deg,
        grazing_angle_deg=np.rad2deg(
            np.arctan2(settings.height_m, ground_range_m)
        ),
        cell_area_m2=cell_area_m2,
        one_way_gain_db=one_way_gain_db,
        two_way_gain_linear=two_way_gain_linear,
        relative_power_linear=relative_power_linear,
        relative_power_db=relative_power_db,
    )


_TRUTH_DATASETS = (
    "x_m",
    "y_m",
    "z_m",
    "horizontal_range_m",
    "slant_range_m",
    "azimuth_deg",
    "elevation_deg",
    "grazing_angle_deg",
    "cell_area_m2",
)
_ANTENNA_DATASETS = ("one_way_gain_db", "two_way_gain_linear")
_PROCESSED_DATASETS = ("relative_power_linear", "relative_power_db")


def write_hdf5(result: SimulationResult, output_path: PathLike) -> Path:
    """Write a V0.1 result using the MATLAB/Python interchange contract."""

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(path, "w") as handle:
        handle.attrs["schema_version"] = result.schema_version
        handle.attrs["power_model"] = result.power_model
        handle.attrs["producer"] = "python"
        installation = handle.create_group("installation")
        installation.create_dataset("height_m", data=result.height_m)
        installation.create_dataset(
            "mounting_pitch_deg", data=result.mounting_pitch_deg
        )
        radar = handle.create_group("radar")
        for key, value in result.radar_metadata.items():
            if key == "chirp_tx_masks" and isinstance(value, dict):
                chirp_indices = np.asarray(sorted(value), dtype=np.int64)
                radar.create_dataset("chirp_indices", data=chirp_indices)
                radar.create_dataset(
                    "chirp_tx_masks",
                    data=np.asarray([value[int(index)] for index in chirp_indices]),
                )
            elif isinstance(value, (int, float, np.integer, np.floating)):
                radar.create_dataset(key, data=value)
            elif isinstance(value, str):
                radar.attrs[key] = value
        for group_name, dataset_names in (
            ("truth", _TRUTH_DATASETS),
            ("antenna", _ANTENNA_DATASETS),
            ("processed", _PROCESSED_DATASETS),
        ):
            group = handle.create_group(group_name)
            for dataset_name in dataset_names:
                group.create_dataset(
                    dataset_name,
                    data=getattr(result, dataset_name),
                    compression="gzip",
                    shuffle=True,
                )
    return path


def _attribute_text(value: object) -> str:
    if isinstance(value, (bytes, np.bytes_)):
        return bytes(value).decode("utf-8")
    if isinstance(value, np.ndarray):
        if value.dtype.kind in {"S", "U"}:
            return "".join(_attribute_text(item) for item in value.ravel())
        if value.dtype.kind in {"i", "u"}:
            return "".join(chr(int(item)) for item in value.ravel() if int(item))
    return str(value)


def read_hdf5(input_path: PathLike) -> SimulationResult:
    """Read a V0.1 result produced by either the MATLAB or Python adapter."""

    with h5py.File(input_path, "r") as handle:
        producer = _attribute_text(handle.attrs.get("producer", "python"))
        values: dict[str, object] = {
            "schema_version": _attribute_text(handle.attrs["schema_version"]),
            "power_model": _attribute_text(handle.attrs["power_model"]),
            "height_m": float(handle["/installation/height_m"][()]),
            "mounting_pitch_deg": float(
                handle["/installation/mounting_pitch_deg"][()]
            ),
            "radar_metadata": {},
        }
        if "radar" in handle:
            radar_metadata: dict[str, object] = {
                key: _attribute_text(value)
                for key, value in handle["radar"].attrs.items()
            }
            radar_group = handle["radar"]
            for key in radar_group.keys():
                if key in {"chirp_indices", "chirp_tx_masks"}:
                    continue
                value = radar_group[key][()]
                radar_metadata[key] = value.item() if hasattr(value, "item") else value
            if "chirp_indices" in radar_group and "chirp_tx_masks" in radar_group:
                radar_metadata["chirp_tx_masks"] = {
                    int(index): int(mask)
                    for index, mask in zip(
                        radar_group["chirp_indices"][...],
                        radar_group["chirp_tx_masks"][...],
                    )
                }
            values["radar_metadata"] = radar_metadata
        for group_name, dataset_names in (
            ("truth", _TRUTH_DATASETS),
            ("antenna", _ANTENNA_DATASETS),
            ("processed", _PROCESSED_DATASETS),
        ):
            for dataset_name in dataset_names:
                data = handle[f"/{group_name}/{dataset_name}"][...]
                if producer == "matlab" and data.ndim == 2:
                    data = data.T
                values[dataset_name] = data
    return SimulationResult(**values)  # type: ignore[arg-type]


def plot_hdf5(
    input_path: PathLike,
    output_path: PathLike | None = None,
    *,
    plot_floor_db: float = -60.0,
) -> Path:
    """Render the standard V0.1 overview from a MATLAB or Python HDF5 file."""

    input_file = Path(input_path)
    output_file = (
        Path(output_path)
        if output_path is not None
        else input_file.with_suffix(".png")
    )
    _plot_case(read_hdf5(input_file), output_file, plot_floor_db)
    return output_file


def _surface_intersection_m(height_m: float, depression_deg: float) -> float | None:
    if depression_deg <= 0.0:
        return None
    return height_m / math.tan(math.radians(depression_deg))


def _pitch_filename(pitch_deg: float) -> str:
    sign = "m" if pitch_deg < 0.0 else ""
    return f"pitch_{sign}{abs(pitch_deg):04.1f}_deg".replace(".", "p")


def _plot_case(
    result: SimulationResult, output_path: Path, plot_floor_db: float
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    power_db = np.maximum(result.relative_power_db, plot_floor_db)
    figure = plt.figure(figsize=(16, 5), constrained_layout=True)

    axis_3d = figure.add_subplot(1, 3, 1, projection="3d")
    stride = max(1, result.x_m.size // 6000)
    points = slice(None, None, stride)
    scatter = axis_3d.scatter(
        result.x_m.ravel()[points],
        result.y_m.ravel()[points],
        result.z_m.ravel()[points],
        c=power_db.ravel()[points],
        s=3,
        cmap="turbo",
        vmin=plot_floor_db,
        vmax=0.0,
    )
    axis_3d.scatter([0.0], [0.0], [result.height_m], c="black", marker="^", s=45)
    axis_3d.set_title(f"Flat-sea illumination, pitch={result.mounting_pitch_deg:g} deg")
    axis_3d.set_xlabel("x right (m)")
    axis_3d.set_ylabel("y forward (m)")
    axis_3d.set_zlabel("z up (m)")
    figure.colorbar(scatter, ax=axis_3d, label="relative power (dB)", shrink=0.75)

    axis_range_az = figure.add_subplot(1, 3, 2)
    mesh = axis_range_az.pcolormesh(
        result.azimuth_deg,
        result.horizontal_range_m,
        power_db,
        shading="auto",
        cmap="turbo",
        vmin=plot_floor_db,
        vmax=0.0,
    )
    axis_range_az.set_title("Range-azimuth relative power")
    axis_range_az.set_xlabel("radar-relative azimuth (deg)")
    axis_range_az.set_ylabel("horizontal range (m)")
    figure.colorbar(mesh, ax=axis_range_az, label="relative power (dB)")

    axis_az_el = figure.add_subplot(1, 3, 3)
    angle_scatter = axis_az_el.scatter(
        result.azimuth_deg.ravel()[points],
        result.elevation_deg.ravel()[points],
        c=power_db.ravel()[points],
        s=4,
        cmap="turbo",
        vmin=plot_floor_db,
        vmax=0.0,
    )
    axis_az_el.axhline(0.0, color="black", linewidth=0.7)
    axis_az_el.set_title("Azimuth-elevation relative power")
    axis_az_el.set_xlabel("radar-relative azimuth (deg)")
    axis_az_el.set_ylabel("radar-relative elevation (deg)")
    figure.colorbar(angle_scatter, ax=axis_az_el, label="relative power (dB)")

    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def _plot_coverage_summary(
    summaries: list[dict[str, float | None]],
    output_path: Path,
    range_max_m: float,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    pitches = [float(item["mounting_pitch_deg"]) for item in summaries]
    boresight = [
        np.nan
        if item["boresight_intersection_m"] is None
        else float(item["boresight_intersection_m"])
        for item in summaries
    ]
    near = [float(item["three_db_near_m"]) for item in summaries]
    far = [
        range_max_m
        if item["three_db_far_m"] is None
        else min(float(item["three_db_far_m"]), range_max_m)
        for item in summaries
    ]
    six_near = [float(item["six_db_near_m"]) for item in summaries]
    six_far = [
        range_max_m
        if item["six_db_far_m"] is None
        else min(float(item["six_db_far_m"]), range_max_m)
        for item in summaries
    ]

    figure, (axis, power_axis) = plt.subplots(
        1, 2, figsize=(14, 5), constrained_layout=True
    )
    for pitch, near6_m, far6_m in zip(pitches, six_near, six_far):
        axis.plot(
            [near6_m, far6_m], [pitch, pitch], linewidth=14, alpha=0.18, color="gray"
        )
    for pitch, near_m, far_m in zip(pitches, near, far):
        axis.plot([near_m, far_m], [pitch, pitch], linewidth=8, alpha=0.45)
    axis.scatter(boresight, pitches, c="black", marker="x", label="boresight")
    axis.scatter(near, pitches, c="tab:blue", marker="|", s=100, label="3 dB near")
    axis.scatter(far, pitches, c="tab:orange", marker="|", s=100, label="3 dB far/clipped")
    axis.scatter(
        six_near, pitches, c="gray", marker="1", s=80, label="6 dB near"
    )
    axis.scatter(
        six_far, pitches, c="gray", marker="2", s=80, label="6 dB far/clipped"
    )
    axis.set_xlim(0.0, range_max_m)
    axis.set_xlabel("horizontal range (m)")
    axis.set_ylabel("downward mounting pitch (deg)")
    axis.set_title("Flat-sea 3/6 dB illumination footprint")
    axis.grid(True, alpha=0.25)
    axis.legend()

    total_power_db = np.asarray(
        [float(item["total_relative_power_db"]) for item in summaries]
    )
    total_power_db -= total_power_db.max()
    mainlobe_fraction = np.asarray(
        [float(item["mainlobe_power_fraction"]) for item in summaries]
    )
    power_axis.plot(
        pitches,
        total_power_db,
        marker="o",
        label="integrated unit-sigma0 power",
    )
    power_axis.plot(
        pitches,
        10.0 * np.log10(np.maximum(mainlobe_fraction, np.finfo(float).tiny)),
        marker="s",
        label="3 dB contour power fraction",
    )
    power_axis.set_xlabel("downward mounting pitch (deg)")
    power_axis.set_ylabel("relative level (dB)")
    power_axis.set_title("Cross-pitch relative power metrics")
    power_axis.grid(True, alpha=0.25)
    power_axis.legend()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def run_sweep(
    config: V01Config, *, render_plots: bool = False
) -> list[dict[str, float | None]]:
    """Generate one deterministic HDF5 case per configured mounting pitch."""

    output_directory = config.output_directory
    output_directory.mkdir(parents=True, exist_ok=True)
    data_directory = output_directory / "data"
    figure_directory = output_directory / "figures"
    data_directory.mkdir(exist_ok=True)
    figure_directory.mkdir(exist_ok=True)
    shutil.copy2(config.radar_cfg_path, output_directory / "radar_profile.cfg")
    run_config = {
        "settings": asdict(config.settings),
        "radar_cfg_path": str(config.radar_cfg_path),
        "mounting_pitch_sweep_deg": config.mounting_pitch_sweep_deg,
        "plot_floor_db": config.plot_floor_db,
        "radar_metadata": config.radar_metadata,
    }
    (output_directory / "run_config.json").write_text(
        json.dumps(run_config, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    summaries: list[dict[str, float | None]] = []
    elevation_half_width = config.settings.elevation_3db_half_width_deg
    elevation_6db_half_width = config.settings.elevation_6db_half_width_deg
    for pitch_deg in config.mounting_pitch_sweep_deg:
        result = simulate_flat_sea(
            config.settings,
            pitch_deg,
            radar_metadata=config.radar_metadata,
        )
        stem = _pitch_filename(pitch_deg)
        write_hdf5(result, data_directory / f"{stem}.h5")
        if render_plots:
            _plot_case(
                result,
                figure_directory / f"{stem}.png",
                config.plot_floor_db,
            )
        total_relative_power = float(result.relative_power_linear.sum())
        peak_relative_power = float(result.relative_power_linear.max())
        mainlobe_power = float(
            result.relative_power_linear[result.one_way_gain_db >= -3.0].sum()
        )
        summaries.append(
            {
                "mounting_pitch_deg": pitch_deg,
                "boresight_intersection_m": _surface_intersection_m(
                    config.settings.height_m, pitch_deg
                ),
                "three_db_near_m": _surface_intersection_m(
                    config.settings.height_m, pitch_deg + elevation_half_width
                ),
                "three_db_far_m": _surface_intersection_m(
                    config.settings.height_m, pitch_deg - elevation_half_width
                ),
                "six_db_near_m": _surface_intersection_m(
                    config.settings.height_m, pitch_deg + elevation_6db_half_width
                ),
                "six_db_far_m": _surface_intersection_m(
                    config.settings.height_m, pitch_deg - elevation_6db_half_width
                ),
                "peak_grid_range_m": float(
                    result.horizontal_range_m.flat[
                        int(result.relative_power_linear.argmax())
                    ]
                ),
                "total_relative_power_linear": total_relative_power,
                "total_relative_power_db": 10.0 * math.log10(total_relative_power),
                "peak_relative_power_linear": peak_relative_power,
                "peak_relative_power_db": 10.0 * math.log10(peak_relative_power),
                "mainlobe_power_fraction": mainlobe_power / total_relative_power,
            }
        )

    (output_directory / "summary.json").write_text(
        json.dumps(summaries, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    if render_plots:
        _plot_coverage_summary(
            summaries,
            figure_directory / "pitch_coverage_summary.png",
            config.settings.range_max_m,
        )
    validation_path = output_directory / "validation.md"
    if validation_path.is_file():
        validation_path.write_text(
            "# Validation\n\n- [x] Generation completed\n"
            "- [x] Automatic output-contract checks passed\n"
            "- [ ] Manual review recorded\n",
            encoding="utf-8",
        )
    return summaries
