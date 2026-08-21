import json
from pathlib import Path
import tempfile
import unittest

import h5py
import numpy as np

from simulation.run_v02 import analyze_matlab_run
from simulation.v02 import (
    analyze_height_cube,
    classify_sea_state,
    load_config,
    normalize_height_cube,
    RawSeaSurface,
    read_raw_hdf5,
    SeaStateCase,
    write_raw_hdf5,
    write_truth_hdf5,
)


class DynamicSeaTruthTests(unittest.TestCase):
    def test_default_config_covers_zero_through_three_without_exceeding_limit(self):
        config = load_config(
            Path("simulation/configs/sea_states_0_to_3.json").resolve()
        )

        self.assertEqual(
            tuple(case.case_id for case in config.cases),
            (
                "ss0_flat",
                "ss1_rippled",
                "ss2_normal",
                "ss3_nominal",
                "ss3_upper",
            ),
        )
        self.assertEqual(
            tuple(case.target_hs_m for case in config.cases),
            (0.0, 0.05, 0.30, 0.85, 1.00),
        )
        self.assertEqual(
            tuple(classify_sea_state(case.target_hs_m) for case in config.cases),
            (0, 1, 2, 3, 3),
        )
        self.assertTrue(all(case.sea_state <= 3 for case in config.cases))
        self.assertEqual(config.raw["sea_surface"]["wind_direction_deg"], 90.0)

    def test_sea_state_above_three_or_hs_above_limit_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "between 0 and 3"):
            SeaStateCase(
                case_id="ss4_forbidden",
                sea_state=4,
                target_hs_m=1.30,
                label="forbidden",
            )
        self.assertEqual(classify_sea_state(1.00), 3)
        self.assertEqual(classify_sea_state(1.01), 3)
        with self.assertRaisesRegex(ValueError, "project limit.*1.00 m"):
            SeaStateCase(
                case_id="ss3_above_project_limit",
                sea_state=3,
                target_hs_m=1.01,
                label="forbidden",
            )

    def test_flat_height_cube_returns_zero_hs_and_flat_surface_geometry(self):
        x_m = np.asarray([-1.0, 0.0, 1.0])
        y_m = np.asarray([10.0, 11.0])
        time_s = np.asarray([0.0, 0.5, 1.0])
        height_m = np.zeros((time_s.size, y_m.size, x_m.size))

        result = analyze_height_cube(
            x_m=x_m,
            y_m=y_m,
            time_s=time_s,
            height_m=height_m,
            radar_height_m=1.0,
            mounting_pitch_deg=5.0,
            target_hs_m=0.0,
            hs_relative_tolerance=0.10,
        )

        self.assertEqual(result.sea_state, 0)
        self.assertEqual(result.achieved_hs_m, 0.0)
        self.assertTrue(result.hs_validation_passed)
        self.assertAlmostEqual(result.min_radar_clearance_m, 1.0)
        self.assertTrue(np.array_equal(result.normal_x, np.zeros_like(height_m)))
        self.assertTrue(np.array_equal(result.normal_y, np.zeros_like(height_m)))
        self.assertTrue(np.array_equal(result.normal_z, np.ones_like(height_m)))
        self.assertTrue(
            np.array_equal(result.vertical_velocity_mps, np.zeros_like(height_m))
        )
        self.assertAlmostEqual(
            result.grazing_angle_deg[0, 0, 1],
            np.degrees(np.arctan2(1.0, 10.0)),
        )

    def test_height_cube_is_normalized_to_requested_state_three_hs(self):
        raw = np.asarray(
            [
                [[-0.2, 0.0, 0.2], [-0.1, 0.1, 0.3]],
                [[0.2, 0.0, -0.2], [0.1, -0.1, -0.3]],
            ],
            dtype=float,
        )

        normalized, raw_hs_m, scale_factor = normalize_height_cube(
            raw, target_hs_m=1.00
        )

        self.assertGreater(raw_hs_m, 0.0)
        self.assertGreater(scale_factor, 0.0)
        self.assertAlmostEqual(
            4.0
            * float(
                np.std(
                    normalized
                    - normalized.mean(axis=(1, 2), keepdims=True)
                )
            ),
            1.00,
            places=12,
        )
        self.assertTrue(
            np.allclose(normalized.mean(axis=(1, 2)), 0.0, atol=1e-15)
        )

    def test_plane_wave_reports_direction_period_phase_speed_and_range_rate(self):
        x_m = np.arange(-8.0, 8.0, 1.0)
        y_m = np.arange(2.0, 18.0, 1.0)
        time_s = np.arange(0.0, 8.0, 0.25)
        wavelength_m = 8.0
        period_s = 4.0
        phase = 2.0 * np.pi * (
            y_m[None, :, None] / wavelength_m
            - time_s[:, None, None] / period_s
        )
        height_m = 0.1 * np.cos(phase) * np.ones((1, 1, x_m.size))

        result = analyze_height_cube(
            x_m=x_m,
            y_m=y_m,
            time_s=time_s,
            height_m=height_m,
            radar_height_m=1.0,
            mounting_pitch_deg=5.0,
            target_hs_m=4.0 * float(np.std(height_m)),
            hs_relative_tolerance=0.10,
        )

        self.assertAlmostEqual(result.dominant_wave_direction_deg, 0.0, places=6)
        self.assertAlmostEqual(result.dominant_wave_period_s, period_s, places=6)
        self.assertAlmostEqual(
            result.dominant_wavelength_m, wavelength_m, places=6
        )
        self.assertAlmostEqual(result.dominant_phase_speed_mps, 2.0, places=6)
        self.assertEqual(result.slant_range_rate_mps.shape, height_m.shape)

    def test_rightward_plane_wave_reports_positive_ninety_degree_direction(self):
        x_m = np.arange(-8.0, 8.0, 1.0)
        y_m = np.arange(2.0, 18.0, 1.0)
        time_s = np.arange(0.0, 8.0, 0.25)
        phase = 2.0 * np.pi * (
            x_m[None, None, :] / 8.0 - time_s[:, None, None] / 4.0
        )
        height_m = 0.1 * np.cos(phase) * np.ones((1, y_m.size, 1))

        result = analyze_height_cube(
            x_m=x_m,
            y_m=y_m,
            time_s=time_s,
            height_m=height_m,
            radar_height_m=1.0,
            mounting_pitch_deg=5.0,
            target_hs_m=4.0 * float(np.std(height_m)),
            hs_relative_tolerance=0.10,
        )

        self.assertAlmostEqual(result.dominant_wave_direction_deg, 90.0, places=6)

    def test_reverse_plane_waves_report_negative_axis_directions(self):
        x_m = np.arange(-8.0, 8.0, 1.0)
        y_m = np.arange(2.0, 18.0, 1.0)
        time_s = np.arange(0.0, 8.0, 0.25)
        x_grid, y_grid, time_grid = np.meshgrid(
            x_m, y_m, time_s, indexing="xy"
        )
        for expected_direction_deg, phase in (
            (
                -180.0,
                2.0 * np.pi * (-y_grid / 8.0 - time_grid / 4.0),
            ),
            (
                -90.0,
                2.0 * np.pi * (-x_grid / 8.0 - time_grid / 4.0),
            ),
        ):
            height_m = 0.1 * np.cos(phase).transpose(2, 0, 1)
            result = analyze_height_cube(
                x_m=x_m,
                y_m=y_m,
                time_s=time_s,
                height_m=height_m,
                radar_height_m=1.0,
                mounting_pitch_deg=5.0,
                target_hs_m=4.0 * float(np.std(height_m)),
                hs_relative_tolerance=0.10,
            )
            self.assertAlmostEqual(
                result.dominant_wave_direction_deg,
                expected_direction_deg,
                places=6,
            )

    def test_slant_range_rate_uses_line_of_sight_projection(self):
        x_m = np.asarray([-1.0, 1.0])
        y_m = np.asarray([2.0, 3.0])
        time_s = np.asarray([0.0, 0.5, 1.0])
        height_m = 0.10 * time_s[:, None, None] * np.ones((1, 2, 2))

        result = analyze_height_cube(
            x_m=x_m,
            y_m=y_m,
            time_s=time_s,
            height_m=height_m,
            radar_height_m=1.0,
            mounting_pitch_deg=5.0,
            target_hs_m=0.0,
            hs_relative_tolerance=0.10,
        )

        expected = (height_m - 1.0) * 0.10 / result.slant_range_m
        self.assertTrue(np.allclose(result.slant_range_rate_mps, expected))
        self.assertTrue(np.all(result.slant_range_rate_mps < 0.0))

    def test_nonuniform_axes_are_rejected_before_spectral_estimation(self):
        with self.assertRaisesRegex(ValueError, "uniformly spaced"):
            analyze_height_cube(
                x_m=np.asarray([-1.0, 0.0, 2.0]),
                y_m=np.asarray([2.0, 3.0]),
                time_s=np.asarray([0.0, 0.5, 1.0]),
                height_m=np.zeros((3, 2, 3)),
                radar_height_m=1.0,
                mounting_pitch_deg=5.0,
                target_hs_m=0.0,
                hs_relative_tolerance=0.10,
            )

    def test_all_default_target_wave_heights_pass_after_amplitude_control(self):
        config = load_config(
            Path("simulation/configs/sea_states_0_to_3.json").resolve()
        )
        x_m = np.asarray([-1.0, 0.0, 1.0])
        y_m = np.asarray([2.0, 3.0])
        time_s = np.asarray([0.0, 0.5, 1.0])
        varying = np.arange(18, dtype=float).reshape(3, 2, 3)

        for case in config.cases:
            normalized = normalize_height_cube(
                varying, target_hs_m=case.target_hs_m
            )[0]
            truth = analyze_height_cube(
                x_m=x_m,
                y_m=y_m,
                time_s=time_s,
                height_m=normalized,
                radar_height_m=1.0,
                mounting_pitch_deg=5.0,
                target_hs_m=case.target_hs_m,
                hs_relative_tolerance=0.10,
            )
            self.assertTrue(truth.hs_validation_passed, case.case_id)
            self.assertAlmostEqual(
                truth.achieved_hs_m, case.target_hs_m, places=12
            )

    def test_raw_dynamic_surface_hdf5_round_trip_preserves_axes_and_provenance(self):
        surface = RawSeaSurface(
            case_id="ss2_normal",
            sea_state=2,
            target_hs_m=0.30,
            raw_hs_m=0.42,
            amplitude_scale_factor=0.30 / 0.42,
            random_seed=101,
            wind_speed_mps=3.5,
            wind_direction_deg=0.0,
            fetch_m=10000.0,
            radar_height_m=1.0,
            mounting_pitch_deg=5.0,
            x_m=np.asarray([-1.0, 0.0, 1.0]),
            y_m=np.asarray([2.0, 3.0]),
            time_s=np.asarray([0.0, 0.5]),
            height_m=np.arange(12, dtype=float).reshape(2, 2, 3) / 100.0,
            producer="python-test",
        )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "surface.h5"
            write_raw_hdf5(surface, path)
            loaded = read_raw_hdf5(path)

        self.assertEqual(loaded.case_id, surface.case_id)
        self.assertEqual(loaded.sea_state, surface.sea_state)
        self.assertAlmostEqual(loaded.target_hs_m, surface.target_hs_m)
        self.assertAlmostEqual(loaded.raw_hs_m, surface.raw_hs_m)
        self.assertEqual(loaded.random_seed, 101)
        self.assertTrue(np.array_equal(loaded.x_m, surface.x_m))
        self.assertTrue(np.array_equal(loaded.time_s, surface.time_s))
        self.assertTrue(np.array_equal(loaded.height_m, surface.height_m))

    def test_reader_normalizes_matlab_attributes_scalars_and_cube_layout(self):
        height_m = np.arange(24, dtype=float).reshape(2, 3, 4)
        surface = RawSeaSurface(
            case_id="ss2_normal",
            sea_state=2,
            target_hs_m=0.30,
            raw_hs_m=0.40,
            amplitude_scale_factor=0.75,
            random_seed=101,
            wind_speed_mps=3.5,
            wind_direction_deg=0.0,
            fetch_m=10000.0,
            radar_height_m=1.0,
            mounting_pitch_deg=5.0,
            x_m=np.asarray([-1.5, -0.5, 0.5, 1.5]),
            y_m=np.asarray([2.0, 3.0, 4.0]),
            time_s=np.asarray([0.0, 0.5]),
            height_m=height_m,
            producer="matlab",
        )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "matlab_layout.h5"
            write_raw_hdf5(surface, path)
            with h5py.File(path, "r+") as handle:
                for name in ("producer", "case_id", "schema_version"):
                    value = str(handle.attrs[name])
                    del handle.attrs[name]
                    handle.attrs.create(
                        name,
                        np.asarray(
                            [value], dtype=h5py.string_dtype(encoding="utf-8")
                        ),
                    )
                del handle["/truth/height_m"]
                handle.create_dataset(
                    "/truth/height_m",
                    data=np.transpose(height_m, (2, 1, 0)),
                )
                for dataset_path in (
                    "/case/sea_state",
                    "/case/target_hs_m",
                    "/installation/height_m",
                ):
                    value = handle[dataset_path][()]
                    del handle[dataset_path]
                    handle.create_dataset(dataset_path, data=np.atleast_1d(value))
            loaded = read_raw_hdf5(path)

        self.assertEqual(loaded.producer, "matlab")
        self.assertEqual(loaded.case_id, "ss2_normal")
        self.assertEqual(loaded.height_m.shape, (2, 3, 4))
        self.assertTrue(np.array_equal(loaded.height_m, height_m))

    def test_analyzed_hdf5_separates_surface_truth_from_validation_metrics(self):
        x_m = np.asarray([-1.0, 0.0, 1.0])
        y_m = np.asarray([2.0, 3.0])
        time_s = np.asarray([0.0, 0.5])
        height_m = np.zeros((2, 2, 3))
        raw = RawSeaSurface(
            case_id="ss0_flat",
            sea_state=0,
            target_hs_m=0.0,
            raw_hs_m=0.0,
            amplitude_scale_factor=0.0,
            random_seed=101,
            wind_speed_mps=0.0,
            wind_direction_deg=0.0,
            fetch_m=10000.0,
            radar_height_m=1.0,
            mounting_pitch_deg=5.0,
            x_m=x_m,
            y_m=y_m,
            time_s=time_s,
            height_m=height_m,
            producer="matlab",
        )
        truth = analyze_height_cube(
            x_m=x_m,
            y_m=y_m,
            time_s=time_s,
            height_m=height_m,
            radar_height_m=1.0,
            mounting_pitch_deg=5.0,
            target_hs_m=0.0,
            hs_relative_tolerance=0.10,
        )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "truth.h5"
            write_truth_hdf5(raw, truth, path)
            with h5py.File(path, "r") as handle:
                dataset_paths = {
                    name for name in handle["truth"].keys()
                }
                validation_passed = bool(
                    handle["/validation/hs_validation_passed"][()]
                )
                kinematics_paths = set(handle["kinematics"].keys())
                flat_period = float(
                    handle["/kinematics/dominant_wave_period_s"][()]
                )

        self.assertIn("height_m", dataset_paths)
        self.assertIn("normal_x", dataset_paths)
        self.assertIn("vertical_velocity_mps", dataset_paths)
        self.assertIn("slant_range_rate_mps", dataset_paths)
        self.assertIn("grazing_angle_deg", dataset_paths)
        self.assertNotIn("relative_power_db", dataset_paths)
        self.assertIn("dominant_wave_direction_deg", kinematics_paths)
        self.assertIn("dominant_phase_speed_mps", kinematics_paths)
        self.assertTrue(np.isnan(flat_period))
        self.assertTrue(validation_passed)

    def test_matlab_run_analysis_writes_isolated_truth_and_comparison_artifacts(self):
        config_path = Path(
            "simulation/configs/sea_states_0_to_3.json"
        ).resolve()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_run = root / "matlab_input"
            input_data = input_run / "data"
            input_data.mkdir(parents=True)
            x_m = np.asarray([-1.0, 0.0, 1.0])
            y_m = np.asarray([2.0, 3.0])
            time_s = np.asarray([0.0, 0.5, 1.0])
            for case_id, sea_state, target_hs_m, height_m in (
                (
                    "ss0_flat",
                    0,
                    0.0,
                    np.zeros((3, 2, 3)),
                ),
                (
                    "ss2_normal",
                    2,
                    0.30,
                    normalize_height_cube(
                        np.stack(
                            (
                                np.arange(6, dtype=float).reshape(2, 3),
                                -np.arange(6, dtype=float).reshape(2, 3),
                                np.roll(
                                    np.arange(6, dtype=float).reshape(2, 3),
                                    1,
                                    axis=1,
                                ),
                            )
                        ),
                        target_hs_m=0.30,
                    )[0],
                ),
            ):
                write_raw_hdf5(
                    RawSeaSurface(
                        case_id=case_id,
                        sea_state=sea_state,
                        target_hs_m=target_hs_m,
                        raw_hs_m=target_hs_m,
                        amplitude_scale_factor=1.0 if target_hs_m else 0.0,
                        random_seed=101,
                        wind_speed_mps=0.0 if sea_state == 0 else 3.5,
                        wind_direction_deg=90.0,
                        fetch_m=10000.0,
                        radar_height_m=1.0,
                        mounting_pitch_deg=5.0,
                        x_m=x_m,
                        y_m=y_m,
                        time_s=time_s,
                        height_m=height_m,
                        producer="matlab",
                    ),
                    input_data / f"{case_id}_seed101.h5",
                )

            output_run = analyze_matlab_run(
                input_run=input_run,
                config_path=config_path,
                results_root=root / "results",
                run_id="comparison_test",
                render_plots=True,
            )
            output_names = {path.name for path in output_run.iterdir()}
            data_names = {path.name for path in (output_run / "data").iterdir()}
            figure_names = {
                path.name for path in (output_run / "figures").iterdir()
            }
            summaries = json.loads(
                (output_run / "summary.json").read_text(encoding="utf-8")
            )

        self.assertIn("summary.json", output_names)
        self.assertIn("run_config.json", output_names)
        self.assertIn("input_source.json", output_names)
        self.assertIn("ss0_flat_seed101_truth.h5", data_names)
        self.assertIn("ss2_normal_seed101_truth.h5", data_names)
        self.assertIn("ss0_flat_seed101_overview.png", figure_names)
        self.assertIn("ss2_normal_seed101_overview.png", figure_names)
        self.assertIn("sea_state_comparison.png", figure_names)
        self.assertEqual(
            summaries[0]["grazing_angle_temporal_std_mean_deg"], 0.0
        )
        self.assertGreater(
            summaries[1]["grazing_angle_temporal_std_mean_deg"], 0.0
        )
        self.assertIn("grazing_angle_spatiotemporal_std_deg", summaries[0])
        self.assertIsNone(summaries[0]["dominant_wave_period_s"])
        self.assertIn("dominant_wave_period_s", summaries[1])
        self.assertIn("slant_range_rate_p95_mps", summaries[1])
        self.assertEqual(summaries[1]["configured_vessel_wave_direction_deg"], 0.0)
        self.assertIn("dominant_direction_error_deg", summaries[1])

    def test_analysis_rejects_case_id_with_mismatched_state_before_output(self):
        config_path = Path(
            "simulation/configs/sea_states_0_to_3.json"
        ).resolve()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_data = root / "matlab_input" / "data"
            input_data.mkdir(parents=True)
            write_raw_hdf5(
                RawSeaSurface(
                    case_id="ss3_upper",
                    sea_state=2,
                    target_hs_m=0.30,
                    raw_hs_m=0.30,
                    amplitude_scale_factor=1.0,
                    random_seed=101,
                    wind_speed_mps=3.5,
                    wind_direction_deg=90.0,
                    fetch_m=10000.0,
                    radar_height_m=1.0,
                    mounting_pitch_deg=5.0,
                    x_m=np.asarray([-1.0, 1.0]),
                    y_m=np.asarray([2.0, 3.0]),
                    time_s=np.asarray([0.0, 0.5]),
                    height_m=np.zeros((2, 2, 2)),
                    producer="matlab",
                ),
                input_data / "bad_identity.h5",
            )

            with self.assertRaisesRegex(ValueError, "identity mismatch"):
                analyze_matlab_run(
                    input_run=input_data.parent,
                    config_path=config_path,
                    results_root=root / "results",
                    run_id="must_not_exist",
                    render_plots=False,
                )
            self.assertFalse((root / "results").exists())

    def test_analysis_rejects_matlab_wind_direction_mismatch_before_output(self):
        config_path = Path(
            "simulation/configs/sea_states_0_to_3.json"
        ).resolve()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_data = root / "matlab_input" / "data"
            input_data.mkdir(parents=True)
            write_raw_hdf5(
                RawSeaSurface(
                    case_id="ss2_normal",
                    sea_state=2,
                    target_hs_m=0.30,
                    raw_hs_m=0.30,
                    amplitude_scale_factor=1.0,
                    random_seed=101,
                    wind_speed_mps=3.5,
                    wind_direction_deg=0.0,
                    fetch_m=10000.0,
                    radar_height_m=1.0,
                    mounting_pitch_deg=5.0,
                    x_m=np.asarray([-1.0, 1.0]),
                    y_m=np.asarray([2.0, 3.0]),
                    time_s=np.asarray([0.0, 0.5]),
                    height_m=np.zeros((2, 2, 2)),
                    producer="matlab",
                ),
                input_data / "wrong_direction.h5",
            )

            with self.assertRaisesRegex(ValueError, "wind direction mismatch"):
                analyze_matlab_run(
                    input_run=input_data.parent,
                    config_path=config_path,
                    results_root=root / "results",
                    run_id="must_not_exist",
                    render_plots=False,
                )
            self.assertFalse((root / "results").exists())


if __name__ == "__main__":
    unittest.main()
