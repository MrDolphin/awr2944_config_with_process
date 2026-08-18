import math
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import h5py
import numpy as np

from simulation.v01 import (
    SimulationSettings,
    V01Config,
    load_config,
    plot_hdf5,
    read_hdf5,
    run_sweep,
    simulate_flat_sea,
    write_hdf5,
)


class FlatSeaSimulationTests(unittest.TestCase):
    def test_invalid_grid_is_rejected_before_simulation(self):
        with self.assertRaisesRegex(ValueError, "range_step_m"):
            SimulationSettings(range_step_m=0.0)

    def test_baseline_config_resolves_radar_cfg_and_pitch_sweep(self):
        config = load_config(
            Path("simulation/configs/baseline_1m.json").resolve()
        )

        self.assertTrue(config.radar_cfg_path.is_file())
        self.assertEqual(config.mounting_pitch_sweep_deg, (0.0, 3.0, 5.0, 8.0, 10.0))
        self.assertAlmostEqual(config.settings.height_m, 1.0)
        self.assertEqual(config.radar_metadata["num_adc_samples"], 656)
        self.assertEqual(config.radar_metadata["frame_num_adc_samples"], 656)
        self.assertAlmostEqual(config.radar_metadata["frame_period_ms"], 100.0)
        self.assertAlmostEqual(config.radar_metadata["start_freq_ghz"], 77.0)
        self.assertEqual(config.radar_metadata["chirp_tx_masks"], {0: 1, 1: 4, 2: 8, 3: 2})

    def test_boresight_intersects_flat_sea_at_analytic_distance(self):
        settings = SimulationSettings(
            height_m=1.0,
            range_min_m=2.0,
            range_max_m=40.0,
            range_step_m=0.01,
            azimuth_min_deg=0.0,
            azimuth_max_deg=0.0,
            azimuth_step_deg=1.0,
        )

        result = simulate_flat_sea(settings, mounting_pitch_deg=5.0)

        center_index = int(abs(result.elevation_deg[:, 0]).argmin())
        expected_range_m = 1.0 / math.tan(math.radians(5.0))
        self.assertAlmostEqual(
            result.horizontal_range_m[center_index, 0], expected_range_m, delta=0.011
        )
        self.assertAlmostEqual(result.elevation_deg[center_index, 0], 0.0, delta=0.03)

    def test_direct_simulation_preserves_cfg_metadata_for_hdf5(self):
        baseline = load_config(
            Path("simulation/configs/baseline_1m.json").resolve()
        )

        result = simulate_flat_sea(
            baseline.settings,
            mounting_pitch_deg=5.0,
            radar_metadata=baseline.radar_metadata,
        )

        self.assertEqual(result.radar_metadata["frame_num_adc_samples"], 656)
        self.assertAlmostEqual(result.radar_metadata["frame_period_ms"], 100.0)

    def test_three_db_elevation_edges_define_expected_sea_footprint(self):
        settings = SimulationSettings(
            height_m=1.0,
            range_min_m=5.0,
            range_max_m=30.0,
            range_step_m=0.005,
            azimuth_min_deg=0.0,
            azimuth_max_deg=0.0,
            azimuth_step_deg=1.0,
        )

        result = simulate_flat_sea(settings, mounting_pitch_deg=5.0)

        for elevation_deg, expected_range_m in (
            (-3.0, 1.0 / math.tan(math.radians(8.0))),
            (3.0, 1.0 / math.tan(math.radians(2.0))),
        ):
            index = int(abs(result.elevation_deg[:, 0] - elevation_deg).argmin())
            self.assertAlmostEqual(
                result.horizontal_range_m[index, 0], expected_range_m, delta=0.006
            )
            self.assertAlmostEqual(
                result.one_way_gain_db[index, 0], -3.0, delta=0.01
            )

    def test_symmetric_flat_sea_produces_symmetric_relative_power(self):
        settings = SimulationSettings(
            range_min_m=5.0,
            range_max_m=20.0,
            range_step_m=1.0,
            azimuth_min_deg=-30.0,
            azimuth_max_deg=30.0,
            azimuth_step_deg=5.0,
        )

        result = simulate_flat_sea(settings, mounting_pitch_deg=5.0)

        self.assertTrue(
            np.allclose(
                result.relative_power_linear,
                result.relative_power_linear[:, ::-1],
                rtol=1e-12,
                atol=0.0,
            )
        )
        self.assertAlmostEqual(float(result.relative_power_db.max()), 0.0)

    def test_hdf5_round_trip_preserves_truth_and_processed_data(self):
        settings = SimulationSettings(
            range_min_m=5.0,
            range_max_m=6.0,
            range_step_m=0.5,
            azimuth_min_deg=-5.0,
            azimuth_max_deg=5.0,
            azimuth_step_deg=5.0,
        )
        result = simulate_flat_sea(settings, mounting_pitch_deg=3.0)

        with tempfile.TemporaryDirectory() as directory:
            output_path = Path(directory) / "case.h5"
            write_hdf5(result, output_path)
            loaded = read_hdf5(output_path)

        self.assertEqual(loaded.schema_version, "awr2944p-flat-sea-v0.1")
        self.assertEqual(loaded.power_model, "unit_sigma0_pattern_r4")
        self.assertAlmostEqual(loaded.height_m, 1.0)
        self.assertAlmostEqual(loaded.mounting_pitch_deg, 3.0)
        self.assertTrue(np.array_equal(loaded.x_m, result.x_m))
        self.assertTrue(
            np.array_equal(loaded.relative_power_db, result.relative_power_db)
        )

    def test_hdf5_reader_normalizes_matlab_matrix_layout(self):
        settings = SimulationSettings(
            range_min_m=5.0,
            range_max_m=6.0,
            range_step_m=0.5,
            azimuth_min_deg=-5.0,
            azimuth_max_deg=5.0,
            azimuth_step_deg=5.0,
        )
        result = simulate_flat_sea(settings, mounting_pitch_deg=3.0)

        with tempfile.TemporaryDirectory() as directory:
            output_path = Path(directory) / "matlab_case.h5"
            write_hdf5(result, output_path)
            with h5py.File(output_path, "r+") as handle:
                handle.attrs["producer"] = np.bytes_("matlab")
                for dataset_path in (
                    "/installation/height_m",
                    "/installation/mounting_pitch_deg",
                ):
                    scalar_value = handle[dataset_path][()]
                    del handle[dataset_path]
                    handle.create_dataset(
                        dataset_path,
                        data=np.atleast_1d(scalar_value),
                    )
                for group_name in ("truth", "antenna", "processed"):
                    group = handle[group_name]
                    for dataset_name in tuple(group.keys()):
                        transposed = group[dataset_name][...].T
                        del group[dataset_name]
                        group.create_dataset(dataset_name, data=transposed)
            loaded = read_hdf5(output_path)

        self.assertEqual(loaded.x_m.shape, result.x_m.shape)
        self.assertTrue(np.array_equal(loaded.x_m, result.x_m))
        self.assertAlmostEqual(loaded.height_m, 1.0)
        self.assertAlmostEqual(loaded.mounting_pitch_deg, 3.0)

    def test_hdf5_plot_is_saved_in_the_run_figures_directory(self):
        settings = SimulationSettings(
            range_min_m=5.0,
            range_max_m=6.0,
            range_step_m=1.0,
            azimuth_min_deg=0.0,
            azimuth_max_deg=0.0,
            azimuth_step_deg=1.0,
        )
        result = simulate_flat_sea(settings, mounting_pitch_deg=5.0)

        with tempfile.TemporaryDirectory() as directory:
            run_directory = Path(directory)
            data_directory = run_directory / "data"
            data_directory.mkdir()
            input_path = data_directory / "pitch_05p0_deg.h5"
            write_hdf5(result, input_path)

            output_path = plot_hdf5(input_path)

            self.assertEqual(
                output_path,
                run_directory / "figures" / "pitch_05p0_deg.png",
            )
            self.assertTrue(output_path.is_file())

    def test_run_sweep_writes_cases_summary_and_input_snapshots(self):
        baseline = load_config(
            Path("simulation/configs/baseline_1m.json").resolve()
        )
        small_settings = replace(
            baseline.settings,
            range_min_m=5.0,
            range_max_m=10.0,
            range_step_m=1.0,
            azimuth_min_deg=-5.0,
            azimuth_max_deg=5.0,
            azimuth_step_deg=5.0,
        )

        with tempfile.TemporaryDirectory() as directory:
            config = V01Config(
                settings=small_settings,
                radar_cfg_path=baseline.radar_cfg_path,
                mounting_pitch_sweep_deg=(0.0, 5.0),
                output_directory=Path(directory),
                plot_floor_db=-60.0,
                radar_metadata=baseline.radar_metadata,
            )
            summaries = run_sweep(config, render_plots=False)
            output_names = {path.name for path in Path(directory).iterdir()}
            data_names = {
                path.name for path in (Path(directory) / "data").iterdir()
            }
            loaded_case = read_hdf5(
                Path(directory) / "data" / "pitch_05p0_deg.h5"
            )

        self.assertEqual(len(summaries), 2)
        self.assertIn("pitch_00p0_deg.h5", data_names)
        self.assertIn("pitch_05p0_deg.h5", data_names)
        self.assertIn("summary.json", output_names)
        self.assertIn("radar_profile.cfg", output_names)
        self.assertAlmostEqual(summaries[1]["boresight_intersection_m"], 11.4301, places=3)
        self.assertGreater(summaries[1]["total_relative_power_linear"], 0.0)
        self.assertGreaterEqual(summaries[1]["mainlobe_power_fraction"], 0.0)
        self.assertLessEqual(summaries[1]["mainlobe_power_fraction"], 1.0)
        self.assertAlmostEqual(summaries[1]["six_db_near_m"], 5.67128, places=4)
        self.assertIsNone(summaries[1]["six_db_far_m"])
        self.assertEqual(loaded_case.radar_metadata["num_adc_samples"], 656)

    def test_run_sweep_renders_case_and_coverage_figures(self):
        baseline = load_config(
            Path("simulation/configs/baseline_1m.json").resolve()
        )
        with tempfile.TemporaryDirectory() as directory:
            config = V01Config(
                settings=replace(
                    baseline.settings,
                    range_min_m=5.0,
                    range_max_m=15.0,
                    range_step_m=1.0,
                    azimuth_min_deg=-10.0,
                    azimuth_max_deg=10.0,
                    azimuth_step_deg=5.0,
                ),
                radar_cfg_path=baseline.radar_cfg_path,
                mounting_pitch_sweep_deg=(5.0,),
                output_directory=Path(directory),
                plot_floor_db=-60.0,
                radar_metadata=baseline.radar_metadata,
            )
            run_sweep(config, render_plots=True)
            figure_names = {
                path.name for path in (Path(directory) / "figures").iterdir()
            }

        self.assertIn("pitch_05p0_deg.png", figure_names)
        self.assertIn("pitch_coverage_summary.png", figure_names)


if __name__ == "__main__":
    unittest.main()
