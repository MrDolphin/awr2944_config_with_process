from pathlib import Path
import tempfile
import unittest

from simulation.artifacts import create_run_directory
from simulation.run_v01 import prepare_v01_run
from simulation.v01 import load_config


class SimulationArtifactTests(unittest.TestCase):
    def test_run_directory_is_isolated_and_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            results_root = Path(directory)
            run_directory = create_run_directory(
                results_root,
                producer="python",
                stage_id="v01_flat_sea_geometry",
                run_id="test_baseline",
            )

            self.assertEqual(
                run_directory,
                results_root / "python" / "test_baseline",
            )
            self.assertTrue((run_directory / "data").is_dir())
            self.assertTrue((run_directory / "figures").is_dir())
            self.assertTrue((run_directory / "design_snapshot.md").is_file())
            self.assertTrue((run_directory / "environment.json").is_file())
            self.assertTrue((run_directory / "validation.md").is_file())

            with self.assertRaises(FileExistsError):
                create_run_directory(
                    results_root,
                    producer="python",
                    stage_id="v01_flat_sea_geometry",
                    run_id="test_baseline",
                )

    def test_v01_run_is_nested_by_producer_and_run_id(self):
        config = load_config(
            Path("simulation/configs/baseline_1m.json").resolve()
        )
        with tempfile.TemporaryDirectory() as directory:
            prepared = prepare_v01_run(
                config,
                results_root=Path(directory),
                run_id="comparison_case",
            )

            self.assertEqual(
                prepared.output_directory,
                Path(directory).resolve() / "python" / "comparison_case",
            )


if __name__ == "__main__":
    unittest.main()
