import unittest
from pathlib import Path


class BeamMaskedPointCloudTest(unittest.TestCase):
    def test_run_contract_and_summary_fields(self):
        from simulation.run_v04_119_beam_masked_point_cloud import run
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("simulation/stages/v02_dynamic_sea_truth/results/matlab/v02b_quick_seed101_hs1m/data"), Path(directory) / "out", frame_index=20)
            self.assertEqual(result["status"], "completed_beam_masked_3d_point_cloud")
            self.assertEqual(len(result["selected_cases"]), 5)


if __name__ == "__main__":
    unittest.main()
