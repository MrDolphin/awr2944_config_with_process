import unittest
from pathlib import Path


class MaskedMicrofacetClutterTest(unittest.TestCase):
    def test_run_produces_masked_proxy_rows(self):
        from simulation.run_v04_120_masked_microfacet_clutter import run
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            result = run(Path("simulation/stages/v02_dynamic_sea_truth/results/matlab/v02b_quick_seed101_hs1m/data"), Path(directory) / "out")
            self.assertEqual(result["case_count"], 5)
            self.assertGreater(result["row_count"], 0)
            self.assertIn("scatter_proxy", (Path(directory) / "out" / "masked_microfacet_clutter.csv").read_text(encoding="utf-8").splitlines()[0])


if __name__ == "__main__":
    unittest.main()
