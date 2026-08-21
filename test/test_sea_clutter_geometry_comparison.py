import unittest


class SeaClutterGeometryComparisonTest(unittest.TestCase):
    def test_contract_names_keep_provisional_geometry_boundary(self):
        from simulation.run_v04_108_sea_clutter_geometry_comparison import run
        self.assertTrue(callable(run))


if __name__ == "__main__":
    unittest.main()
