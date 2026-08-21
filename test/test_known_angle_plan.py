import unittest

from simulation.run_v04_93_known_angle_plan import build_plan


class KnownAnglePlanTest(unittest.TestCase):
    def test_plan_covers_multiple_axes_and_ranges(self):
        rows = build_plan()
        self.assertEqual(len(rows), 30)
        self.assertEqual(sorted({row["azimuth_deg"] for row in rows}), [-30, -15, 0, 15, 30])
        self.assertEqual(sorted({row["elevation_deg"] for row in rows}), [-10, 0, 10])
        self.assertEqual(sorted({row["range_m"] for row in rows}), [10, 20])
        self.assertEqual(len({row["case_id"] for row in rows}), 30)


if __name__ == "__main__":
    unittest.main()
