import unittest

from simulation.run_v04_57_report_figures import read_csv


class ReportFiguresTests(unittest.TestCase):
    def test_read_csv_preserves_project_result_rows(self):
        rows = read_csv(__import__('pathlib').Path('simulation/hardware/awr2944pev/v04_48_physical_target_model/physical_target_sweep.csv'))
        self.assertGreater(len(rows), 0)
        self.assertIn('detection_probability', rows[0])


if __name__ == '__main__':
    unittest.main()
