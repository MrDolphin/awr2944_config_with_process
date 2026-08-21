import csv
import tempfile
import unittest
from pathlib import Path
import h5py
import numpy as np

from simulation.run_v04_78_mapping_sea_clutter import run


class MappingSeaClutterTest(unittest.TestCase):
    def test_one_point_expands_to_eight_mappings(self):
        root = Path(__file__).resolve().parents[1]
        mapping = root / "simulation/hardware/awr2944pev/antgeometry_mapping.csv"
        with tempfile.TemporaryDirectory() as temp_name:
            temp = Path(temp_name); input_root = temp / "input"; input_root.mkdir()
            with h5py.File(input_root / "case_range_doppler.h5", "w") as handle:
                handle.create_dataset("/range_doppler/spectrum_complex", data=np.ones((1,1,1,4,4), dtype=complex))
            points = temp / "points.csv"
            with points.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=["case_id","scenario_id","detector","frame","doppler_index","range_index"])
                writer.writeheader(); writer.writerow({"case_id":"case","scenario_id":"s","detector":"d","frame":0,"doppler_index":0,"range_index":0})
            result = run(points, input_root, mapping, temp / "output")
            self.assertEqual(result["input_point_count"], 1)
            self.assertEqual(result["group_count"], 8)


if __name__ == "__main__":
    unittest.main()
