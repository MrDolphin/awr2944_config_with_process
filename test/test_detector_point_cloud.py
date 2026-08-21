import csv, tempfile, unittest
from pathlib import Path
import h5py, numpy as np
from simulation.run_v04_72_detector_point_cloud import run, _geometry_for_input


class DetectorPointCloudTest(unittest.TestCase):
    def test_projection_contract(self):
        root = Path(__file__).resolve().parents[1]
        source = root / "simulation/hardware/awr2944pev/v04_43_sea_range_doppler/ss0_flat_range_doppler.h5"
        geometry = root / "simulation/hardware/awr2944pev/antgeometry_mapping.csv"
        with tempfile.TemporaryDirectory() as temp:
            temp = Path(temp); input_root = temp / "input"; input_root.mkdir()
            with h5py.File(source, "r") as src, h5py.File(input_root / source.name, "w") as dst:
                for key in ("axes", "range_doppler"):
                    src.copy(key, dst)
            result = run(input_root, geometry, temp / "output", max_points_per_group=3)
            self.assertEqual(result["status"], "completed_detector_point_cloud")
            self.assertTrue((temp / "output/detector_point_cloud_summary.csv").exists())
            self.assertTrue((temp / "output/output_analysis.md").exists())
            with (temp / "output/detector_point_cloud_summary.csv").open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 4 * 9)
            self.assertTrue(all(int(row["stored_point_count"]) <= 3 for row in rows))

    def test_cad_geometry_contract(self):
        root = Path(__file__).resolve().parents[1]
        cad = root / "simulation/hardware/awr2944pev/v04_73_new_pcb_package/cad_virtual_array_coordinates.csv"
        x, y = _geometry_for_input(cad, 299792458.0 / 77e9)
        self.assertEqual(x.shape, (4, 4))
        self.assertGreater(float(np.ptp(x)), 0.0)
        self.assertGreater(float(np.ptp(y)), 0.0)


if __name__ == "__main__": unittest.main()
