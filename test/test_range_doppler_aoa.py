import tempfile
import unittest
from pathlib import Path

import h5py
import numpy as np

from simulation.run_v04_range_doppler_aoa import run


class RangeDopplerAoATests(unittest.TestCase):
    def test_processes_virtual_h5_and_writes_peak_cell(self):
        geometry = Path(__file__).parents[1] / "simulation" / "hardware" / "awr2944pev" / "antgeometry_mapping.csv"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_h5 = root / "input.h5"
            output_h5 = root / "output.h5"
            with h5py.File(input_h5, "w") as handle:
                handle.create_dataset("/recovered/virtual_iq", data=np.ones((8, 16, 4, 4), dtype=complex))
            summary = run(input_h5, geometry, output_h5, use_calibrated=False)
            self.assertEqual(summary["virtual_shape"], [8, 16, 4, 4])
            self.assertIn("peak_range_m", summary)
            with h5py.File(output_h5, "r") as handle:
                self.assertEqual(handle["/range_doppler/power_linear"].shape, (8, 8))
                self.assertEqual(handle["/peak/channel_complex"].shape, (4, 4))


if __name__ == "__main__":
    unittest.main()
