import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_107_provisional_array_comparison import load_provisional
from simulation.v03 import FmcwConfig


class ProvisionalArrayComparisonTest(unittest.TestCase):
    def test_loads_four_by_four_virtual_positions(self):
        text = "channel,kind,x_board_mil,y_board_mil,x_relative_mm,y_relative_mm,source_rank,confidence,phase_center_validated\n"
        for i, name in enumerate(["TX1", "TX2", "TX3", "TX4", "RX1", "RX2", "RX3", "RX4"]):
            kind = "TX" if name.startswith("TX") else "RX"
            text += f"{name},{kind},0,0,{i},0,1,provisional_geometric_candidate,False\n"
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "geometry.csv"
            path.write_text(text, encoding="utf-8")
            x, y = load_provisional(path, FmcwConfig())
            self.assertEqual(x.shape, (4, 4))
            self.assertEqual(y.shape, (4, 4))
            self.assertAlmostEqual(x[0, 0], 0.004)


if __name__ == "__main__":
    unittest.main()
