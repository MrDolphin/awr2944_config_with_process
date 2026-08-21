import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_106_provisional_array_geometry import build


class ProvisionalArrayGeometryTest(unittest.TestCase):
    def test_builds_eight_marked_provisional_elements(self):
        candidates = {name: [{"x_mil": 100.0 + i, "y_mil": 200.0 + i}] for i, name in enumerate(["TX1", "TX2", "TX3", "TX4", "RX1", "RX2", "RX3", "RX4"])}
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "summary.json"
            out = root / "out"
            source.write_text(json.dumps({"candidates_by_net": candidates}), encoding="utf-8")
            result = build(source, out, 100.0, 200.0)
            self.assertEqual(result["element_count"], 8)
            self.assertFalse(result["phase_center_validated"])
            self.assertIn("provisional", (out / "provisional_array_geometry.yaml").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
