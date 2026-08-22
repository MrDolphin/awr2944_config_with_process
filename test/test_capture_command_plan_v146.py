import json
import tempfile
import unittest
from pathlib import Path


class CaptureCommandPlanV146Test(unittest.TestCase):
    def test_plan_is_non_executing_and_contains_review_gates(self):
        from simulation.run_v04_146_capture_command_plan import run

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); cfg = root / "candidate.cfg"; dca = root / "cf.json"
            cfg.write_text("sensorStart\n", encoding="utf-8"); dca.write_text("{}", encoding="utf-8")
            result = run(cfg, dca, root / "out")
            self.assertFalse(result["execution_performed"])
            self.assertFalse(result["hardware_commands_executed"])
            self.assertTrue(any(item["requires_operator_review"] for item in result["commands"]))
            self.assertTrue((root / "out" / "command_plan.json").exists())


if __name__ == "__main__":
    unittest.main()
