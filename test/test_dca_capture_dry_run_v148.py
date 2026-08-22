import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


class DcaCaptureDryRunV148Test(unittest.TestCase):
    def test_dry_run_does_not_create_capture_or_open_hardware(self):
        root = Path(__file__).parents[1]
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            cfg = work / "candidate.cfg"
            cf = work / "cf.json"
            cfg.write_text("channelCfg 15 15 0 0 0\nprofileCfg 0 77 1 2 3 4 5 6 7 8 16 9 10 11\nlvdsStreamCfg -1 0 1 0\nsensorStart\n", encoding="utf-8")
            cf.write_text(json.dumps({"DCA1000Config": {"ethernetConfig": {"DCA1000IPAddress": "192.0.2.10", "DCA1000ConfigPort": 4096, "DCA1000DataPort": 4098}}}), encoding="utf-8")
            completed = subprocess.run([sys.executable, str(root / "tools/dca1000_capture.py"), "--cfg", str(cfg), "--cf-json", str(cf), "--duration", "3", "--dry-run"], capture_output=True, text=True, check=True)
            plan = json.loads(completed.stdout)
            self.assertEqual(plan["status"], "dry_run_only")
            self.assertFalse(plan["hardware_commands_executed"])
            self.assertFalse(plan["udp_socket_opened"])
            self.assertFalse(list(work.glob("adc_data*")))


if __name__ == "__main__":
    unittest.main()
