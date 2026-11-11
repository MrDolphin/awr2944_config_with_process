import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class CaptureReportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.capture = Path(self.temp.name) / 'capture'
        self.capture.mkdir()
        self.output = Path(self.temp.name) / 'report'
        cfg = 'frameCfg 0 0 16 0 656 10 1 0\n'
        metadata = {'active_config': {'name': 'test.cfg', 'content': cfg,
                   'sha256': hashlib.sha256(cfg.encode()).hexdigest()},
                    'registration': {'direction': 1, 'zero_deg': 0},
                    'writer_error': None, 'dropped_frames': 0,
                    'dropped_registration_events': 0}
        (self.capture / 'metadata.json').write_text(json.dumps(metadata), encoding='utf-8')
        frames = [dict(frame_num=i, host_monotonic_s=i * .05, tlv_types=[2])
                  for i in range(5)]
        events = [dict(frame_num=i, host_monotonic_s=i * .05,
                       status='registered', yaw_deg=angle)
                  for i, angle in enumerate([0, 90, 180, 270, 0])]
        for name, rows in [('frames.jsonl', frames),
                           ('live_registration.jsonl', events)]:
            (self.capture / name).write_text(''.join(json.dumps(row) + '\n'
                                                   for row in rows), encoding='utf-8')

    def test_cli_writes_report_to_new_directory_only(self):
        cmd = [sys.executable, str(ROOT / 'scan_revolutions.py'),
               str(self.capture), str(self.output), '--max-step-deg', '100']
        before = {p.name: p.read_bytes() for p in self.capture.iterdir()}
        first = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(first.returncode, 0, first.stderr)
        data = json.loads((self.output / 'report.json').read_text(encoding='utf-8'))
        self.assertEqual(data['raw_missing_frames'], 0)
        self.assertEqual(data['turns'][0]['tlv2_lines'], 4)
        self.assertTrue(data['turns'][0]['complete'])
        self.assertEqual(data['config']['name'], 'test.cfg')
        self.assertIn('report.csv', [p.name for p in self.output.iterdir()])
        self.assertIn('非 RF', (self.output / 'report.md').read_text(encoding='utf-8'))
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.capture.iterdir()})
        second = subprocess.run(cmd, capture_output=True, text=True)
        self.assertNotEqual(second.returncode, 0)
        self.assertEqual(data, json.loads((self.output / 'report.json').read_text(encoding='utf-8')))

    def test_bad_config_hash_refuses_output(self):
        meta_path = self.capture / 'metadata.json'
        meta = json.loads(meta_path.read_text(encoding='utf-8'))
        meta['active_config']['sha256'] = '0' * 64
        meta_path.write_text(json.dumps(meta), encoding='utf-8')
        cmd = [sys.executable, str(ROOT / 'scan_revolutions.py'),
               str(self.capture), str(self.output)]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.output.exists())

    def test_output_inside_capture_is_refused(self):
        nested = self.capture / 'report'
        cmd = [sys.executable, str(ROOT / 'scan_revolutions.py'),
               str(self.capture), str(nested)]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(nested.exists())


if __name__ == '__main__':
    unittest.main()
