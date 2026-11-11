import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from scan_revolutions import write_capture_report


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
        self.assertTrue((self.output / 'COMPLETE').is_file())
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

    def test_render_failure_does_not_publish_partial_report(self):
        original_write = Path.write_text

        def fail_markdown(path, *args, **kwargs):
            if path.name == 'report.md':
                raise OSError('simulated report write failure')
            return original_write(path, *args, **kwargs)

        with patch.object(Path, 'write_text', fail_markdown):
            with self.assertRaisesRegex(OSError, 'simulated report'):
                write_capture_report(self.capture, self.output, max_step_deg=100)
        self.assertFalse(self.output.exists())

    def test_publication_failure_removes_only_owned_output(self):
        original_link = os.link
        calls = 0

        def fail_second_link(source, target):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise OSError('simulated publication failure')
            return original_link(source, target)

        with patch('scan_revolutions.os.link', fail_second_link):
            with self.assertRaisesRegex(OSError, 'simulated publication'):
                write_capture_report(self.capture, self.output, max_step_deg=100)
        self.assertFalse(self.output.exists())

    def test_incomplete_completion_marker_is_removed(self):
        original_write = Path.write_text

        def fail_after_marker_write(path, *args, **kwargs):
            written = original_write(path, *args, **kwargs)
            if path.name == 'COMPLETE':
                raise OSError('simulated marker failure')
            return written

        with patch.object(Path, 'write_text', fail_after_marker_write):
            with self.assertRaisesRegex(OSError, 'simulated marker'):
                write_capture_report(self.capture, self.output, max_step_deg=100)
        self.assertFalse(self.output.exists())


if __name__ == '__main__':
    unittest.main()
