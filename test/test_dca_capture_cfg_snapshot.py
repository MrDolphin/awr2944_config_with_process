import hashlib
import tempfile
import unittest
from pathlib import Path

from tools.dca1000_capture import snapshot_radar_cfg


class DcaCaptureCfgSnapshotTests(unittest.TestCase):
    def test_capture_directory_contains_the_exact_cfg_and_sha256(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "test_full.cfg"
            session = root / "20260911_113827"
            session.mkdir()
            content = "channelCfg 15 15 0 0 0\nlvdsStreamCfg -1 0 1 0\n"
            source.write_text(content, encoding="utf-8")

            snapshot = snapshot_radar_cfg(str(source), session)

            saved = session / "capture_config.cfg"
            self.assertEqual(saved.read_bytes(), source.read_bytes())
            self.assertEqual(snapshot["snapshot_file"], "capture_config.cfg")
            self.assertEqual(snapshot["source_path"], str(source))
            self.assertEqual(snapshot["sha256"], hashlib.sha256(source.read_bytes()).hexdigest())


if __name__ == "__main__":
    unittest.main()
