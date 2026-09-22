from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from tools.camera.camera_config import CameraConfig, load_camera_config
from tools.camera.camera_probe import parse_mjpeg_modes


class CameraConfigTests(unittest.TestCase):
    def test_loads_stable_device_and_supported_mode(self):
        """Changing a valid capture mode must not turn numeric fields into strings."""
        with TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "camera.cfg"
            path.write_text(
                "device=/dev/v4l/by-id/camera-video-index0\n"
                "width=1280\nheight=720\nfps=30\ninput_format=mjpeg\n",
                encoding="utf-8",
            )
            config = load_camera_config(path)

        self.assertEqual(config.device, "/dev/v4l/by-id/camera-video-index0")
        self.assertEqual(config.width, 1280)
        self.assertEqual(config.height, 720)
        self.assertEqual(config.fps, 30)
        self.assertEqual(config.input_format, "mjpeg")

    def test_rejects_non_positive_frame_rate(self):
        """Removing rate validation would permit an unusable FFmpeg command."""
        with self.assertRaisesRegex(ValueError, "fps"):
            CameraConfig(device="/dev/video0", width=1280, height=720, fps=0)

    def test_rejects_unknown_key(self):
        """A misspelled acquisition setting must fail rather than be ignored."""
        with TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "camera.cfg"
            path.write_text(
                "device=/dev/video0\nwidth=1280\nheight=720\nfps=30\ncodec=mjpeg\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "unknown"):
                load_camera_config(path)


class CameraProbeParserTests(unittest.TestCase):
    def test_parses_only_mjpeg_discrete_modes(self):
        """Treating a YUYV mode as MJPEG would make capability validation lie."""
        output = """\
        [0]: 'MJPG' (Motion-JPEG, compressed)
            Size: Discrete 1280x720
                Interval: Discrete 0.033s (30.000 fps)
            Size: Discrete 640x480
                Interval: Discrete 0.067s (15.000 fps)
        [1]: 'YUYV' (YUYV 4:2:2)
            Size: Discrete 1280x720
                Interval: Discrete 0.033s (30.000 fps)
        """

        self.assertEqual(
            parse_mjpeg_modes(output),
            {(1280, 720, 30), (640, 480, 15)},
        )


if __name__ == "__main__":
    unittest.main()
