import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class DeployTemplateTests(unittest.TestCase):
    def test_service_uses_dialout_and_no_stale_pythonpath(self):
        service = (ROOT / "deploy" / "radar.service").read_text(encoding="utf-8")
        self.assertIn("SupplementaryGroups=dialout", service)
        self.assertIn("After=network-online.target", service)
        self.assertNotIn("python3.13", service)
        self.assertNotIn("BindsTo=dev-ttyACM0.device", service)

    def test_setup_requires_explicit_app_and_does_not_change_network(self):
        script = (ROOT / "deploy" / "setup_rpi.sh").read_text(encoding="utf-8")
        self.assertIn("APP_DIR", script)
        self.assertIn("python3-serial python3-websockets", script)
        self.assertIn("sudo env APP_DIR=/home/pi/camera_web_fusion ./deploy/setup_rpi.sh", script)
        self.assertNotIn("nmcli connection modify", script)

    def test_camera_service_is_opt_in_and_has_safe_shutdown_contract(self):
        service = (ROOT / "deploy" / "radar.service").read_text(encoding="utf-8")
        env_example = (ROOT / "deploy" / "radar-camera.env.example").read_text(encoding="utf-8")
        checklist = (ROOT / "deploy" / "DEPLOYMENT_CHECKLIST.md").read_text(encoding="utf-8")

        self.assertIn("WorkingDirectory=/home/pi/camera_web_fusion", service)
        self.assertIn("EnvironmentFile=-/etc/default/radar-camera", service)
        # systemd splits a standalone $NAME into argv entries; ${NAME} stays one argv entry.
        self.assertIn("ExecStart=/usr/bin/python3 radar_server.py --ws_port 8765 $RADAR_CAMERA_ARGS", service)
        self.assertNotIn("${RADAR_CAMERA_ARGS}", service)
        self.assertIn("TimeoutStopSec=15", service)
        self.assertNotIn("--enable-camera", service)
        self.assertIn("RADAR_CAMERA_ARGS=", env_example)
        self.assertIn("RADAR_CAMERA_CONFIG=/home/pi/camera_web_fusion/tools/camera/camera_config.cfg", env_example)
        self.assertIn("RADAR_CAMERA_HTTP_PORT=8081", env_example)
        for command in ("readlink -f /dev/v4l/by-id", "v4l2-ctl --device", "ss -ltnp", "pgrep -af", "vcgencmd measure_temp", "git rev-parse HEAD"):
            self.assertIn(command, checklist)
        self.assertIn("removing --enable-camera", checklist)


if __name__ == "__main__":
    unittest.main()
