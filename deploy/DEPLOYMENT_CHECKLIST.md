# Radar-camera deployment checklist

Use the verified checkout at `/home/pi/camera_web_fusion`. Record these before and after every acceptance gate:

```bash
git rev-parse HEAD
git status --short
vcgencmd measure_temp
free -h
df -h
readlink -f /dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0
v4l2-ctl --device /dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0 --list-formats-ext
ss -ltnp | grep -E ':(8765|8081)\b'
pgrep -af 'ffmpeg|camera.py|radar_server.py'
```

Install from the checkout with `sudo env APP_DIR=/home/pi/camera_web_fusion bash ./deploy/setup_rpi.sh`. The installer enables but does not start `radar.service`; first verify the radar-only WebSocket path, serial ownership, and absence of FFmpeg.

`/etc/default/radar-camera` defaults to `RADAR_CAMERA_ARGS=`. Enable the camera only after the camera-only gate by setting a complete explicit argument string, including `--enable-camera`, `--camera-config`, `--camera-http-port`, and a browser-reachable `--camera-public-base-url`.

The service uses systemd's standalone `$RADAR_CAMERA_ARGS` expansion so each option reaches Python as a separate argument. Keep this variable unbraced in `ExecStart`; `${RADAR_CAMERA_ARGS}` would pass the whole option string as one argument.

For QD4310 USB control, identify its `/dev/serial/by-id/` symlink with `ls -l /dev/serial/by-id/` and add `--qd4310-port <that-path>` to the same `RADAR_CAMERA_ARGS` value. Leave it absent until supervised bench testing. The web panel uses `enable`, `ctrl speed <rpm>`, `ctrl speed 0`, and `disable`; verify the stop behavior with the installed firmware and an independent power cut available. USB is the control link; the motor needs its own XT30 power supply.

## Rollback

Rollback is one environment edit: removing --enable-camera from `RADAR_CAMERA_ARGS`, then run `sudo systemctl daemon-reload` and `sudo systemctl restart radar.service`. Preserve session data and logs; do not delete them during rollback.
