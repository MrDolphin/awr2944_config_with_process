# Radar-Camera Web Fusion Execution Environment Baseline

**Captured at:** 2026-09-22 16:17–16:23 CST  
**Purpose:** 固化 `Radar-Camera Web Fusion Implementation Plan` 开始执行前的 PC、树莓派、相机、雷达接口、网络和工具链事实。  
**Collection method:** PC 本地只读 PowerShell/Git 命令，以及通过 `ssh pi@172.20.10.10` 执行的只读 Linux 命令。采集过程没有打开相机视频流、发送雷达配置、启动采集或驱动云台。

## 1. Authoritative Development Locations

| Role | Required value | Snapshot status |
|---|---|---|
| PC worktree | `D:\hp-laptop\USV\awr2944_radar_camera_web_fusion` | Exists |
| Git branch | `codex/radar-camera-web-fusion` | Checked out |
| Pi development/deployment checkout | `/home/pi/camera_web_fusion` | **Missing; must be created during first deployment** |
| Pi user | `pi` | SSH read-only inspection succeeded |
| Current lab SSH endpoint | `pi@172.20.10.10` | Reachable at capture time |
| Sea-clutter worktree | `D:\hp-laptop\USV\awr2944_sea_clutter_v02` | Out of scope for this feature |

The SSH address is a network-mode-dependent observation, not a source-code constant. The target Pi directory is the only development/deployment checkout for this feature after initial setup; `/home/pi/radar_server.py` and `/home/pi/awr2944_config_with_process_github` are legacy/current-runtime references, not the new development location.

## 2. PC Hardware and Operating System

| Item | Observed value |
|---|---|
| Manufacturer/model | MECHREVO KUANGSHI Series |
| OS | Microsoft Windows 11 Home Chinese, 64-bit |
| OS version/build | `10.0.26200`, build `26200` |
| CPU | Intel Core i9-14900HX |
| CPU topology | 24 physical cores, 32 logical processors |
| Installed memory | 34,056,667,136 bytes, approximately 31.7 GiB |
| Integrated GPU | Intel UHD Graphics, driver `32.0.101.6733` |
| Discrete GPU | NVIDIA GeForce RTX 5070 Laptop GPU, driver `32.0.15.7322` |
| D: free space | 154,517,516,288 bytes, approximately 143.9 GiB |

The WMI-reported GPU memory value was not retained because the Windows provider can truncate adapter memory. GPU acceleration is not required by the first implementation stages.

## 3. PC Toolchain

| Tool | Observed value | Plan implication |
|---|---|---|
| Git | `2.54.0.windows.1` | Sufficient |
| Python | CPython `3.12.10`, 64-bit | Development/test interpreter |
| Python executable | `C:\Program Files\Python312\python.exe` | Use explicit `python` from this environment |
| pip | `26.1.2` | Global/user pip |
| Python launcher | `py` command missing | Do not use `py -m ...` in plan commands |
| PC `websockets` package | Missing from active Python | Must be installed in project virtual environment before integration tests needing it |
| PC `pyserial` package | Missing from active Python | Must be installed in project virtual environment before serial integration tests |
| PowerShell | `7.6.5` Core | Primary PC shell |
| FFmpeg | `8.1.2-full_build-www.gyan.dev` | Available for offline media checks |
| OpenSSH client | `OpenSSH_for_Windows_9.5p2`, LibreSSL `3.8.2` | Pi deployment/inspection |
| Chrome | `153.0.8010.53` | Primary browser acceptance option |
| Edge | `153.0.4234.32` | Secondary browser acceptance option |

### PC environment bootstrap requirement

Do not install project packages globally. At the beginning of implementation, create a worktree-local environment:

```powershell
Set-Location D:\hp-laptop\USV\awr2944_radar_camera_web_fusion
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install websockets pyserial
```

Record exact versions after installation:

```powershell
python --version
python -m pip freeze | Sort-Object
```

OpenCV is not required for Tasks 0–6. Install it only in the PC virtual environment when starting Task 7 offline calibration, and pin the selected version in a dedicated requirements file.

## 4. PC Git Baseline and Divergence

| Item | Observed value |
|---|---|
| Current branch | `codex/radar-camera-web-fusion` |
| Current HEAD | `840a4feb5211b96ab10a740118776febecb4eabb` |
| HEAD subject | `feat: add selectable DCA1000 LVDS lane mode` |
| HEAD date | `2026-09-11T10:57:19+08:00` |
| `origin/feat/codex` | `25ff6a001b60fb69427e59208501aef7fee848f1` |
| Ahead/behind relative to `origin/feat/codex` | local side 16 commits; remote side 3 commits |
| Worktree status at capture | untracked `plan/` only |
| Upstream tracking | none shown for `codex/radar-camera-web-fusion` |

This is a diverged-history baseline, not a clean branch from the current remote tip. The local side contains the DCA1000 and radar-analysis work needed by this project. The remote side contains commit `f80da27`, which adds camera source along with GNSS/IMU work and tracked sample media.

### Camera source availability

- `HEAD` contains no `tools/camera` files.
- `origin/feat/codex` contains:
  - `tools/camera/camera.py`
  - `tools/camera/camera_config.cfg`
  - tracked JPEG, MP4 and text artifacts under `tools/camera/output/`
- Task 0 must restore only the two text source/config files from commit `f80da27` and must not restore `tools/camera/output/`.
- Do not merge or overwrite the entire remote tree: comparing `HEAD..origin/feat/codex` shows deletion of local DCA1000, capture and range-analysis files.

## 5. Raspberry Pi Hardware

| Item | Observed value |
|---|---|
| Board | Raspberry Pi 4 Model B Rev 1.5 |
| Architecture | `aarch64` |
| CPU | 4× Cortex-A72, 600–1800 MHz |
| RAM | 1.8 GiB usable |
| Swap | 1.8 GiB zram |
| System storage | USB mass-storage device `AITO`, 57.7 GB |
| Root filesystem | `/dev/sda2`, ext4, 57 GB |
| Root free space | approximately 39 GB; 30% used |
| Capture-time temperature | `52.1 °C` |
| Power/throttle flags | `throttled=0x50000` |

`0x50000` has no current low-order fault bits but records that under-voltage and throttling occurred previously. This is a hardware acceptance warning. It does not prove a fault was active during the snapshot. Before a 30-minute combined radar/camera test, use a correctly rated supply and record `vcgencmd get_throttled` before and after; a new event invalidates the performance run until power is corrected.

The reported `volt=0.9160V` is the SoC core rail and is not the 5 V input measurement.

## 6. Raspberry Pi Operating System and Toolchain

| Tool/system | Observed value |
|---|---|
| OS | Debian GNU/Linux 13.2 (`trixie`) |
| Kernel | `6.12.47+rpt-rpi-v8` aarch64 |
| Time zone | Asia/Shanghai, UTC+08:00 |
| Clock status | synchronized; NTP active |
| Python | `3.13.5`, `/usr/bin/python3` |
| Git | `2.47.3` |
| FFmpeg | `7.1.5-0+deb13u1+rpt1` |
| v4l2-ctl | `1.30.1` |
| Python `websockets` imported version | `16.0` from `/home/pi/.local/lib/python3.13/site-packages` |
| Debian `python3-websockets` package | `15.0.1-1` |
| Python `serial` | `3.5` from Debian dist-packages |
| Debian `python3-serial` package | `3.5-2` |

The user-local `websockets 16.0` shadows Debian package `15.0.1`. Deployment tests must therefore record `python3 -c "import websockets; print(websockets.__version__, websockets.__file__)"`, not infer the runtime version from `dpkg` alone.

## 7. Connected Radar and Camera Interfaces

### Radar control/data USB

| Item | Observed value |
|---|---|
| USB VID:PID | `0451:bef3` |
| Device identity | Texas Instruments XDS110 `03.00.00.29`, CMSIS-DAP |
| Stable serial identity | `Texas_Instruments_XDS110__03.00.00.29__Embed_with_CMSIS-DAP_RE450055` |
| Linux driver | `cdc_acm` |
| Ports | `/dev/ttyACM0`, `/dev/ttyACM1` |
| USB link | 12 Mbit/s through the Pi USB 2 hub |

The current service assigns `/dev/ttyACM0` as config and `/dev/ttyACM1` as data. This mapping must be verified after reboot; the long-term deployment should add stable udev links based on the XDS110 identity and interface number.

### Camera

| Item | Observed value |
|---|---|
| USB identity | `1bcf:2cd1`, TSTC USB20 WEB CAMERA |
| Driver | `uvcvideo` |
| USB link | 480 Mbit/s through the Pi USB 2 hub |
| Stable video link | `/dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0` |
| Resolved node | `/dev/video0` |
| Current format | MJPG, 1280×720, 30 FPS |
| Process owning `/dev/video0` | none at snapshot |

Enumerated MJPEG modes include 3840×2160, 2592×1944, 2048×1536, 1600×1200, 1920×1080, 1280×960, 1280×720, 800×600, 640×480 and 320×240. The plan baseline remains 1280×720 at 30 FPS to limit Pi memory, CPU and browser latency.

The camera also exposes audio interfaces. Audio capture is outside the first fusion plan and must remain disabled in FFmpeg arguments.

## 8. Network and DCA1000 Link

| Interface | Address | Purpose |
|---|---|---|
| `wlan0` | `172.20.10.10/24` | Lab/PC SSH and web access |
| `eth0` | `192.168.33.30/24` | Dedicated DCA1000 network |
| DCA1000 configured peer | `192.168.33.180` |

At snapshot time:

- `eth0` was `UP, LOWER_UP`, with zero RX/TX errors or drops reported by the interface.
- ARP/neighbour cache contained DCA1000 MAC `0c:22:38:4e:5a:0c` in `STALE` state.
- One ICMP ping to `192.168.33.180` received no reply. This is not sufficient to declare the DCA1000 unavailable because its UDP control/data behavior, not ICMP, is the acceptance criterion.
- No DCA configuration or capture command was sent during environment export.

## 9. Current Pi Runtime and Deployment Drift

| Item | Observed value |
|---|---|
| `radar.service` | active/running |
| WebSocket listener | `0.0.0.0:8765`, PID 809 |
| Camera HTTP listener | no listener on port 8081 |
| Current ExecStart | `/usr/bin/python3 /home/pi/radar_server.py --cfg_port /dev/ttyACM0 --data_port /dev/ttyACM1 --cfg_file /home/pi/profile-2944.cfg --ws_port 8765` |
| Target project `/home/pi/camera_web_fusion` | missing |
| Legacy repository | `/home/pi/awr2944_config_with_process_github` exists |
| Standalone camera area | `/home/pi/tools/camera` exists |

The running `/home/pi/radar_server.py` is outside the future project checkout. Initial deployment must preserve the current command for rollback, establish `/home/pi/camera_web_fusion`, run the radar-only regression from the new checkout, and only then update systemd. Camera enablement remains a separate later gate.

## 10. Execution-Start Blockers and Gates

### Gate A: Protect Git history

- Do not reset the feature branch to `origin/feat/codex`.
- Do not merge the entire remote branch before reviewing its deletions.
- Recover only `camera.py` and `camera_config.cfg` from `f80da27` as specified in Task 0.
- Commit the plan/environment baseline and camera-output ignore rules before functional changes.

### Gate B: Create reproducible PC Python environment

- Create `.venv` in the new worktree.
- Install and pin `websockets` and `pyserial` there.
- Add OpenCV only when starting offline calibration.
- Run the radar-only baseline suite before modifying source.

### Gate C: Establish the Pi checkout without interrupting the current service

- Create or clone `/home/pi/camera_web_fusion` at the verified feature commit.
- Do not edit `/home/pi/radar_server.py` in place.
- Verify repository-based radar startup manually before changing systemd.
- Save the old systemd unit and ExecStart value for rollback.

### Gate D: Resolve power-history warning

- Record `vcgencmd get_throttled` before and after each hardware acceptance run.
- Use a supply and cabling arrangement that causes no new under-voltage/throttle events.
- Log temperature, memory and disk during the 30-minute camera-only and combined tests.

### Gate E: Preserve hardware safety

- Automated tests do not open camera, radar serial ports or GPIO by default.
- No environment setup step issues a radar `sensorStart` or DCA record command.
- No automated step moves the gimbal.
- Sector-scan testing requires an operator, verified limits, cable routing and emergency stop.

## 11. Re-Capture Commands

Run these commands when the baseline must be refreshed. Review output before copying it into this document; do not store credentials or environment secrets.

### PC

```powershell
Set-Location D:\hp-laptop\USV\awr2944_radar_camera_web_fusion
git branch --show-current
git rev-parse HEAD
git status --short
git rev-list --left-right --count HEAD...origin/feat/codex
python --version
python -m pip freeze | Sort-Object
git --version
ffmpeg -version | Select-Object -First 3
```

### Raspberry Pi

```bash
ssh pi@172.20.10.10
date --iso-8601=seconds
timedatectl
uname -a
python3 --version
python3 -c 'import websockets, serial; print(websockets.__version__, websockets.__file__); print(serial.__version__, serial.__file__)'
git -C /home/pi/camera_web_fusion branch --show-current
git -C /home/pi/camera_web_fusion rev-parse HEAD
git -C /home/pi/camera_web_fusion status --short
readlink -f /dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0
v4l2-ctl --device /dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0 --get-fmt-video --get-parm
systemctl show radar.service -p FragmentPath -p ExecStart -p ActiveState -p SubState
ss -ltnp | grep -E ':(8765|8081)\b'
vcgencmd measure_temp
vcgencmd get_throttled
free -h
df -h /
```

If the Pi IP changes, substitute the current address only in the operator command. Do not commit a transient Wi-Fi/phone-hotspot address as application configuration.
