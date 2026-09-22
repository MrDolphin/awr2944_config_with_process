# Radar-Camera Web Fusion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在树莓派上由单一相机采集进程稳定取得带时间戳的画面，将它与 AWR2944P 点云按同一主机时钟匹配，在现有网页中同步显示、记录，并在完成标定和云台姿态接入后支持可信的点云投影叠加。

**Architecture:** 保留现有 `radar_server.py` WebSocket 点云链路，新增一个由雷达服务管理、默认关闭的 `CameraRuntime`。相机运行时独占 UVC 设备，通过 FFmpeg 读取 MJPEG 并把最近帧保存在有界环形缓冲区；轻量 HTTP 服务提供带帧号和单调时钟时间戳的 JPEG。雷达帧进入服务端时，由纯函数同步器寻找时间上最近的相机帧，并把匹配结果加入 WebSocket 消息。网页先提供“相机画面 + 雷达图”并排与匹配帧回放，完成内外参和云台角度接入后，再在独立叠加画布上投影点云。相机、雷达、编码器数据都以树莓派 `monotonic_ns` 为内部时间基准，墙钟只用于文件命名和人类阅读。

**Tech Stack:** Python 3 标准库、现有 `websockets`、FFmpeg/v4l2、HTML/CSS/原生 JavaScript、现有 `unittest` 测试体系；离线标定允许在 PC 使用 OpenCV，树莓派在线投影仅使用已保存的标定参数和纯 Python 数学运算。

## Global Constraints

- PC 端只在已建立的工作树 `D:\hp-laptop\USV\awr2944_radar_camera_web_fusion`、分支 `codex/radar-camera-web-fusion` 开发本功能；不要在 `awr2944_config_and_process_with_trace_codex` 或 `awr2944_sea_clutter_v02` 同时实现同一功能。
- 树莓派部署和现场调试统一使用 `/home/pi/camera_web_fusion`。当前该目录尚未创建，因此第一次部署必须显式建立仓库检出并验证提交号，不能继续把 `/home/pi/radar_server.py` 当作本功能的开发副本。
- PC 分支当前基线 `840a4feb5211b96ab10a740118776febecb4eabb` 与 `origin/feat/codex` 已经分叉：本地侧独有 16 个提交，远端侧独有 3 个提交。禁止直接快进、硬重置或整树覆盖；先按 Task 0 仅恢复相机源文件并保护本地雷达采集改动。
- 相机和云台硬件适配器默认关闭。自动化测试不得打开 `/dev/video*`、串口、GPIO 或启动电机；真实运动只允许操作员在场时显式触发。
- 一个 UVC 设备只能由一个采集所有者打开；网页预览、同步匹配和录像必须从同一 `CameraRuntime` 分流，不能各自启动 FFmpeg 抢占设备。
- 第一阶段的同步是“树莓派收到数据时刻”的软件同步，不等同于雷达射频采样时刻的硬件触发同步。所有报告和 UI 必须保留该限定。
- 第一阶段完成意味着同步显示和可复现实验记录完成，不代表完成空间融合、目标检测、船只识别或油膜检测。
- 不把示例照片、录像、BIN 原始数据或现场采集结果提交到 Git；只提交代码、测试、小型配置示例和文档。

## Execution Environment Baseline

- 当前执行环境快照：`plan/2026-09-22-radar-camera-web-fusion-environment.md`
- PC 工作树：`D:\hp-laptop\USV\awr2944_radar_camera_web_fusion`
- PC 分支：`codex/radar-camera-web-fusion`
- 树莓派目标检出：`/home/pi/camera_web_fusion`
- 树莓派运行用户：`pi`
- 当前实验室 SSH 地址：`pi@172.20.10.10`；地址可能随网络模式变化，部署脚本不得把它写入业务代码。
- 所有阶段开始前先核对环境快照中的“执行前阻塞项”；快照记录的是 2026-09-22 的事实，不作为以后设备状态不变的保证。

---

## Delivery Map

| 阶段 | 可交付结果 | 预计工程时间 | 人工验收点 |
|---|---|---:|---|
| 0 | 隔离分支、现状基线、真实相机参数固化 | 0.5 天 | 用户确认 Pi 相机设备符号链接 |
| 1 | 单一相机采集运行时与状态接口 | 1–2 天 | 1280×720 MJPEG 连续 30 分钟 |
| 2 | 网页并排显示与时间匹配状态 | 1 天 | 相机与点云同页稳定显示 |
| 3 | 同步记录包与离线复现 | 1–2 天 | 同一实验目录可重放与追溯 |
| 4 | 相机标定、雷达到相机外参和静态投影 | 2–3 天 | 静态标靶投影误差过门限 |
| 5 | 云台编码器姿态接入与旋转叠加 | 2–3 天 | 操作员监护的扇区扫描验收 |
| 6 | 部署、长稳和外场验收 | 1–2 天 | 30 分钟联合运行与故障恢复 |

总计约 8–13 个工程日，另计标靶制作、现场布置和天气窗口。每个阶段验收通过后再进入下一阶段。

---

### Task 0: Establish an isolated baseline

**Files:**
- Inspect: `plan/2026-09-22-radar-camera-web-fusion-environment.md`
- Modify: `.gitignore`
- Recover: `tools/camera/camera.py`
- Recover: `tools/camera/camera_config.cfg`
- Inspect: `radar_server.py`
- Inspect: `radar_app.html`
- Inspect: `deploy/radar.service`
- Inspect: `deploy/setup_rpi.sh`
- Test: `test/test_radar_server_serial.py`
- Test: `test/test_radar_app.py`
- Test: `test/test_deploy_templates.py`

- [ ] **Step 1: Verify the pre-created worktree and exported baseline**

Run from the designated PC worktree:

```powershell
Set-Location D:\hp-laptop\USV\awr2944_radar_camera_web_fusion
git branch --show-current
git rev-parse HEAD
git status --short
Test-Path .\plan\2026-09-22-radar-camera-web-fusion-environment.md
```

Expected: branch is `codex/radar-camera-web-fusion`, HEAD is `840a4feb5211b96ab10a740118776febecb4eabb` before the first implementation commit, status contains only the untracked `plan/` directory, and the final line is `True`. If HEAD has legitimately advanced, record the new commit in the validation report instead of resetting it.

- [ ] **Step 2: Reconfirm branch divergence without changing history**

```powershell
git fetch origin feat/codex
git rev-list --left-right --count HEAD...origin/feat/codex
git log --left-right --cherry-pick --oneline HEAD...origin/feat/codex
```

Expected at the exported baseline: `16 3`. Do not run `git reset --hard`, do not merge the full remote branch, and do not copy the remote tree wholesale because the remote-only side deletes local DCA/range-analysis files.

- [ ] **Step 3: Recover only the two camera source/config files from their source commit**

```powershell
git restore --source=f80da27 -- tools/camera/camera.py tools/camera/camera_config.cfg
git status --short -- tools/camera
```

Expected: only `tools/camera/camera.py` and `tools/camera/camera_config.cfg` appear. Do not restore `tools/camera/output`; commit `f80da27` contains tracked photos and videos that are experimental artifacts.

- [ ] **Step 4: Add camera-output ignore rules and commit the recovered baseline**

Append these exact rules if equivalent rules are not already present:

```gitignore
tools/camera/output/
camera_sessions/
```

Then run:

```powershell
git add .gitignore tools/camera/camera.py tools/camera/camera_config.cfg plan
git commit -m "chore: establish radar camera fusion baseline"
```

Expected: one commit containing plans, environment evidence, the two recovered camera text files, and ignore rules; no image, video or BIN file is tracked.

- [ ] **Step 5: Run the unmodified baseline tests**

```powershell
python -m unittest discover -s test -p "test_radar_*.py" -v
python -m unittest test.test_deploy_templates -v
```

Expected: all existing radar and deployment tests pass. Record any pre-existing failure before writing production code.

---

### Task 1: Define and validate the real camera contract

**Files:**
- Modify: `tools/camera/camera_config.cfg`
- Create: `tools/camera/camera_config.py`
- Create: `tools/camera/camera_probe.py`
- Create: `test/test_camera_config.py`
- Modify: `deploy/setup_rpi.sh`

- [ ] **Step 1: Write failing config parsing tests**

Add tests for a typed configuration object:

```python
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from tools.camera.camera_config import CameraConfig, load_camera_config


class CameraConfigTest(unittest.TestCase):
    def test_loads_stable_device_and_supported_mode(self):
        with TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "camera.cfg"
            path.write_text(
                "device=/dev/v4l/by-id/camera-video-index0\n"
                "width=1280\nheight=720\nfps=30\ninput_format=mjpeg\n",
                encoding="utf-8",
            )
            config = load_camera_config(path)
        self.assertEqual(config.width, 1280)
        self.assertEqual(config.height, 720)
        self.assertEqual(config.fps, 30)
        self.assertEqual(config.input_format, "mjpeg")

    def test_rejects_non_positive_frame_rate(self):
        with self.assertRaisesRegex(ValueError, "fps"):
            CameraConfig(device="/dev/video0", width=1280, height=720, fps=0)
```

Run:

```powershell
python -m unittest test.test_camera_config -v
```

Expected: import failure because `camera_config.py` does not exist.

- [ ] **Step 2: Implement the typed config parser**

Expose these exact interfaces:

```python
@dataclass(frozen=True)
class CameraConfig:
    device: str
    width: int
    height: int
    fps: int
    input_format: str = "mjpeg"


def load_camera_config(path: Path) -> CameraConfig:
    """Parse key=value camera settings and reject invalid dimensions/rates."""
```

Validation rules: device is non-empty; width, height and fps are positive; `input_format` is `mjpeg`; unknown keys raise `ValueError` so spelling errors cannot silently change acquisition.

- [ ] **Step 3: Replace the unsupported deployed mode**

Set `tools/camera/camera_config.cfg` to the proven Pi mode:

```ini
device=/dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0
width=1280
height=720
fps=30
input_format=mjpeg
```

Do not retain `1440x720`; the current camera silently falls back to 1280×720, making metadata incorrect.

- [ ] **Step 4: Add a read-only probe command**

`camera_probe.py` must run `v4l2-ctl --list-formats-ext --device <device>`, parse supported MJPEG modes, compare the selected mode, and return non-zero without opening a stream when the mode is unsupported.

Test the parser using stored text fixtures inside the test file; do not require a camera in unit tests.

- [ ] **Step 5: Add Pi packages to deployment prerequisites**

Add `ffmpeg` and `v4l-utils` to the apt package list in `deploy/setup_rpi.sh`. Keep Python dependencies unchanged.

- [ ] **Step 6: Verify and commit**

```powershell
python -m unittest test.test_camera_config test.test_deploy_templates -v
git add tools/camera/camera_config.cfg tools/camera/camera_config.py tools/camera/camera_probe.py test/test_camera_config.py deploy/setup_rpi.sh
git commit -m "feat: define validated camera capture settings"
```

Expected: all tests pass; no device is accessed.

---

### Task 2: Build a single-owner timestamped camera runtime

**Files:**
- Create: `tools/camera/camera_capture.py`
- Create: `test/test_camera_capture.py`
- Modify: `tools/camera/camera.py`

- [ ] **Step 1: Write failing JPEG stream parser and buffer tests**

Cover fragmented FFmpeg output, two consecutive JPEG frames, buffer eviction, monotonic frame IDs and shutdown:

```python
class CameraCaptureTest(unittest.TestCase):
    def test_extracts_fragmented_jpeg_frames(self):
        parser = JpegStreamParser()
        self.assertEqual(parser.feed(b"noise\xff"), [])
        self.assertEqual(parser.feed(b"\xd8abc\xff\xd9"), [b"\xff\xd8abc\xff\xd9"])

    def test_ring_buffer_returns_nearest_frame(self):
        buffer = CameraFrameBuffer(capacity=3)
        buffer.append(make_frame(1, 1_000_000_000))
        buffer.append(make_frame(2, 1_040_000_000))
        self.assertEqual(buffer.nearest(1_030_000_000).frame_id, 2)
```

Run:

```powershell
python -m unittest test.test_camera_capture -v
```

Expected: import failure for the missing module.

- [ ] **Step 2: Implement immutable frame metadata and bounded storage**

Use these interfaces:

```python
@dataclass(frozen=True)
class CameraFrame:
    frame_id: int
    host_monotonic_ns: int
    host_wall_time_ns: int
    width: int
    height: int
    jpeg: bytes


class CameraFrameBuffer:
    def __init__(self, capacity: int = 120): ...
    def append(self, frame: CameraFrame) -> None: ...
    def latest(self) -> CameraFrame | None: ...
    def get(self, frame_id: int) -> CameraFrame | None: ...
    def nearest(self, host_monotonic_ns: int) -> CameraFrame | None: ...
```

Protect storage with one lock. Copy references, not JPEG byte arrays. Capacity 120 retains roughly four seconds at 30 FPS.

- [ ] **Step 3: Implement the FFmpeg capture owner**

Expose:

```python
class CameraRuntime:
    def __init__(self, config: CameraConfig, frame_buffer: CameraFrameBuffer, clock=time): ...
    def start(self) -> None: ...
    def stop(self, timeout_s: float = 5.0) -> None: ...
    def status(self) -> dict[str, object]: ...
```

Construct an argument list, never a shell string:

```text
ffmpeg -hide_banner -loglevel warning -f v4l2 -input_format mjpeg
-framerate 30 -video_size 1280x720 -i <stable-device>
-an -c:v copy -f image2pipe pipe:1
```

Stamp each complete JPEG immediately after reading it with `time.monotonic_ns()` and `time.time_ns()`. Capture stderr separately, retain its last 20 lines in status, detect unexpected FFmpeg exit, and make `stop()` terminate then kill only after timeout.

- [ ] **Step 4: Refactor the old CLI to reuse the config and process builder**

Keep existing photo/record commands functional, but make them consume `CameraConfig`. Change output naming to include microseconds or an incrementing suffix so two photos in one second cannot collide.

- [ ] **Step 5: Verify runtime logic and commit**

```powershell
python -m unittest test.test_camera_capture test.test_camera_config -v
git add tools/camera/camera_capture.py tools/camera/camera.py test/test_camera_capture.py
git commit -m "feat: add timestamped single-owner camera runtime"
```

Expected: tests use fake process streams and pass without FFmpeg or camera hardware.

---

### Task 3: Serve timestamped camera frames over HTTP

**Files:**
- Create: `tools/camera/camera_http.py`
- Create: `test/test_camera_http.py`

- [ ] **Step 1: Write failing HTTP contract tests**

Start the server on port `0` with a preloaded buffer and verify:

- `GET /camera/status` returns JSON and `running`, `latest_frame_id`, `fps_observed`.
- `GET /camera/frame/latest.jpg` returns JPEG bytes.
- `GET /camera/frame/17.jpg` returns the exact retained frame or 404.
- Image responses include `X-Camera-Frame-Id`, `X-Capture-Monotonic-Ns`, `X-Capture-Wall-Time-Ns`, `Cache-Control: no-store`, and `Access-Control-Allow-Origin: *`.
- Unknown paths return 404; missing frames return JSON error bodies.

Run:

```powershell
python -m unittest test.test_camera_http -v
```

Expected: import failure for `camera_http.py`.

- [ ] **Step 2: Implement a standard-library threaded HTTP server**

Expose:

```python
class CameraHttpServer:
    def __init__(self, host: str, port: int, runtime: CameraRuntime, frame_buffer: CameraFrameBuffer): ...
    @property
    def bound_port(self) -> int: ...
    def start(self) -> None: ...
    def stop(self) -> None: ...
```

Use `ThreadingHTTPServer`; cap response size to one frame and never write an unbounded MJPEG stream. This lets the browser read per-frame timestamps and prevents a slow client from holding the capture thread.

- [ ] **Step 3: Verify and commit**

```powershell
python -m unittest test.test_camera_http test.test_camera_capture -v
git add tools/camera/camera_http.py test/test_camera_http.py
git commit -m "feat: expose timestamped camera frame service"
```

---

### Task 4: Match radar frames to camera frames

**Files:**
- Create: `radar_camera_sync.py`
- Create: `test/test_radar_camera_sync.py`
- Modify: `radar_server.py`
- Modify: `test/test_radar_server_serial.py`

- [ ] **Step 1: Write failing pure synchronization tests**

Use an explicit result type:

```python
@dataclass(frozen=True)
class CameraSyncResult:
    status: str
    frame_id: int | None
    capture_monotonic_ns: int | None
    time_offset_ms: float | None
    frame_url: str | None
```

Test exact match, nearest earlier/later match, empty buffer, stale match and deterministic tie-breaking. Use provisional quality bands:

- `matched`: absolute offset ≤ 50 ms
- `degraded`: 50 ms < absolute offset ≤ 100 ms
- `stale`: absolute offset > 100 ms
- `unavailable`: no camera frame

- [ ] **Step 2: Implement the pure matcher**

```python
def match_camera_frame(
    radar_monotonic_ns: int,
    frame_buffer: CameraFrameBuffer,
    base_url: str,
    matched_limit_ms: float = 50.0,
    stale_limit_ms: float = 100.0,
) -> CameraSyncResult:
    ...
```

The sign convention is `camera_time - radar_time`; document it and test both signs.

- [ ] **Step 3: Add disabled-by-default server arguments**

Add to `radar_server.py`:

```text
--enable-camera
--camera-config tools/camera/camera_config.cfg
--camera-http-host 0.0.0.0
--camera-http-port 8081
--camera-public-base-url http://<pi-host>:8081
```

Do not instantiate `CameraRuntime` unless `--enable-camera` is provided. Importing `radar_server.py` in tests must not probe hardware.

- [ ] **Step 4: Attach synchronization metadata to each radar message**

Keep the existing `host_time_s` and `host_monotonic_s` fields. Add:

```json
"camera_sync": {
  "status": "matched",
  "frame_id": 1234,
  "capture_monotonic_ns": 987654321000,
  "time_offset_ms": -12.4,
  "frame_url": "http://pi-host:8081/camera/frame/1234.jpg",
  "clock_basis": "pi_receive_monotonic"
}
```

If the camera is disabled or fails, continue radar service and emit `unavailable`; camera failure must not stop point-cloud delivery.

- [ ] **Step 5: Add lifecycle tests**

Patch fake camera runtime and HTTP server objects. Verify start order, stop order, failure isolation and that SIGINT cleanup stops both objects exactly once.

- [ ] **Step 6: Verify and commit**

```powershell
python -m unittest test.test_radar_camera_sync test.test_radar_server_serial -v
git add radar_camera_sync.py radar_server.py test/test_radar_camera_sync.py test/test_radar_server_serial.py
git commit -m "feat: synchronize radar messages with camera frames"
```

---

### Task 5: Add synchronized camera display to the existing webpage

**Files:**
- Modify: `radar_app.html`
- Modify: `test/test_radar_app.py`

- [ ] **Step 1: Write failing static UI contract tests**

Extend `test_radar_app.py` to assert the page contains stable DOM IDs:

```text
cameraCanvas
cameraStatus
cameraFrameId
cameraSyncOffset
cameraDisplayMode
cameraOverlayCanvas
```

Also assert JavaScript includes `fetchCameraFrame`, `drawCameraFrame`, and does not use an unbounded `<img src="...mjpeg">` stream.

- [ ] **Step 2: Add a camera panel beside the radar canvases**

The panel contains:

- canvas preserving the configured 16:9 aspect ratio;
- connection state (`disabled`, `connecting`, `live`, `stale`, `error`);
- camera frame ID and capture wall-clock time;
- radar-to-camera offset badge with green/yellow/red quality bands;
- display selector: `最新画面` and `匹配雷达帧`;
- explicit text: `软件接收时钟同步，非硬件触发同步`.

- [ ] **Step 3: Fetch frames sequentially and draw to canvas**

Implement:

```javascript
async function fetchCameraFrame(url, abortSignal) {
  const response = await fetch(url, { cache: "no-store", signal: abortSignal });
  if (!response.ok) throw new Error(`camera HTTP ${response.status}`);
  const blob = await response.blob();
  const bitmap = await createImageBitmap(blob);
  return {
    bitmap,
    frameId: Number(response.headers.get("X-Camera-Frame-Id")),
    captureMonotonicNs: response.headers.get("X-Capture-Monotonic-Ns"),
  };
}
```

Only request the next image after the previous response is decoded or aborted. Close old `ImageBitmap` objects after drawing. In matched mode, use the URL received in the radar message and ignore an older response if a newer radar frame already arrived.

- [ ] **Step 4: Keep phase-one rendering side-by-side**

Do not project points onto pixels in this task. Retain the existing radar/PPI visualization and add a clear label `尚未空间标定`; this avoids presenting aligned timing as aligned geometry.

- [ ] **Step 5: Verify and commit**

```powershell
python -m unittest test.test_radar_app -v
git add radar_app.html test/test_radar_app.py
git commit -m "feat: show synchronized camera frames in radar web UI"
```

Expected: static tests pass. Browser/device acceptance is deferred to Task 11.

---

### Task 6: Record a reproducible synchronized experiment package

**Files:**
- Create: `radar_camera_recording.py`
- Create: `test/test_radar_camera_recording.py`
- Modify: `radar_server.py`
- Modify: `radar_app.html`

- [ ] **Step 1: Write failing session writer tests**

For a temporary session directory, verify these files and columns:

```text
session_metadata.json
radar_frames.jsonl
fusion_index.csv
camera_frames/<frame_id>.jpg
```

`fusion_index.csv` columns:

```text
radar_frame_num,radar_monotonic_ns,camera_frame_id,camera_monotonic_ns,time_offset_ms,sync_status,yaw_deg,pitch_deg,pose_age_ms
```

Test duplicate camera frame IDs write one JPEG only, partial sessions close cleanly, and absent camera/pose values serialize as empty fields.

- [ ] **Step 2: Implement a bounded session writer**

Expose:

```python
class RadarCameraSessionWriter:
    def start(self, root: Path, metadata: dict[str, object]) -> Path: ...
    def append(self, radar_message: dict[str, object], camera_frame: CameraFrame | None, pose: SensorPose | None) -> None: ...
    def stop(self) -> None: ...
```

Write via temporary filenames and atomic rename for JSON metadata. Flush line-oriented indexes at least once per second. Save only the camera frame matched to a radar frame for the first implementation; do not add continuous H.264 indexing complexity yet.

- [ ] **Step 3: Add recording controls without changing existing radar-only recording defaults**

Add a separate `同步记录` control. The UI sends a WebSocket command; the server acknowledges the created session path. Existing point-cloud recording remains compatible.

- [ ] **Step 4: Add clock and configuration provenance**

`session_metadata.json` must include:

- Git commit and dirty flag;
- radar CLI/config snapshot identifier;
- camera config values and stable device path;
- sync threshold values;
- mount mode;
- wall-clock start/end and monotonic start/end;
- statement that radar timestamps are receive-time unless hardware timestamps are available.

- [ ] **Step 5: Verify and commit**

```powershell
python -m unittest test.test_radar_camera_recording test.test_radar_server_serial test.test_radar_app -v
git add radar_camera_recording.py radar_server.py radar_app.html test/test_radar_camera_recording.py
git commit -m "feat: record synchronized radar and camera sessions"
```

---

### Task 7: Add calibration data and pure point projection

**Files:**
- Create: `tools/fusion/__init__.py`
- Create: `tools/fusion/calibration.py`
- Create: `tools/fusion/projection.py`
- Create: `tools/fusion/calibrate_radar_camera.py`
- Create: `Config/radar_camera_calibration.example.json`
- Create: `test/test_radar_camera_projection.py`
- Create: `docs/radar_camera_calibration.md`

- [ ] **Step 1: Write failing schema and projection tests**

Test:

- valid calibration loads;
- non-orthonormal rotation is rejected;
- a point behind the camera is omitted;
- a point on the optical axis projects to `(cx, cy)`;
- radial/tangential distortion has a deterministic expected result;
- image-bound clipping is optional and explicit.

Core interface:

```python
@dataclass(frozen=True)
class RadarCameraCalibration:
    image_width: int
    image_height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, float, float, float, float]
    rotation_radar_to_camera: tuple[tuple[float, float, float], ...]
    translation_radar_to_camera_m: tuple[float, float, float]
    mount_mode: str
    rms_reprojection_error_px: float


def project_radar_point(
    point_radar_m: tuple[float, float, float],
    calibration: RadarCameraCalibration,
    pose: SensorPose | None = None,
) -> tuple[float, float, float] | None:
    """Return image u, v and positive camera depth, or None."""
```

- [ ] **Step 2: Implement strict calibration loading**

The JSON contains:

```json
{
  "schema_version": 1,
  "image_size": [1280, 720],
  "camera_matrix": {"fx": 0.0, "fy": 0.0, "cx": 0.0, "cy": 0.0},
  "distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
  "radar_to_camera": {
    "rotation_3x3": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    "translation_m": [0.0, 0.0, 0.0]
  },
  "mount_mode": "co_rotating",
  "calibrated_at": "2000-01-01T00:00:00Z",
  "rms_reprojection_error_px": 0.0
}
```

The example file is deliberately non-deployable: the loader must reject zero focal lengths, so it cannot be mistaken for a real calibration.

- [ ] **Step 3: Implement an offline PC calibration command**

The command consumes a CSV of corresponding radar 3D points and image pixels plus camera intrinsics, solves the rigid transform using OpenCV on the PC, reports RMS/median/p95 reprojection error, and writes the validated JSON. Keep OpenCV optional and outside Pi runtime imports.

- [ ] **Step 4: Document the physical calibration procedure**

Use a radar reflector rigidly paired with a visible AprilTag or high-contrast board. Collect stationary samples at azimuths `-30, -15, 0, 15, 30` degrees and ranges `5, 10, 20` m, keeping the full paired target inside both fields of view. Measure camera-to-radar translation with a tape measure and use it as the solver initial check.

Initial acceptance gates:

- intrinsic calibration RMS ≤ 1.5 px;
- combined radar-camera median reprojection error ≤ 8 px;
- combined p95 error ≤ 20 px at calibration ranges;
- validation uses positions excluded from fitting.

- [ ] **Step 5: Verify and commit**

```powershell
python -m unittest test.test_radar_camera_projection -v
git add tools/fusion Config/radar_camera_calibration.example.json test/test_radar_camera_projection.py docs/radar_camera_calibration.md
git commit -m "feat: add radar camera calibration and projection model"
```

---

### Task 8: Attach timestamped gimbal pose without automatic motion

**Files:**
- Create: `sensor_pose.py`
- Create: `test/test_sensor_pose.py`
- Modify: `encoder_server.py`
- Modify: `test/test_encoder_server.py`
- Modify: `radar_server.py`

- [ ] **Step 1: Write failing pose history tests**

Define:

```python
@dataclass(frozen=True)
class SensorPose:
    host_monotonic_ns: int
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    source: str


class SensorPoseHistory:
    def append(self, pose: SensorPose) -> None: ...
    def nearest(self, host_monotonic_ns: int) -> tuple[SensorPose | None, float | None]: ...
```

Test bounded retention, out-of-order sample rejection, nearest lookup and pose age calculation.

- [ ] **Step 2: Publish measured encoder pose**

Extend the encoder data path to publish measured yaw/pitch with `monotonic_ns`, not only commanded angle or speed. Keep roll at zero until a measured source exists and label its source explicitly.

- [ ] **Step 3: Attach nearest pose to radar messages**

Add:

```json
"sensor_pose": {
  "yaw_deg": 12.3,
  "pitch_deg": -1.4,
  "roll_deg": 0.0,
  "pose_age_ms": 8.1,
  "source": "encoder"
}
```

Mark pose `stale` for age > 50 ms and suppress pixel overlay when age > 100 ms.

- [ ] **Step 4: Support the two physical mount modes in pure transforms**

- `co_rotating`: camera and radar share one rigid bracket; the radar-camera extrinsic stays fixed while the pair rotates. This is the recommended first prototype with one ordinary camera.
- `fixed_camera`: camera stays fixed while radar rotates; apply the measured dynamic yaw transform and project only points inside the camera field of view.

No code in this task may issue a motor command.

- [ ] **Step 5: Verify and commit**

```powershell
python -m unittest test.test_sensor_pose test.test_encoder_server test.test_radar_server_serial -v
git add sensor_pose.py encoder_server.py radar_server.py test/test_sensor_pose.py test/test_encoder_server.py
git commit -m "feat: timestamp measured sensor pose for fusion"
```

---

### Task 9: Add calibrated point-cloud overlay

**Files:**
- Modify: `radar_server.py`
- Modify: `radar_app.html`
- Modify: `test/test_radar_app.py`
- Modify: `test/test_radar_camera_projection.py`

- [ ] **Step 1: Write failing overlay message and UI tests**

Assert the server sends calibration identity/version and either projected points or sufficient 3D point + transform data. Assert the UI has:

- overlay enable checkbox disabled when calibration is absent;
- calibration status and reprojection error;
- legend for range/power coloring;
- a reason string when points are suppressed for stale camera or pose;
- an opacity control;
- a debug toggle for projected point IDs.

- [ ] **Step 2: Project on the server using the matched frame and measured pose**

For each radar point, compute `(u, v, depth)` with the exact calibration associated with the session. Clip behind-camera and out-of-frame points. Add:

```json
"camera_projection": {
  "status": "valid",
  "calibration_id": "sha256:...",
  "points": [{"u": 640.2, "v": 358.9, "range_m": 12.4, "power": 18.0}]
}
```

Suppress projection unless camera sync is `matched`, pose is fresh for a rotating mount, image size matches calibration, and calibration validation passed.

- [ ] **Step 3: Draw the overlay on a separate transparent canvas**

Keep camera pixels untouched. Resize the overlay canvas to the decoded image dimensions and map it with the same CSS transform. Display projected points by range or signal strength and show counts before/after clipping.

- [ ] **Step 4: Add an explicit non-detection disclaimer**

UI text: `投影点是坐标配准结果，不代表目标已被分类或确认。` Do not label points as ships, people, oil film or obstacles without a separate detection and truth-validation stage.

- [ ] **Step 5: Verify and commit**

```powershell
python -m unittest test.test_radar_camera_projection test.test_radar_app test.test_radar_server_serial -v
git add radar_server.py radar_app.html test/test_radar_app.py test/test_radar_camera_projection.py
git commit -m "feat: overlay calibrated radar points on camera frames"
```

---

### Task 10: Make deployment reproducible and eliminate Pi drift

**Files:**
- Modify: `deploy/radar.service`
- Modify: `deploy/setup_rpi.sh`
- Create: `deploy/radar-camera.env.example`
- Create: `deploy/DEPLOYMENT_CHECKLIST.md`
- Modify: `test/test_deploy_templates.py`

- [ ] **Step 1: Write failing deployment template tests**

Assert:

- the service executes `radar_server.py` from the repository checkout, not `/home/pi/radar_server.py`;
- camera is absent from default arguments;
- environment file can enable it explicitly;
- restart policy does not create a tight crash loop;
- shutdown timeout allows FFmpeg cleanup;
- setup script installs FFmpeg and v4l-utils.

- [ ] **Step 2: Parameterize the service**

Use an environment file with safe defaults:

```ini
RADAR_CAMERA_ARGS=
RADAR_CAMERA_CONFIG=/home/pi/camera_web_fusion/tools/camera/camera_config.cfg
RADAR_CAMERA_HTTP_PORT=8081
```

The operator enables the camera by setting `RADAR_CAMERA_ARGS=--enable-camera ...`; installation must not turn it on automatically.

- [ ] **Step 3: Add a deployment preflight checklist**

The checklist must verify:

```bash
readlink -f /dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0
v4l2-ctl --device /dev/v4l/by-id/usb-TSTC_USB20_WEB_CAMERA_TSTC_USB20_WEB_CAMERA_01.00.00-video-index0 --list-formats-ext
ss -ltnp | grep -E ':(8765|8081)\b'
pgrep -af 'ffmpeg|camera.py|radar_server.py'
```

Also record `git rev-parse HEAD`, `git status --short`, `vcgencmd measure_temp`, `free -h`, and `df -h` before and after the test.

- [ ] **Step 4: Add rollback instructions**

Rollback is one environment edit removing `--enable-camera`, then `systemctl daemon-reload` and `systemctl restart radar.service`. Preserve session data and logs; do not delete them as part of rollback.

- [ ] **Step 5: Verify and commit**

```powershell
python -m unittest test.test_deploy_templates -v
git add deploy/radar.service deploy/setup_rpi.sh deploy/radar-camera.env.example deploy/DEPLOYMENT_CHECKLIST.md test/test_deploy_templates.py
git commit -m "feat: deploy radar camera service with safe opt-in"
```

---

### Task 11: Perform operator-approved Pi acceptance in small gates

**Files:**
- Create after test: `docs/validation/radar-camera-<timestamp>.md`
- Generated outside Git: synchronized session under the configured data root

- [ ] **Step 1: Deploy without enabling camera**

Clone or update the verified commit at `/home/pi/camera_web_fusion`, run setup from that checkout, restart the radar-only service, and verify the existing page and WebSocket before enabling anything new. Before changing the service, record its current command because the exported baseline still runs `/home/pi/radar_server.py`.

Acceptance:

- WebSocket port 8765 listens;
- existing radar status renders;
- serial/radar behavior is unchanged;
- no FFmpeg process is running;
- no motor command is issued.

- [ ] **Step 2: Run camera-only acceptance**

With radar stopped or camera feature disabled, run the probe and camera service for 30 minutes at 1280×720 MJPEG 30 FPS.

Acceptance:

- measured frame rate ≥ 25 FPS;
- no unsupported-mode fallback;
- no unexpected FFmpeg restart;
- camera HTTP latest frame works from the operator PC;
- memory does not grow continuously;
- Pi CPU temperature is recorded at start, 10, 20 and 30 minutes.

- [ ] **Step 3: Run combined static acceptance**

Enable camera in the radar service while the gimbal remains stationary. Use a known target in both sensor fields of view.

Acceptance over 10 minutes:

- point-cloud messages continue without a camera-induced server failure;
- camera measured rate ≥ 25 FPS;
- camera sync matched ratio ≥ 95%;
- absolute receive-time offset p95 ≤ 50 ms;
- no radar frame loss attributable to camera loading;
- disconnecting the camera changes status to `unavailable` while radar continues;
- reconnect recovery occurs without restarting the entire Pi, or the validation report records the exact remaining restart requirement.

- [ ] **Step 4: Validate synchronized recording and offline replay**

Record a 60-second session, stop normally, copy it to the PC and verify every `fusion_index.csv` reference resolves to an existing radar row and camera JPEG. Replay ten selected radar frames and their matched images.

Acceptance: no missing referenced files; metadata contains exact configs, Git revision, clock basis and sync thresholds.

- [ ] **Step 5: Validate static spatial calibration**

Only after Task 7, collect the calibration grid and an independent validation grid. Do not move the gimbal during this gate.

Acceptance: intrinsic RMS ≤ 1.5 px, median radar-camera error ≤ 8 px and p95 ≤ 20 px. If the gate fails, inspect target association and coordinate axes before tuning thresholds.

- [ ] **Step 6: Validate a supervised sector scan**

Only after the operator confirms cable routing, mechanical limits, emergency stop and safe surroundings, command a slow bounded sector such as `-30° to +30°`. Camera and radar should initially be rigidly mounted together so the extrinsic remains fixed.

Acceptance:

- measured pose age p95 ≤ 50 ms;
- no overlay is shown with pose age > 100 ms;
- independent validation targets remain within p95 30 px while moving;
- no cable twist, limit collision or uncontrolled restart;
- stop command and emergency stop are demonstrated.

- [ ] **Step 7: Write the evidence report**

The validation Markdown records command, hardware connection, commit, config hashes, pass/fail table, screenshots, performance percentiles and all deviations. Separate automated/local tests from real Pi, camera, radar and gimbal results.

---

### Task 12: Run the full regression and prepare review

**Files:**
- Inspect: all changed files
- Create: pull request only after user requests or authorizes the remote workflow

- [ ] **Step 1: Run focused tests**

```powershell
python -m unittest `
  test.test_camera_config `
  test.test_camera_capture `
  test.test_camera_http `
  test.test_radar_camera_sync `
  test.test_radar_camera_recording `
  test.test_radar_camera_projection `
  test.test_sensor_pose `
  test.test_radar_server_serial `
  test.test_radar_app `
  test.test_encoder_server `
  test.test_deploy_templates -v
```

Expected: all focused tests pass without connected hardware.

- [ ] **Step 2: Run the full test suite**

```powershell
python -m unittest discover -s test -p "test_*.py" -v
```

Expected: all tests pass. Any hardware tests must be explicitly skipped unless an operator enabled them.

- [ ] **Step 3: Review the diff for unsafe side effects**

```powershell
git diff --check origin/feat/codex...HEAD
git status --short
git diff --stat origin/feat/codex...HEAD
git grep -n -E '/dev/video0|1440x720|/home/pi/radar_server.py|/home/pi/awr2944_config_with_process_github' -- . ':!plan'
```

Expected: no whitespace errors, no tracked generated data, no hard-coded volatile `/dev/video0`, no unsupported mode, and no service execution from the drifted Pi root copy.

- [ ] **Step 4: Prepare a review summary**

The summary must report separately:

- local unit/static test results;
- Pi camera-only acceptance;
- Pi radar-plus-camera acceptance;
- calibration accuracy;
- supervised gimbal result;
- uncompleted hardware gates.

Do not claim a gate passed from code inspection alone.

---

## Recommended Physical Arrangement

For the first working prototype, mount the ordinary USB camera and AWR2944P rigidly on the same slow pan/tilt platform, with optical axis and radar boresight as parallel as mechanical tolerances permit. This keeps one fixed radar-to-camera extrinsic transform and makes calibration and debugging tractable. Use a bounded sector scan rather than continuous 360° rotation until cable routing and slip-ring requirements are solved.

A fixed camera plus rotating radar is viable for surveillance coverage, but it is the second implementation mode: every point needs a fresh encoder angle, world-to-radar transform and camera field-of-view check. A single fixed camera cannot visually cover all bearings of a continuously rotating radar. For eventual continuous 360° operation, use multiple fixed cameras, a panoramic/360° camera, or a radar-cued PTZ camera rather than forcing one ordinary camera to follow every sweep.

## Definition of Done

The feature is complete only when all applicable statements are true:

- radar-only behavior remains intact with camera disabled;
- the camera runs in a truly supported 1280×720 MJPEG mode and has one capture owner;
- every displayed/recorded match carries frame IDs, timestamps, offset and clock basis;
- the webpage distinguishes live, matched, stale and unavailable camera states;
- a synchronized session can be copied and replayed without hidden dependencies;
- pixel overlay is unavailable until a valid calibration is loaded;
- rotating overlay uses measured, timestamped encoder pose and fails closed when stale;
- automated tests do not move hardware;
- Pi and field evidence are recorded separately from unit tests;
- all claims distinguish receive-time synchronization, spatial calibration and actual target detection.
