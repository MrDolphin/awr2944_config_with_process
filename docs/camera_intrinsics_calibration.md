# DW800 相机内参标定

本流程只求相机内参，不需要雷达、角反射器或云台运动。标定照片必须来自融合服务实际使用的相机模式；当前仓库配置是 MJPEG、1280×720、30 FPS。若以后更改分辨率或裁剪方式，必须重新标定。

## 1. 制作棋盘格

打印 `docs/assets/camera-intrinsics-checkerboard-9x6-20mm.svg`：

- A4 横向；
- 打印比例 100% 或“实际大小”，禁止“适合页面”；
- 图案是 10×7 个方格、9×6 个内角点，每格 20 mm；
- 打印后用尺测量页面下方 100 mm 参考线和棋盘格。误差明显时重新打印；
- 把纸完整贴在平整、刚性的板上，不能起皱或弯曲。

标定命令中的 `--columns 9 --rows 6 --square-mm 20` 必须与实物一致。OpenCV 参数使用的是内角点数，不是方格数量。

## 2. 采集 1280×720 原始 JPEG

保持树莓派相机服务运行，在 PC PowerShell 中执行：

```powershell
$imageDir = 'C:\calibration-case\camera-intrinsics\images'
New-Item -ItemType Directory -Path $imageDir -Force | Out-Null
$url = 'http://172.20.10.10:8081/camera/frame/latest.jpg'

1..24 | ForEach-Object {
    Read-Host "把棋盘移动到第 $_ 个位置，保持静止并按 Enter"
    $output = Join-Path $imageDir ('board-{0:D2}.jpg' -f $_)
    Invoke-WebRequest -Uri $url -OutFile $output -UseBasicParsing
    Start-Sleep -Milliseconds 300
}
```

采集 20–30 张，至少覆盖：

- 画面中央、四角、四条边；
- 正对相机、向左/右倾斜、向上/下倾斜；
- 近、中、远三个距离；
- 棋盘占画面宽度约 20%–70%。

每张照片必须看见完整棋盘，边缘清楚、不过曝、无明显运动模糊。不要只在画面中央连续拍摄，也不要通过网页截图、缩放图或二次压缩图片标定。

检查实际图片尺寸：

```powershell
cd D:\hp-laptop\USV\awr2944_radar_camera_web_fusion
.\.venv\Scripts\python.exe -c "import cv2, pathlib; p=next(pathlib.Path(r'C:\calibration-case\camera-intrinsics\images').glob('*.jpg')); im=cv2.imread(str(p)); print(p, im.shape[1], im.shape[0])"
```

结果应为 `1280 720`。如果不是，应先核对树莓派相机配置和本次标定会话中的 `camera_image_size`。

## 3. 求相机内参

输出文件必须是新路径，工具不会覆盖旧证据：

```powershell
cd D:\hp-laptop\USV\awr2944_radar_camera_web_fusion

$runId = Get-Date -Format 'yyyyMMdd-HHmmss'
$runDir = Join-Path 'C:\calibration-case\camera-intrinsics\runs' $runId
New-Item -ItemType Directory -Path $runDir -ErrorAction Stop | Out-Null

$intrinsics = Join-Path $runDir 'dw800-1280x720-intrinsics.json'

.\.venv\Scripts\python.exe tools\camera\calibrate_intrinsics.py `
    'C:\calibration-case\camera-intrinsics\images' `
    $intrinsics `
    --glob '*.jpg' `
    --columns 9 `
    --rows 6 `
    --square-mm 20 `
    --minimum-images 12

$calibrationExit = $LASTEXITCODE
Get-Content -Raw -Encoding UTF8 $intrinsics | ConvertFrom-Json
Get-Content -Raw -Encoding UTF8 ($intrinsics -replace '\.json$', '.report.json') | ConvertFrom-Json
Get-FileHash -Algorithm SHA256 $intrinsics
```

输出的内参 JSON 可直接传给 `tools/fusion/calibrate_radar_camera.py`。检查：

- `image_size` 为 `[1280, 720]`；
- `fx`、`fy` 为正数；
- `cx`、`cy` 大致位于图像中心附近；
- `distortion` 正好有 5 个值；
- `calibration_quality.accepted_images` 不少于 12；
- `calibration_quality.rms_px` 不大于 1.5 px；
- 报告中的 `quality_passed` 为 `true`；
- `$calibrationExit` 为 0。

RMS 超过 1.5 px 时不要继续求雷达—相机外参。查看报告中被拒绝的图片，重新拍摄模糊、过曝、棋盘太小或边角覆盖不足的照片。

## 4. 120° 广角边界

当前融合运行时支持 OpenCV `pinhole` 模型和 5 个径向/切向畸变参数。DW800 的 120° 标称视场较宽，即使产品称“无畸变”，也不能用标称视场角代替实测焦距和主点。

若 RMS 已通过，但雷达投影在画面中央准确、左右边缘仍系统性偏离，应保存证据并停止外参验收。这说明当前 5 参数模型可能不足，需要给运行时增加 fisheye 或更高阶畸变模型；不能通过手工调整外参来掩盖边缘模型误差。

