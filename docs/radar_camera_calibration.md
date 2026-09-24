# 雷达—相机人工角反射器静态标定

本流程用 **5 cm 角反射器**直接建立 AWR2944P 原始雷达点与相机像素的对应关系。雷达和相机须刚性固定，角反在每次选点时须静止。网页只导出会话 JSON；PC 上的 `tools/fusion/calibrate_radar_camera.py` 才生成候选外参。该流程不求相机内参，不自动控制云台、修改雷达配置或部署标定。仓库内的示例标定 JSON 焦距为零，不能部署。

## 1. 测量安装差并证明原始 Z 方向

记录**相机光心相对雷达天线相位中心**的位置，单位为米，基准不要取外壳边缘或镜头前缘。在雷达本地物理轴上，`dx_m` 向右为正，`dy_m` 向前为正，`dz_m` 向上为正。若雷达相位中心比相机光心高 10 cm，且左右、前后无差，则填写 `dx_m = 0.00`、`dy_m = 0.00`、`dz_m = -0.10`。同时记录测量方法、`uncertainty_m` 和基准 `reference`；可选填机械 yaw/pitch/roll。量不到时，网页中的安装测量留空并在验收记录说明，不能把估计值当作已测外参。

在采样前做**原始 Z 方向证明**：把角反固定在一个基准位置，记录同一目标原始点的 `x/y/z`、帧号和点索引；保持左右和前后位置，按尺量抬高一段已知距离，再降低并分别记录原始 `z`。抬高时原始 z 应增大，降低时应减小。把已知高度变化、观测值和截图写入[验收模板](validation/radar-camera-template.md)；若方向或目标关联不确定，停止标定，先查明雷达原始轴与物理轴的映射，不能靠猜测翻转 `dz_m` 符号。

## 2. 冻结匹配帧并保存独立样本

1. 确认相机内参已通过独立流程取得并记录内参 RMS（现场模板要求 ≤1.5 px）；有效焦距、畸变参数和 `image_size` 与网页导出的原始相机尺寸一致。确认网页中的相机帧已载入且同步状态为 `matched`；接收时钟匹配不等于硬件触发同步。
2. 在网页“人工标定”工作台点击“冻结匹配帧”。在冻结的 PPI 中点选 5 cm 角反射器对应的**原始雷达点**，核对 `x/y/z`、帧号和原始点索引；再点击同一冻结相机图像中**角反射器的相机像素中心**。不要用聚类中心或后续实时帧代替这对点。
3. 选 `fit`（拟合样本）或 `validation`（验证样本），核对两侧点和组别后点击“保存样本”。点击“继续实时画面”，移动并静置角反，在不同位置重新冻结匹配帧。可删除误选的末条样本并重新采集。
4. 建议采集 **12–20 个 fit** 样本，覆盖画面左/中/右及近/中/远；另采 **6–10 个 validation** 样本，必须来自**不同位置**，不能复用拟合位置。求解器最低要求 6 个 fit 和 1 个 validation，但最低数量不是现场覆盖度验收标准。
5. 检查样本表、雷达和相机帧 ID、同步偏差、安装测量，然后点击“下载 JSON”。保存原始会话和截图到 Git 仓库之外，记录会话路径与 SHA-256；原始会话不触发服务重载。

## 3. 在 PC 上求解并审阅候选报告

把导出的会话 JSON 和**单独取得**的相机内参 JSON 放在 PC 的证据目录。下面以已有的 `C:\calibration-case` 目录为例；先把三个路径改成现场实际路径，`$output` 应与两个输入及其衍生报告路径互不相同。OpenCV 和 NumPy 只需装在本地 PC 环境，Pi 运行时不需要它们。

```powershell
$session = 'C:\calibration-case\session.json'
$intrinsics = 'C:\calibration-case\intrinsics.json'
$output = 'C:\calibration-case\calibration_candidate.json'
.\.venv\Scripts\python.exe tools\fusion\calibrate_radar_camera.py $session $intrinsics $output --mount-mode co_rotating
$solverExit = $LASTEXITCODE
$report = 'C:\calibration-case\calibration_candidate.report.json'
Get-FileHash -Algorithm SHA256 $session
Get-FileHash -Algorithm SHA256 $intrinsics
if (Test-Path $report) {
    Get-FileHash -Algorithm SHA256 $report
    Get-Content -Raw -Encoding UTF8 $report | ConvertFrom-Json
}
if (Test-Path $output) { Get-FileHash -Algorithm SHA256 $output }
```

`co_rotating` 只适用于雷达和相机随同一刚性支架一起转动的安装；实际安装不同，应先核对服务支持的安装模式。求解器只用 `fit` 组调用 `solvePnP`，单独投影 `validation` 组。成功求解后，无论独立误差是否过门，都会写出 `$output` 的 `.report.json` 同名报告，即本例的 `calibration_candidate.report.json`。如果输入无效或输出/报告路径与输入或彼此指向同一文件，求解会提前拒绝，此时可能没有候选报告。

检查 `$solverExit`、报告中的样本数、`fit.rms_px`、`validation.median_px`、`validation.p95_px` 和 `validation.max_px`。**只有** `validation.median_px ≤ 8 px` 且 `validation.p95_px ≤ 20 px`，求解器才写出可供进一步审查的运行时 `$output` JSON；未通过时退出非零、保留候选报告并移除该安全独立输出路径上的旧运行时 JSON。不要把 `fit` 误差或浏览器模拟结果代替独立验证，也不要放宽门限掩盖错误关联。报告及运行时 JSON 的路径、SHA-256、求解器提交号和退出码都写入验收记录。

报告的 `camera_center_in_radar_m = -R^T t` 是**相机光心在雷达坐标系中的求解位置**，可与测得的 `dx_m/dy_m/dz_m` 比较；`mount_comparison.residual_m` 是“求解值减测量值”，未提供安装测量时为 `null`。运行时 `radar_to_camera.translation_m` 在**相机坐标系**中，不能直接当成安装测量。机械残差只用于复核，不是替代独立像素误差的自动门限。若残差或像素误差异常，复查原始 Z 证明、内参分辨率、目标关联、刚性安装和验证点覆盖。

## 4. 现场验收与手动加载边界

本地单元测试和浏览器模拟只证明软件流程；它们不能证明真实 Pi、雷达、相机的同步、物理轴向、安装测量或叠加精度。现场操作员须按[验收模板](validation/radar-camera-template.md)保存真实采样和 Z 方向证据，人工复核独立报告与机械残差，并确认两个验证门限均通过。只有这些门禁通过后，才按[部署检查表](../deploy/DEPLOYMENT_CHECKLIST.md)由操作员手动复制已审核的标定 JSON、在服务参数中指定 `--camera-calibration` 并验证加载；记录部署提交、部署校验和、服务状态及回退依据。未通过或缺少证据时保持投影未加载。
