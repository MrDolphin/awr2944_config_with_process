# 离线 TLV2 强度 PPI：第一阶段验证

本工具只读现有采集，不启动雷达或电机，也不修改 CFG。它先用逐帧主机时间与 QD4310 编码器样本配准方位，再把每帧 TLV2 距离剖面的原始强度画成一条方位线。图中的空白扇区保持空白，不能用插值伪装成已扫描区域。

## 对已有旋转采集运行

在项目根目录执行（下面的 `-1` 是本次采集实际验证的编码器递减趋势，不应直接套用于另一台电机）：

```powershell
.\.venv\Scripts\python.exe tools/register_scan_capture.py `
  'docs/linux/capture_20261029_123620递增方向-1' `
  'docs/linux/capture_20261029_123620递增方向-1/motor_samples.jsonl' `
  'docs/validation/artifacts/offline_ppi_20261029_123620_registered' `
  --angle-direction decreasing --zero-deg 0 --mounting-yaw-deg 0 --mounting-roll-deg 90

.\.venv\Scripts\python.exe tools/render_polar_range_ppi.py `
  'docs/validation/artifacts/offline_ppi_20261029_123620_registered/polar_range_profiles.jsonl' `
  'docs/validation/artifacts/offline_ppi_20261029_123620_render'
```

两个输出目录必须不存在；工具拒绝覆盖旧结果。若需重跑，换新的输出目录名。`--mounting-roll-deg 90` 只是本次安装姿态的临时输入，不能凭这张图证明绝对方位；零位 `0` 也尚未做外部基准校准。其他采集应根据自身安装与编码器趋势填写。

输出文件：

- `*_registered/registered_frames.jsonl`：逐帧角度配准结果，包含未配准原因。
- `*_registered/polar_range_profiles.jsonl`：通过配准的 TLV2 强度方位线。
- `*_registered/summary.json`：原始帧数、通过/拒绝数及拒绝原因。
- `*_render/intensity_ppi.svg`：浏览器可打开的诊断图。
- `*_render/summary.json`：方位线数、每 10° 覆盖、最大方位缺口、输入哈希和证据边界。

## 本轮判读与后续门槛

2026-10-29 12:36:20 的采集共有 52 帧；48 帧配准、4 帧拒绝（`unbracketed` 1、`angular_span` 2、`sample_gap` 1）。48 条 TLV2 方位线，每条 512 个原始强度格；观测到 24/36 个 10° 方位格，最大相邻方位线间隙 18.719°。原始输入未改动，默认跳过的近距格数为 0；没有依据 TLV7 SNR 或当前速度设置过滤门限。

此图的距离轴仍是 TLV2 格号，颜色是本次数据的线性百分位显示，不是米、dB、RCS 或 CFAR 检测概率；时间是主机解析数据包的时间，不是校准过的 RF 采集时间。近距强度可能遮蔽远处结构，参数 `--skip-bins N` 只做显示遮罩，必须同时保存未遮罩基线图，并在结果中注明 N。不能依据此图断言真实 360° 物理覆盖、距离或旋转多普勒补偿已经通过。

2026-10-31 更新：原先“对已较高的原始值再取对数”的画法会把低、高强度都压到亮绿色。现在显示色阶使用当前数据的第 5/99 百分位做线性拉伸，低端为暗绿、高端为亮绿；`summary.json` 记录 `display_lower_raw`、`display_upper_raw` 和 `display_transform`。这是**显示变换**，没有修改、删除或按 SNR/速度筛选任何原始距离格。原始采集和旧版渲染结果仍保留，可对照查看。

## 停转采集的正确验证图

`capture_20261030_144620tlv2验证` 的 28 帧有 TLV2、但 `motor_samples=0`，不能产生有证据支撑的 360° PPI。用下列命令生成“距离格 × 帧”的强度热图，专门验证目标峰与显示对比度：

```powershell
.\.venv\Scripts\python.exe tools/render_tlv2_range_time.py `
  'docs/linux/capture_20261030_144620tlv2验证' `
  'docs/validation/artifacts/tlv2_20261030_144620_contrast'
```

输出的 `range_time_contrast.svg` 横轴是未标定距离格号、纵轴是帧，`summary.json` 标明强度色阶和 `ppi_generated=false`。这组数据的 14,336 个 TLV2 原始格全部非零；第 5/99 百分位分别为 2619/4289，旧画法会把它们映为几乎一样的亮绿，新画法则映为暗绿/亮绿。热图能显示哪些距离格持续偏强，但未标定距离轴和目标位置前，不能仅凭绿色亮带认定某处就是 2 m 角反。

天线约 6° 的水平 3 dB 波束宽度是物理照射范围，不是把编码器角度相差 6° 的帧直接合并为一个点的规则。显示可以另画“波束足迹”参考层，但不能把其内每个角度都当作独立测量，也不能用线宽填补未采样扇区。

下一轮需要在稳定电源、固定标靶及现场记录条件下，核对角度零位/方向、帧时间、距离格宽，并做有/无目标对照。若要形成类似商用绿色连续回波 PPI，必须提高有效方位采样密度和完成时空标定；不能通过把稀疏方位线涂满来代替测量。
