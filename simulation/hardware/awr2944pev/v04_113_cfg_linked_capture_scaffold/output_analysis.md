# V0.4.113 CFG 绑定的 30 工况已知角采集骨架

共 30 个计划工况；方位 `[-30, -15, 0, 15, 30]°`，俯仰 `[-10, 0, 10]°`，距离 `[10, 20] m`。

当前 CFG：`D:\hp-laptop\USV\awr2944_sea_clutter_v02\Config\profile_3d_3Azim_1ElevTx_awr2944P.cfg`
CFG SHA-256：`5c605926277ab9f67bb3a11656b0640f37c49d455a709d40715b6200bf8d2027`
ADC samples/chirp：`656`；chirps/frame：`64`；TDM TX：`[0, 2, 3, 1]`。

## 每个案例目录

每个目录都包含与当前 CFG 同步的 `manifest.json`、CFG 快照和 README；真实采集后再放入 `capture.bin`。所有 manifest 初始状态均为 `planned_awaiting_capture`。

## 采集顺序建议

先完成距离 10 m、俯仰 0° 的五个方位点，再完成方位 0° 的三个俯仰点，最后完成距离 20 m 的重复点。每个案例必须保存完整 TDM 原始帧，不要只保存 CFAR 点云。

## 证据边界

这是采集骨架，不是测量结果。只有真实 ADC IQ、目标测量、姿态、通道顺序和 TI 校准证据齐全后，才能进入 V0.4.112 和 AoA 验证。
