# V0.4.111 与 CFG 绑定的已知角采集清单

CFG：`D:\hp-laptop\USV\awr2944_sea_clutter_v02\Config\profile_3d_3Azim_1ElevTx_awr2944P.cfg`
CFG SHA-256：`5c605926277ab9f67bb3a11656b0640f37c49d455a709d40715b6200bf8d2027`

## 从 CFG 自动提取的几何

- ADC samples/chirp：`656`
- RX：`4`
- chirps/frame：`64`
- TDM TX 顺序（按 chirp mask bit）：`[0, 2, 3, 1]`

## 使用限制

这只是与当前 CFG 一致的采集模板，尚未包含真实 capture.bin、通道顺序实测、TI 校准矩阵、目标测量和船体姿态。因此不能直接进入硬件 AoA 精度结论。相比旧 V0.4.95 模板，本模板不再硬编码 128 samples/64 chirps，而是从实际 CFG 读取。

## 下一步

将模板复制到一个具体已知角案例目录，放入原始 capture.bin 和同一份 profile.cfg，填写目标真值、安装姿态、时间戳、IMU 参考、wire order 与校准记录，再通过 V0.4.94/V0.4.90 完整性检查。
