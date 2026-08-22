# V0.4.133 DCA1000 真实采集准入门

扫描目录：`D:\hp-laptop\USV\awr2944_sea_clutter_v02\simulation\hardware\awr2944pev\v04_113_cfg_linked_capture_scaffold`；manifest 数量：30；准入数量：0。

## 准入条件

每个 manifest 必须同时满足：capture.bin 存在；目标方位/俯仰/距离真值完整；CFG 文件和哈希已绑定；LVDS wire order 和通道顺序已验证；TI 校准状态为 measured/verified；采集绑定姿态或明确记录无 IMU。

## 当前结论

文件存在或尺寸匹配不等于 ADC IQ 已验证。只有 `ready=true` 的 manifest 才能进入真实 IQ 解码和 AoA；当前输出明确保守地保持 `real_dca_iq_admitted=false`。

## 下一次采集要求

1. 保存原始 capture.bin，不经过 CFAR。
2. 保存采集时使用的 CFG 和 DCA1000 配置。
3. 用角反射器记录已知方位、俯仰和距离。
4. 记录天线安装角度、雷达高度、船体姿态和时间戳。
5. 完成 TI 校准并保存校准结果文件。
