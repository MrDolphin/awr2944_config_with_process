# V0.4.79 DCA1000/LVDS 原始 IQ 接入契约

输入：`D:\hp-laptop\USV\awr2944_sea_clutter_v02\simulation\hardware\awr2944pev\v04_80_end_to_end_replay\end_to_end_replay.h5`

状态：`valid_contract`

数据类型：`raw_adc_iq`；形状：`[16, 64, 4]`；数据集：`/radar/raw_iq`。

## 必须记录的采集元数据

- ADC 原始类型和端序；
- LVDS wire order；
- RX 数量、样本数、chirp 数；
- TDM TX 序列和 frame 边界；
- CFG 文件快照；
- 通道顺序是否已验证；
- TI 校准状态、校准命令和幅相矩阵；
- 时间戳、船体姿态和安装角度。

当前缺失元数据：`[]`。

契约验证通过不代表 AoA 已经经过硬件验证；`channel_order_verified=false` 时，数据只能进入排列敏感性分析，不能作为实板角度精度结论。
