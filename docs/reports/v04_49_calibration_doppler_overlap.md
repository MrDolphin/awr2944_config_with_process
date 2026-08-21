# V0.4.49 校准接口与目标-海杂波 Doppler 重叠

## 本阶段目的

本阶段同时完成两个工作：

1. 建立功率校准契约，明确哪些字段必须由 DCA1000 已知目标采集提供，避免把合成谱单位直接解释成 dBm；
2. 将 20 m、方位 10°、俯仰 2° 的目标速度设置为 `-0.6/-0.3/0/0.3/0.6/1.0 m/s`，观察目标 Doppler 与海杂波 Doppler 接近时的检测变化。

输出中的校准状态为 `awaiting_dca1000_reference_capture`，说明当前仍没有实测参考功率。

## 如何读取结果

- `mean_clutter_to_training_db`：未注入目标时，目标 Doppler 单元海杂波功率相对 CFAR 训练噪声的平均值；越高表示海杂波越接近 CFAR 判决区域。
- `detection_probability`：注入目标后，41 帧中目标单元 ±2 个网格被命中的比例。
- `mean_false_alarms_per_frame`：扣除目标命中后的平均虚警点数。
- `calibration_schema.json`：功率量纲和后续实测数据的入口。

## 主要结果

在 `Pfa=10^-2` 下，正常海况 `ss2_normal` 的 5 dB 目标检测概率为：

- `-0.6 m/s`：`0`
- `-0.3 m/s`：`0.0976`
- `0.0 m/s`：`0.6829`
- `0.3 m/s`：`0.0244`
- `0.6 m/s`：`0`
- `1.0 m/s`：`0`

三级海况上限 `ss3_upper` 的 5 dB 目标检测概率为：

- `-0.6 m/s`：`0.0244`
- `-0.3 m/s`：`0.0244`
- `0.0 m/s`：`0.1951`
- `0.3 m/s`：`0.0488`
- `0.6 m/s`：`0`
- `1.0 m/s`：`0`

在 10 dB 和 15 dB 下，所有速度点检测概率均为 `1.0`，说明当前目标仍有足够 SNR 裕量；在 5 dB 附近，速度接近海杂波集中区域时，检测概率出现明显变化。

未注入目标时，0 m/s 附近的海杂波相对训练噪声最高：

- `ss2_normal`：约 `+1.52 dB`；
- `ss3_upper`：约 `-3.39 dB`。

三级海况上限的额外虚警仍约为 `0.268/帧 (Pfa=10^-2)` 和 `0.146/帧 (Pfa=10^-3)`。

## 这说明什么

当前模型支持一个有价值的趋势判断：

> 目标速度落在海杂波 Doppler 集中区域附近时，低 SNR 目标的检测概率会下降；三级海况还会额外增加虚警点。目标 SNR 提高到 10 dB 后，本次模型中的速度重叠影响被裕量掩盖。

这不是实测结论，因为海面 Doppler 仍由 V0.2 高度场差分得到，海杂波绝对功率和目标功率还未用 DCA1000 标定。

## 功率校准需要什么

`calibration_schema.json` 已定义后续需要的证据：

- 未经 CFAR 的 DCA1000 原始 ADC/IQ 文件；
- 实际使用的 chirp/profile CFG；
- 已知目标距离；
- 已知目标 RCS；
- 距离窗、Doppler 窗和 FFT 长度定义；
- 参考距离处的复数谱功率或 ADC RMS。

拿到这些数据后，才能把当前合成谱的 `power_linear` 变换成可与实测比较的相对 ADC 功率或 dB 标度。

## 输出文件

- [doppler_overlap_sweep.csv](../../simulation/hardware/awr2944pev/v04_49_calibration_doppler_overlap/doppler_overlap_sweep.csv)
- [calibration_schema.json](../../simulation/hardware/awr2944pev/v04_49_calibration_doppler_overlap/calibration_schema.json)
- [summary.json](../../simulation/hardware/awr2944pev/v04_49_calibration_doppler_overlap/summary.json)
- [output_analysis.md](../../simulation/hardware/awr2944pev/v04_49_calibration_doppler_overlap/output_analysis.md)

