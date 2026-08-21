# V0.4.46 海杂波背景下已知目标检测分析

## 本阶段目的

在 V0.4.43 生成的合成复数距离-Doppler 海杂波谱中，加入一个已知目标，检查现有 CA-CFAR 和稀疏阵列 AoA 链路是否能把目标从海杂波背景中检出。目标真值为：方位 `10°`、俯仰 `2°`、距离 `20 m`、径向速度 `1 m/s`。由于离散网格限制，实际落入 `19.3226 m`、`0.9125 m/s` 单元。

扫描变量为目标 SNR `10/15/20 dB` 和 CA-CFAR 虚警概率 `Pfa=10^-2/10^-3`；训练窗为 `(2,2)`、保护窗为 `(1,1)`，每个海况包含 41 帧。

## 结果如何读取

- `detection_probability`：41 帧中，目标距离-Doppler 单元周围 ±1 个单元内出现 CFAR 点的帧比例。
- `mean_false_alarms_per_frame`：每帧检测点中扣除目标命中后的平均点数；它不是严格意义上的概率虚警率。
- `target_azimuth_rmse_deg`、`target_elevation_rmse_deg`：仅在目标命中的帧上计算 AoA 误差的 RMSE。RMSE 是“均方根误差”，越小越好。

## 主要观察

1. 本次受控目标在五种海况、两种 Pfa 和三个 SNR 下检测概率均为 `1.0`，说明“目标注入→CA-CFAR→目标命中统计”的软件接口闭环已经打通。
2. 平静至正常海况 `ss0/ss1/ss2` 的平均虚警点为 `0`；三级海况名义/上限 `ss3_nominal/ss3_upper` 出现少量海杂波虚警。`ss3_upper` 的虚警点约为 `0.268/帧 (Pfa=10^-2)` 和 `0.146/帧 (Pfa=10^-3)`，高于 `ss3_nominal` 的 `0.122/帧` 和 `0.098/帧`。
3. 本次目标直接按指定阵列导向矢量写入目标单元，因此命中帧的 AoA RMSE 为 `0°` 是受控注入的理论回归结果，不代表 AWR2944P 实测角度精度。
4. SNR 从 10 dB 提高到 20 dB 没有改变检测概率，原因是当前目标已经远高于局部 CFAR 阈值；这不是“任何海况下 10 dB 都能达到 100% 检测”的结论。

## 工程结论

当前结果可以向领导汇报为：目标检测评估链路已经具备，三级海况上限 1 m 的合成海面会增加虚警点，且 Pfa 从 `10^-2` 降到 `10^-3` 可降低虚警；但当前模型仍是合成谱，PCB 阵列使用候选铜区质心，CFAR/AoA 为独立实现，尚未接入 TI SDK 的真实通道校准、实测方向图和 DCA1000 实测 IQ。因此结果只能作为算法链路回归和参数趋势参考，不能作为实测探测距离、检测概率或虚警率指标。

## 输出与复现

- 原始结果：`simulation/hardware/awr2944pev/v04_46_target_in_sea_clutter/target_detection_sweep.csv`
- 汇总：`simulation/hardware/awr2944pev/v04_46_target_in_sea_clutter/summary.json`
- 自动分析：`simulation/hardware/awr2944pev/v04_46_target_in_sea_clutter/output_analysis.md`
- 运行命令：

```powershell
python -m simulation.run_v04_46_target_in_sea_clutter `
  --input-root simulation/hardware/awr2944pev/v04_43_sea_range_doppler `
  --output simulation/hardware/awr2944pev/v04_46_target_in_sea_clutter
```

