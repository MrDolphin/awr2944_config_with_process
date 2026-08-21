# V0.4.57 仿真结果汇报图

## 本阶段目的

把 V0.48、V0.49 和 V0.54 的 CSV 结果生成三张可以直接放入汇报材料的图片，并在同一目录保存图片、输入路径和解释文档。

## 图片与含义

### 1. Doppler 重叠检测图

[doppler_overlap_detection.png](../../simulation/hardware/awr2944pev/v04_57_report_figures/doppler_overlap_detection.png)

- 横轴：目标径向速度；
- 纵轴：目标检测概率；
- 条件：目标 SNR=5 dB、Pfa=10^-2；
- 曲线：`ss2_normal` 与 `ss3_upper`。

读图时重点看 0 m/s 附近：目标速度接近海杂波集中区域时，低 SNR 检测概率会出现变化。这反映的是当前合成海杂波模型中的趋势，不是实测检测概率。

### 2. 距离检测边界图

[physical_range_detection.png](../../simulation/hardware/awr2944pev/v04_57_report_figures/physical_range_detection.png)

- 横轴：目标距离；
- 纵轴：检测概率；
- 每条曲线：一个海况和一个目标 SNR；
- 模型：相对 `σ/R⁴` 目标功率衰减。

读图时看曲线从 1 降到 0 的位置，它是当前模型的候选距离边界。不能直接将它称为 AWR2944P 的最大探测距离，因为绝对功率尚未用 DCA1000 实测标定。

### 3. 通道候选 RMSE 排名图

[channel_order_rmse_rank.png](../../simulation/hardware/awr2944_sea_clutter_v02/simulation/hardware/awr2944pev/v04_57_report_figures/channel_order_rmse_rank.png)

- 横轴：RX/TX 排列候选；
- 纵轴：多场景联合 AoA RMSE；
- 蓝色：身份排列；
- 橙色：错误排列候选。

这个图用于说明为什么需要多角度、多距离已知目标采集来确认通道顺序。某个错误排列偶然靠近真值，并不能证明它是正确硬件顺序。

## 汇报时可使用的结论

1. 低 SNR 目标在海杂波 Doppler 集中区域附近更容易受到影响；
2. 相对 `σ/R⁴` 模型显示距离增加会显著降低目标检测概率；
3. RX/TX 通道顺序对 AoA 误差敏感，必须用多场景实测确认；
4. 当前图片来自合成/相对模型，不能替代实测探测距离、真实虚警率或真实 AoA 精度。

## 输出文件

- [summary.json](../../simulation/hardware/awr2944pev/v04_57_report_figures/summary.json)
- [output_analysis.md](../../simulation/hardware/awr2944pev/v04_57_report_figures/output_analysis.md)
