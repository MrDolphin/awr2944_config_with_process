# V0.4.120 波束掩膜海面微元杂波代理

本阶段复用 V0.42 的海面法向、入射余弦、径向速度、Doppler 和 `incidence²/range²` 散射代理，再套用 V0.119 的 3 dB/6 dB 几何波束掩膜。

## 字段含义

`scatter_proxy` 是相对散射权重代理；`radial_velocity_mps` 和 `doppler_hz` 来自 V0.2 高度场时间差分；`masked_scatter_3db/6db` 只保留落入对应几何窗口的权重。

## 结果边界

这是微元海杂波候选的几何/物理代理，不是实测雷达点云或功率谱。没有真实海面散射系数、水平流速、极化、天线方向图、通道校准、噪声、CFAR 和 DCA1000 IQ。
