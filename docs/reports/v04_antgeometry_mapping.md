# V0.4.6 antGeometryCfg 与 PCB 网络映射

TI SDK 本机源码和 Doxygen 对 `antGeometryCfg` 的语义说明为：16 组参数按
`Tx0Rx0, Tx0Rx1, ..., Tx3Rx3` 排列；每组先是 row（俯仰/elevation 行），再是 column
（方位/azimuth 列）；末尾两个值是方位和俯仰单元间距（单位 λ）。本项目配置解析为：

```text
azimuth cell spacing = 0.5 λ
elevation cell spacing = 0.8 λ
```

展开后的 16 行映射保存在：

```text
simulation/hardware/awr2944pev/antgeometry_mapping.csv
```

其中 `TX1/RX1` 对应 SDK 的 `Tx0/Rx0`，依次类推；这只是配置索引映射，不代表 PCB 铜区中心就是该通道的电气相位中心。

## 重要发现

`antGeometryCfg` 描述的是 SDK AoA 处理所需的虚拟阵列行列索引，不是直接的 PCB X/Y 坐标。当前配置的行列索引与 PCB-derived 铜区坐标不能直接替换：必须同时保留 SDK 的行列重排和 PCB 的几何坐标，不能只把 PCB 坐标表按 CSV 顺序塞入 AoA 算法。

本阶段完成了“配置索引 → TX/RX 网络名”的映射，但真实 AoA 模型仍需：

- 确认 PCB 天线编号与 SDK TX/RX 编号的方向一致性；
- 将 `antGeometryCfg` 的行列索引映射到阵列响应矩阵；
- 通过角反射器校准或全波仿真得到电气相位中心；
- 用 DCA1000 IQ 验证通道顺序和相位符号。
