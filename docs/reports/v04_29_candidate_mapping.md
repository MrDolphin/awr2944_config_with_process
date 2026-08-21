# V0.4.29 PCB RF 候选通道与 `antGeometryCfg` 交叉映射

## 输入

- `simulation/hardware/awr2944pev/antgeometry_mapping.csv`：当前 CFG 展开的 16 个虚拟通道；
- `simulation/hardware/awr2944pev/pcb_antenna_regions.csv`：从 ASCII PCB 识别出的 8 个 TX/RX 铜区；
- `antGeometryCfg`：4TX×4RX 的行列和间距语义。

## 实际结果

8 个 PCB RF 网络：

```text
TX1 TX2 TX3 TX4
RX1 RX2 RX3 RX4
```

可以组成 16 个候选 TX/RX 虚拟通道。每个候选通道都包含：

- CFG 的 row/column；
- TX 铜区质心坐标；
- RX 铜区质心坐标；
- 来源和证据说明；
- `phase_center_confirmed=false`。

去除 PCB 与 CFG 的任意平移后，四种简单坐标解释的形状筛查结果为：

| 变换 | 归一化 RMS 误差(mm) | X RMS(mm) | Y RMS(mm) |
|---|---:|---:|---:|
| identity | 1.287830 | 1.505346 | 1.025158 |
| mirror_x | 3.379026 | 4.667407 | 1.025158 |
| mirror_y | 1.090072 | 1.505346 | 0.332337 |
| rotate_180 | 3.308711 | 4.667407 | 0.332337 |

在当前“铜区质心 + 理想 CFG 网格”的近似模型下，`mirror_y` 误差最小。它只能说明相对几何形状更接近，不能证明 PCB 阵面实际应该沿 Y 镜像，也不能证明真实 AoA 会采用这个变换。

## 输出

结果目录：

```text
simulation/hardware/awr2944pev/v04_29_candidate_mapping/
```

- `channel_mapping_candidates.csv`：16 个候选虚拟通道；
- `coordinate_transform_candidates.csv`：四种坐标变换的形状筛查；
- `mapping_summary.json`：机器可读汇总；
- `output_analysis.md`：本阶段分析边界。

## 结论边界

本阶段已经完成“网络名称—PCB 铜区—CFG 虚拟通道”的可追溯连接，但仍未确认：

1. 铜区质心是否等于天线电气相位中心；
2. TX/RX 网络名与芯片实际通道编号是否完全一致；
3. PCB 坐标原点、阵面法向和雷达坐标系方向；
4. 通道幅相、I/Q 符号和真实校准矩阵；
5. 真实天线方向图和 AoA 误差。

因此该结果可以作为“候选阵列几何输入”和下一步实测设计的依据，但不能作为真实 EVM 阵列坐标的最终证明。
