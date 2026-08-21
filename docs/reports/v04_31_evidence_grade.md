# V0.4.31 硬件阵列证据等级与校准缺口

## 目的

把前三个阶段的结果统一到每个 TX/RX 虚拟通道：

```text
PCB RF 铜区
    + antGeometryCfg 行列
    + 原理图 PDF 的 50 ohm GCPW 到天线文字
    ↓
每个虚拟通道的证据等级
```

## 实际结果

```text
虚拟通道：16
PCB RF 区域：8
原理图 GCPW 到天线文字支持：是
真实阵列已确认：否
```

16 个通道目前全部是：

```text
overall_status = candidate_only
```

但每个通道已经具有：

- PCB RF 区域提取证据；
- TX/RX 网络名称；
- `antGeometryCfg` 的 row/column；
- 原理图 PDF 第 2 页的 GCPW-to-antenna 文字证据；
- 当前缺失项和后续校准要求。

## 关键缺口

| 缺口 | 当前状态 | 影响 |
|---|---|---|
| 电气相位中心 | 缺失 | 阻止真实阵列坐标声明 |
| 芯片引脚到天线馈点连通 | 候选 | 阻止最终 TX/RX 映射 |
| PCB 到雷达坐标基准 | 候选 | 阻止最终镜像/法向确认 |
| TX/RX 顺序、I/Q、LVDS 语义 | 缺失 | 阻止真实 AoA 通道解释 |
| 复数幅相校准 | 缺失 | 阻止真实角度精度声明 |
| 天线方向图/互耦 | 缺失 | 不阻止理想几何仿真，但影响硬件逼真度 |

## 校准输入契约

机器可读文件：

```text
simulation/hardware/awr2944pev/v04_31_evidence_grade/calibration_input_contract.json
```

接受的后续输入包括：

1. Altium ASCII SchDoc、网表或引脚级导出；
2. 已知角度角反射器的 DCA1000 原始 `.bin`；
3. TX/RX 实测幅相校准数据；
4. 机械安装坐标基准或装配测量。

## 结论

当前结果足以支持“候选阵列几何敏感性仿真”和“下一步校准测试设计”，但不足以支持“AWR2944P 真实相位中心坐标”或“实测 AoA 精度”的结论。
