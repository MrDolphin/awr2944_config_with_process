# V0.2 动态海面图片与运行批次对比记录

## 结论先行

本记录比较的是 V0.2 动态海面真值阶段的图片和运行摘要，不是 AWR2944P 的 IQ、海杂波功率、AoA 或 CFAR 结果。

当前主基线为 `v02b_forward_seed101_hs1m`：MATLAB `WindDirection=90°`，映射为船体坐标中的船首方向 `+y/0°`；三级海况项目输入上限为 `Hs=1.00 m`。MATLAB 运行目录保存原始 HDF5，Python 运行目录读取同一批 HDF5 并生成图片与派生几何指标。

## MATLAB 风向在图中如何识别

MATLAB `WindDirection` 从北方向顺时针计角。当前约定为：

```text
vessel_direction_deg = wrapTo180(90 - matlab_wind_direction_deg)
```

因此：

| MATLAB `WindDirection` | 船体等效波向 | 图中解释 |
|---:|---:|---|
| 90° | 0° | 船首 `+y`，当前主基线 |
| 0° | 90° | 右舷 `+x` |
| 180° | -90° | 左舷 `-x` |
| 270° | 180° | 船尾 `-y` |

Python 生成的 overview 图和 comparison 图标题会显示：

```text
MATLAB WindDirection=90.0 deg
vessel wave direction=0.0 deg (+y forward)
```

图中 `x right (m)` 和 `y forward (m)` 是空间坐标；海面条纹的主方向只能作为视觉提示，最终方向应以 `summary.json` 中的 `dominant_wave_direction_deg` 为准。单个有限随机海面的谱峰方向是诊断量，不应直接当作全部波能的真实平均方向。

## 图片文件的层级

### 单工况 overview 图

每个 `ss*_..._overview.png` 对应一个海况输入：

| 图片 | 目标 Hs | 用途 |
|---|---:|---|
| `ss0_flat_seed101_overview.png` | 0.00 m | 平面海面回归基准 |
| `ss1_rippled_seed101_overview.png` | 0.05 m | 轻微波纹 |
| `ss2_normal_seed101_overview.png` | 0.30 m | 日常小浪基准 |
| `ss3_nominal_seed101_overview.png` | 0.85 m | 三级典型 |
| `ss3_upper_seed101_overview.png` | 1.00 m | 项目三级上限 |

每张 overview 图包含四类信息：`t=0` 海面高度、`t=0` 局部表面掠射角、20 s 高度包络、全时空高度分布及目标/实际 Hs。它用于解释单个工况，不是雷达点云图。

### `sea_state_comparison.png`

这是五个海况的汇总图，四个子图分别比较：目标/实际 Hs、最小雷达净空、逐网格掠射角时间标准差空间均值、斜距变化率绝对值 P95。它最适合说明海况从 0 级增大到三级上限时的总体趋势。

## 运行批次对比

| 批次 | 主要输入差异 | 输出用途 | 是否可与当前主基线直接比较 |
|---|---|---|---|
| `v02_quick_seed101` | 早期快速基线，历史版本曾含 `Hs=1.20 m` | 流程回归 | 只能作历史参考，不能与 1.00 m 上限直接做数值结论 |
| `v02b_quick_seed101_hs1m` | `Hs` 上限改为 1.00 m，快速方向配置 | 上限约束回归 | 可比较海况趋势；比较波向时需确认方向配置 |
| `v02b_forward_seed101_hs1m` | `WindDirection=90°`，船首方向 `0°` | 当前主基线 | 是 |
| `v02b_forward_seed101_hs1m_analysis` | 读取上述 MATLAB HDF5 的 Python 分析 | 生成 overview、comparison、summary | 是，同一物理运行的后处理 |
| `*_analysis_v2` / `*_verified` | 同一类输入的重跑或验证批次 | 检查绘图/解析修订 | 只有确认 `run_config.json` 和输入 HDF5 一致后才可合并比较 |

## 主基线 seed101 数值对比

数据来源：`baselines/v02b_forward_seed101_hs1m/summary.json`。

| 工况 | 目标 Hs (m) | 实际 Hs (m) | 最小净空 (m) | 掠射角时间标准差均值 (°) | 斜距变化率 P95 (m/s) |
|---|---:|---:|---:|---:|---:|
| `ss0_flat` | 0.00 | 0.000000 | 1.000 | 0.000 | 0.0000 |
| `ss1_rippled` | 0.05 | 0.050000 | 0.946 | 0.554 | 0.0011 |
| `ss2_normal` | 0.30 | 0.300000 | 0.659 | 2.959 | 0.0076 |
| `ss3_nominal` | 0.85 | 0.849999 | 0.011 | 6.833 | 0.0219 |
| `ss3_upper` | 1.00 | 0.999999 | -0.164 | 7.978 | 0.0259 |

## 如何做四类比较

### 1. 同一方向、同一种子、不同海况

比较 `v02b_forward_seed101_hs1m_analysis` 的五张 overview 图和 `sea_state_comparison.png`。这隔离了 `Hs/sea_state` 的影响，重点观察海面起伏、局部法向、掠射角波动、净空和斜距变化率。

### 2. 同一海况、同一种子、不同风向

例如分别取两个批次中的 `ss3_upper_seed101_overview.png`。必须先确认除了 `WindDirection` 外，Hs、网格、雷达高度、安装姿态和随机种子一致。重点比较波纹方向、主导波向、局部坡度相对雷达视线的投影以及几何指标。

### 3. 同一输入、不同随机种子

比较 `seed101/202/303/404/505` 的相同 `case_id`。单种子图反映一个随机实现；多种子才能报告均值、标准差和分位数。当前 seed101 结果不能单独作为所有随机海面的统计结论。

### 4. MATLAB 与 Python

MATLAB 是原始动态海面 HDF5 生成阶段；Python 是读取同一 HDF5 后的几何分析和绘图阶段。二者不是两组独立物理实验。MATLAB `figures/` 目前为空是因为 `run_v02.m` 只创建该目录并写入 HDF5/摘要，未实现 MATLAB 绘图；当前可查看的正式图形由 Python 分析器生成。只要 Python 输入目录指向对应 MATLAB `data/`，就不会丢失原始海面数据。

## 当前可以汇报的结论

在雷达高度 1 m、安装俯仰角 5°、船首方向、随机种子 101 的主基线中，目标 Hs 从 0 增加到 1.00 m 时，实际 Hs 闭环保持一致；局部表面掠射角时间波动从约 0° 增加到约 7.98°；最小净空从 1.00 m 降至约 -0.164 m。说明低安装高度下，三级海况上限首先带来明显的动态照射几何和波峰净空风险。

这仍然是动态海面真值/几何层结论，不是 AWR2944P 海杂波功率、AoA 精度、CFAR 虚警率或三维点云性能结论。

## 图形查看路径

当前主基线的 Python 图片位于：

`simulation/stages/v02_dynamic_sea_truth/results/python/v02b_forward_seed101_hs1m_analysis/figures/`

其中 `sea_state_comparison.png` 适合汇报总览，`ss3_nominal_seed101_overview.png` 和 `ss3_upper_seed101_overview.png` 适合说明三级海况和上限工况。
