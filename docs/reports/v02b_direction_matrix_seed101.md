# V0.2-B 方向矩阵调试记录（seed101）

## 运行范围

固定 `ss2_normal`、`ss3_upper` 和随机种子 101，仅改变 MATLAB `seaSurface.WindDirection`：

| MATLAB 方向 | 运行目录 | 状态 |
|---:|---|---|
| 0° | `matlab/v02b_dir000_seed101` | 已生成并完成 Python 分析 |
| 90° | `matlab/v02b_dir090_seed101` | 已生成并完成 Python 分析 |
| 180° | `matlab/v02b_dir180_seed101` | 已生成并完成 Python 分析 |
| 270° | 未生成 | MATLAB `seaSurface` 明确限制 `WindDirection <= 180°` |

MATLAB 方向属性在非地心场景下按 North 顺时针计角；但该接口的 `0..180°` 限制以及方向扩展海谱的对称性意味着，不能仅用 0/90/180/270°四个数直接证明“风来自/波传播到”的语义。

## 合成行波方向回归

在不依赖 MATLAB `WindDirection` 的情况下，Python 分析器先用解析行波验证方向符号：

| 合成波传播方向 | 坐标解释 | 测试结果 |
|---:|---|---|
| 0° | 船首 `+y` | 通过 |
| 90° | 右舷 `+x` | 通过 |
| −180° | 船尾 `−y` | 通过 |
| −90° | 左舷 `−x` | 通过 |

当前 `test_sea_clutter_v02.py` 共 16 项通过。该结果确认 Python 的时空 FFT 方向、坐标轴顺序和正负号定义正确；因此后续 MATLAB 方向差异主要应归因于海面生成器方向语义/方向谱，而不是分析器把 0°、90°、180°、−90°弄反。

## Python 分析结果

| MATLAB 方向 | 船体配置方向 | `ss2` 主导波向 | `ss2` 方向误差 | `ss3` 主导波向 | `ss3` 方向误差 |
|---:|---:|---:|---:|---:|---:|
| 0° | 90° | 86.109° | −3.891° | 131.168° | 41.168° |
| 90° | 0° | 4.005° | 4.005° | 11.092° | 11.092° |
| 180° | −90° | 86.109° | 176.109° | 131.168° | −138.832° |

方向误差使用带符号角度差；三级上限的单谱峰误差较大，属于有限随机海面和单峰 FFT 诊断结果，不能单独作为方向语义判定。

## 关键调试发现

在相同海况和相同随机种子下，`WindDirection=0°` 与 `WindDirection=180°` 的 `/truth/height_m` 数据逐元素相同：

```text
ss2_normal 最大绝对差：1.6653345369377348e-16 m
ss2_normal 平均绝对差：2.338039082997196e-17 m
```

这在数值上等于同一个海面实现；差异主要只出现在 HDF5 元数据中的风向。当前结果说明：

1. 不能把 MATLAB `WindDirection=180°` 当成 `0°` 的可靠反向传播验证；
2. `270°` 不能直接传给当前 `seaSurface` 自动海谱接口；
3. 当前 `WindDirection=90° → 船体配置方向 0°` 仍可以作为船首向波向基线，但“顺风/逆风”语义尚未被本矩阵独立证明；
4. 要验证东西向和正反向传播，下一步需要自定义方向谱、坐标反射验证，或从 `seaSurface` 生成的高度场做独立时空传播方向验证。

## y 轴坐标镜像验证

对已验收的 `v02b_forward_seed101_hs1m` 原始 MATLAB HDF5 做 `y` 轴镜像，保持 Hs、周期、网格、随机种子和元数据不变，仅将 `height_m[:, :, :]` 的前向轴反转。结果如下：

| 工况 | 原始主导波向 | y 镜像后主导波向 | 预期关系 |
|---|---:|---:|---|
| `ss1_rippled` | 19.856° | 160.144° | 前后方向反转 |
| `ss2_normal` | 4.005° | 175.995° | 前后方向反转 |
| `ss3_nominal` | 11.092° | 168.908° | 前后方向反转 |
| `ss3_upper` | 11.092° | 168.908° | 前后方向反转 |

该结果确认 Python 方向估计器和当前船体 `+y` 前向坐标约定能够识别受控的前后传播反转。镜像运行是几何回归输入，不应被描述为 MATLAB `WindDirection=180°` 的物理海谱结果。

## 当前结论边界

本记录不是完整方向验收，也不授权把 `WindDirection` 直接称为气象学“风来自方向”。当前可使用的报告表述是：

> `WindDirection=90°` 经项目坐标映射得到船体 `0°`，作为船首向波向基线；MATLAB 自动海谱方向语义仍需通过自定义方向谱或独立传播方向实验确认。

## 产物位置

Python 图片分别位于：

```text
simulation/stages/v02_dynamic_sea_truth/results/python/
  v02b_dir000_seed101_analysis/figures/
  v02b_dir090_seed101_analysis/figures/
  v02b_dir180_seed101_analysis/figures/
```

本阶段的 MATLAB 方向矩阵生成器为：

```text
simulation/matlab/run_v02_direction_matrix.m
```

批量 Python 分析器为：

```text
simulation/analyze_v02_direction_matrix.py
```

受控镜像工具为：

```text
simulation/mirror_v02_run.py
```
