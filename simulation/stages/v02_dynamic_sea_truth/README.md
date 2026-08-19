# V0.2 动态海面真值（1.0 m船首方向seed101已验收）

## 本阶段问题

研究海况由0级增加到3级时，海面高度、表面法向、垂向速度、雷达相对
方位/俯仰、局部掠射角和雷达净空如何变化。输出是物理场真值，不是
ADC/IQ、海杂波功率、Range-Doppler、AoA、CFAR或点云。

V0.2-B在V0.2-A基础上增加主导波传播方向、周期、波长、相速度，以及固定
海面网格单元相对雷达的径向距离变化率。它们用于检查动态几何和未来多普勒
输入，但还不等于水质点完整三维轨道速度或海杂波多普勒谱。

本阶段最高只允许3级海况；任何 `sea_state > 3` 或 `target_hs_m > 1.00`
的配置在生成前被拒绝。

WMO三级海况的物理分类上界仍是Hs=1.25 m；1.00 m是本项目人为设置的
仿真输入上限，两者不得混为一谈。`classify_sea_state` 保留WMO分类，
配置入口另行执行项目上限检查。

当前数值范围与默认值统一保存在
`simulation/configs/sea_states_0_to_3.json`，参数来源在下文逐项说明。

## 第一版海况矩阵

| case_id | 海况 | 目标有效波高 Hs | 用途 |
|---|---:|---:|---|
| `ss0_flat` | 0 | 0 m | V0.1平面回归 |
| `ss1_rippled` | 1 | 0.05 m | 轻微波纹 |
| `ss2_normal` | 2 | 0.30 m | 暂定日常小浪基准 |
| `ss3_nominal` | 3 | 0.85 m | 三级典型状态 |
| `ss3_upper` | 3 | 1.00 m | 本项目三级海况上限工况 |

默认雷达相位中心在平均海平面上方1 m，PCB参考姿态竖直、波束水平向前，
正安装俯仰角表示波束向下。船体坐标为 `x` 向右、`y` 向前、`z` 向上。

## 固定输入、扫描范围与来源

| 输入 | 默认值/范围 | 来源与用途 |
|---|---|---|
| 海况 | 0、1、2、3；最高3 | WMO海面状态范围；项目上限 |
| 目标Hs | 0、0.05、0.30、0.85、1.00 m | 项目工程代表点；三级人为封顶1.00 m |
| 雷达高度 | 1 m | 当前船载安装假设，待实测相位中心高度校准 |
| 安装姿态 | roll=0°、pitch=5°、yaw=0° | V0.1几何基准；V0.2-A不扫描姿态 |
| 横向网格x | -50～50 m，间隔1 m | 覆盖当前方位观察区域的首版有限网格 |
| 前向网格y | 2～100 m，间隔1 m | 与V0.1距离范围近似对齐，避开雷达原点 |
| 时间 | 0～20 s，间隔0.5 s | 覆盖多帧海面变化的首版低速真值采样 |
| 随机种子 | 101、202、303、404、505 | 冻结复现性；默认共5个种子 |
| MATLAB `WindDirection` | 90° | 非地心场景中从North顺时针；映射到船体坐标的向前 `+y/0°` 基准 |
| Fetch | 10000 m | 已有环境验证参数，属于待水池/海试校准假设 |
| 频谱空间分辨率 | 1 m | 与首版空间采样间隔一致 |
| 初始风速 | `searoughness(sea_state)` 输出 | MATLAB/Barton海况粗糙度模型，仅用于驱动频谱 |
| Hs容差 | 非零海况±10%；0级绝对误差≤1e-12 m | 自动验收阈值 |

上述Hs分级来自WMO海面状态范围；代表点、10 km Fetch、1 m网格和20 s时长
是本项目首版可计算性选择，不是AWR2944P厂家参数或目标海域实测结论。正式论文
应使用造浪池或海试数据重新校准，并保留本配置作为可复现基线。

MathWorks对非地心 `radarScenario` 的定义是：`WindDirection` 从North方向
顺时针计角。在本项目调用 `height(surface,[x;y])` 时，North对应第一个
MATLAB坐标，也就是船体 `x` 右舷轴。因此船体方向角和MATLAB风向的映射为
`vessel_direction = 90° - matlab_wind_direction`（再归一化到±180°）。
所以船首 `+y/0°` 基准必须配置MATLAB `WindDirection=90°`，而不是0°。

## 生成与分析方法

1. MATLAB使用 `searoughness(sea_state)` 获取该等级对应的初始风速；
2. MATLAB用 `seaSpectrum` 和有限 `Boundary` 创建 `seaSurface`；
3. 使用 `height(surface, points, t)` 采样三维动态高度场；
4. 每帧去除空间均值，并按 `Hs = 4*std(height)` 缩放到目标有效波高；
5. HDF5记录原始Hs和振幅缩放因子；
6. Python读取高度立方体并计算法向、垂向速度、雷达相对角度、局部掠射角、
   最小雷达净空和径向距离变化率；
7. Python对时间—前向—横向高度立方体进行三维FFT，提取最强非零行波分量，
   估计传播方向、周期、波长和相速度；
8. MATLAB原始运行与Python分析运行分别保存，互不覆盖。

掠射角和俯仰角同时保存两类离散程度：全空间—时间标准差描述整个观察区域
的综合分布；逐网格时间标准差的空间均值用于隔离海浪引起的时间抖动。
跨海况比较图使用后者，因此0级固定平面海面的时间抖动应严格为0。

振幅缩放保留了所采样频谱的空间形状与相位演化，但缩放后的结果不再表示
未经修改的风速/Fetch平衡海。论文和报告中必须称为“目标Hs控制的频谱动态
海面”，并同时报告风速、Fetch、原始Hs和缩放因子。

视线距离变化率定义为 `d(slant_range)/dt`，并按
`(height-radar_height)*vertical_velocity/slant_range` 计算：正值表示固定
网格海面单元
相对雷达远离，负值表示接近。它包含海面高度变化在视线方向上的投影，但不
包含水质点水平轨道速度、船体六自由度运动或电磁散射相位中心迁移，因此尚
不能据此声称已得到海杂波多普勒谱。

主导波向约定为船体坐标：0°指向 `+y`（船首方向），+90°指向 `+x`
（右舷）。平静海面没有可定义的主导周期/方向，其JSON摘要写为 `null`，
HDF5运动学标量写为 `NaN`。

Python分析会拒绝HDF5中的MATLAB风向与运行配置不一致的输入，并在摘要中
同时保存 `configured_vessel_wave_direction_deg` 和
`dominant_direction_error_deg`。单个最大谱峰是有限随机海面的诊断量，
不应在多随机种子和方向谱统计完成前等同于全部波能的平均传播方向。

## HDF5契约

MATLAB原始文件至少包括：

```text
/axes/x_m
/axes/y_m
/axes/time_s
/truth/height_m
/case/sea_state
/case/target_hs_m
/case/raw_hs_m
/case/amplitude_scale_factor
/case/random_seed
/case/wind_speed_mps
/case/wind_direction_deg
/case/fetch_m
/installation/height_m
/installation/mounting_pitch_deg
```

Python分析文件在 `/truth` 中增加：

```text
normal_x, normal_y, normal_z
vertical_velocity_mps
slant_range_rate_mps
slant_range_m
azimuth_deg
elevation_deg
grazing_angle_deg
```

Python分析文件的 `/kinematics` 还包括：

```text
dominant_wave_direction_deg
dominant_wave_period_s
dominant_wavelength_m
dominant_phase_speed_mps
```

`/validation` 保存实际Hs、Hs验收结果和最小雷达净空。V0.2文件中不得出现
`relative_power_db`、复数IQ或CFAR点云字段。

## 手工验收

在MATLAB R2025a GUI命令窗口执行：

```matlab
cd('D:\hp-laptop\USV\awr2944_sea_clutter_v02\simulation\matlab')

results = runtests('test_run_v02.m');
table(results)
assertSuccess(results)

caseIds = ["ss0_flat", "ss1_rippled", "ss2_normal", ...
    "ss3_nominal", "ss3_upper"];
summaries = run_v02("", "v02b_forward_seed101_hs1m", caseIds, 101);
disp(struct2table(summaries))
```

这一快速运行只使用一个随机种子，生成5个海况文件。通过后再删除第四参数，
运行配置中的5个冻结种子；完整25工况预计明显更慢。

随后在PowerShell、仓库根目录执行：

```powershell
python -m simulation.run_v02 `
  --input-run simulation\stages\v02_dynamic_sea_truth\results\matlab\v02b_forward_seed101_hs1m `
  --run-id v02b_forward_seed101_hs1m_analysis
```

重点查看Python分析运行中的：

```text
summary.json
validation.md
figures/sea_state_comparison.png
figures/<case_id>_seed101_overview.png
data/<case_id>_seed101_truth.h5
```

## 进入V0.3的门槛

1. 零波高结果退化为平面海面，法向为 `[0,0,1]`、垂向速度为零；
2. 五个目标工况均不超过3级，实际Hs误差不超过10%；
3. 固定随机种子可复现；
4. Python与MATLAB的HDF5字段和维度一致；
5. 3级上边界必须报告最小雷达净空，净空非正时标记几何无效；
6. 规则行波测试的周期、传播方向、波长、相速和径向距离变化率误差通过；
7. 多方向海面扫描以及船体运动耦合完成前，不得把径向距离变化率称为完整
   散射单元径向速度；
8. 保存完整运行目录并完成人工看图后，才允许进入复数FMCW回波阶段。

结果保存到：

```text
results/python/<run_id>/
results/matlab/<run_id>/
```

Python runner复用 `simulation.artifacts.create_run_directory`；MATLAB runner
镜像相同的非覆盖目录契约。两者都必须拒绝已存在的 `run_id`，并延续V0.1的
`data/`、`figures/`、环境、配置快照、雷达CFG、摘要和验收文件结构。

当前已验收的小型可提交证据保存在
`baselines/v02b_forward_seed101_hs1m/`。完整HDF5和PNG仍保存在忽略的
`results/` 目录，避免将大型二进制结果写入Git。该基线只接受一个随机种子
下的船首方向运动学闭环，不代表多种子、多方向或真实海杂波统计已经完成。
