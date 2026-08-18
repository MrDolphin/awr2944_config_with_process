# AWR2944P 分阶段仿真与产物保存规范

## 目的

每个仿真阶段必须有独立的设计边界、输入快照、运行结果和验收结论，
以便复现、横向对比、论文制图和定位误差来源。不同阶段、不同生成器及
不同运行批次不得写入同一个结果目录。

## 固定目录

```text
simulation/stages/
  v01_flat_sea_geometry/
    README.md
    results/python/<run_id>/
    results/matlab/<run_id>/
  v02_dynamic_sea_truth/
    README.md
    results/python/<run_id>/
    results/matlab/<run_id>/
  v03_complex_echo_range_doppler/
    README.md
    results/python/<run_id>/
    results/matlab/<run_id>/
  v04_aoa_cfar_point_cloud/
    README.md
    results/python/<run_id>/
    results/matlab/<run_id>/
```

`run_id` 使用可读且不可复用的名字，例如
`20260818_153252_baseline_1m_pitch_sweep`。禁止把 `latest` 作为唯一保存位置。

## 每个阶段README必须说明

1. 科学问题和本阶段不回答的问题；
2. 坐标系、单位、安装姿态和正负号；
3. 输入参数、默认值、扫描范围与来源；
4. 物理模型、近似条件和适用边界；
5. 输出数据集、图、统计指标和单位；
6. 解析验收、数值验收及与上一阶段的回归关系；
7. 已知局限和进入下一阶段的门槛。

## 每个运行目录必须保存

```text
design_snapshot.md       本次运行实际采用的假设和目的
run_config.json          完整数值输入
radar_profile.cfg        雷达配置快照
environment.json         Python、MATLAB、工具箱和代码提交信息
summary.json             可机器比较的指标
figures/                 PNG/SVG结果图
data/                    HDF5或其他数值结果
validation.md            通过项、失败项和人工检查结论
```

现有 V0.1 代码产生的平铺文件可以保留，但新运行应按上述名字分组。HDF5、
图片及大体积结果默认不提交 Git；README、配置模板、结果字段契约和小型摘要
应提交。重要实验结果还必须备份到独立存储，不能只依赖被 Git 忽略的目录。

## 阶段门槛

- V0.1：平面海面几何、3/6 dB落区和相对功率已完成 Python/MATLAB 验收。
- V0.2：只加入动态海面真值。必须验证浪高、浪向、周期/相速、表面法向和
  散射单元径向速度，暂不把输出称为雷达复数回波。
- V0.3：在冻结的 V0.2 真值上加入 4TX/4RX FMCW 复数回波和
  Range-Doppler，并与静止平面海面极限情况回归对比。
- V0.4：加入二维 AoA、CFAR 和三维杂波点云，输出检测概率、虚警率、角度
  误差和点云稳定性指标。

每一阶段通过后才开始下一阶段。若某个下游结果异常，先用上一阶段保存的
输入和真值复现，不同时修改多个物理层或处理层。
