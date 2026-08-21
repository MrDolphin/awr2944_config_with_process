# V0.4.60 安装姿态敏感性扫描

## 阶段目的

V0.4.59 已能将 PCB/CAD 候选坐标变换到雷达坐标系。本阶段围绕名义“板面垂直”姿态扫描安装误差，量化滚转、俯仰和航向变化对 RF 区域三维位置及板面法向的影响，为后续接入 IMU 外参和实测 AoA 提供误差基线。

## 扫描设置

名义姿态为 `roll=90°，pitch=0°，yaw=0°`，坐标系约定沿用 V0.4.59。扫描偏置为：

- roll：`-2°、0°、+2°`；
- pitch：`-2°、0°、+2°`；
- yaw：`-5°、0°、+5°`。

共 27 个姿态组合，输入 8 个 TX/RX PCB 铜区几何中心候选。

## 结果

- 名义板面法向：雷达坐标 `(0,-1,0)`；
- 名义板面法向与竖直夹角：90°；
- 扫描中最大 RF 区域候选坐标移动：约 11.0352 mm；
- 最大移动场景：roll 偏置 -2°、pitch 偏置 +2°、yaw 偏置 +5°；
- 姿态是否实测：否；
- 电气相位中心是否确认：否。

## 如何理解

这 11.0352 mm 是“相对于名义姿态的几何坐标移动”，不是 AoA 误差，也不是雷达检测距离误差。刚体旋转下阵元间距离不变，但每个阵元在雷达前/左/上坐标分量会变化。对于低安装高度、近海面目标，俯仰和滚转误差会直接改变海面微元的局部掠射角和点云投影，因此需要在真实船体姿态补偿中使用 IMU 数据。

## 输出

- `simulation/hardware/awr2944pev/v04_60_pose_sensitivity/candidate_scan/pose_sensitivity.csv`
- `simulation/hardware/awr2944pev/v04_60_pose_sensitivity/candidate_scan/region_displacement.csv`
- `simulation/hardware/awr2944pev/v04_60_pose_sensitivity/candidate_scan/summary.json`
- `simulation/hardware/awr2944pev/v04_60_pose_sensitivity/candidate_scan/output_analysis.md`

## 证据边界

当前偏置范围是工程敏感性假设，不是 IMU 测量结果。PCB RF 区域仍是铜区几何中心候选，不是电气相位中心。下一步应将船上 IMU 的时间同步姿态写入同一坐标链路，再用角反射器或已知角度目标测得 AoA 误差，避免把几何敏感性结果误报成实测性能。
