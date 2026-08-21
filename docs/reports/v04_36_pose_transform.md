# V0.4.36 板框坐标到雷达/船体姿态变换

## 目的

将 V0.4.35 得到的 PCB 板框相对坐标，通过显式的旋转和平移接口转换为雷达或船体坐标候选：

```text
PCB 板框相对坐标
    ↓ Rz(yaw) @ Ry(pitch) @ Rx(roll)
    ↓ 平移向量
雷达/船体坐标候选
```

## 候选姿态

当前生成了：

```text
identity
board_vertical_pitch_plus90
board_vertical_pitch_minus90
board_vertical_roll_plus90
board_vertical_roll_minus90
yaw_plus90
```

这些是数学候选，不是实测安装姿态。

## 垂直安装角的解释

脚本输出 `normal_to_vertical_abs_angle_deg`：

- `0°`：板面法向与竖直方向平行，板面近似水平；
- `90°`：板面法向与竖直方向垂直，板面近似垂直。

这与“雷达安装角垂直地面 90°”不能混为一谈。要确认实际安装角，还需要知道：

1. PCB 板面法向；
2. 雷达波束/天线法向；
3. 船体坐标系；
4. 安装支架或安装孔的机械基准；
5. IMU 到雷达的外参。

## 结果

结果目录：

```text
simulation/hardware/awr2944pev/v04_36_pose_transform/
```

- `pose_candidates.csv`：姿态旋转、板面法向和法向-竖直夹角；
- `rf_regions_pose_candidates.csv`：8 个 RF 区域在各候选姿态下的坐标；
- `pose_transform_schema.json`：坐标链和旋转约定；
- `summary.json`：机器可读状态；
- `output_analysis.md`：边界说明。

## 结论

当前已经有可复现的板框坐标到雷达/船体坐标变换接口，但 `installation_pose_confirmed=false`。任何海杂波姿态补偿或“垂直地面 90°”仿真都应在输入实测安装姿态后再冻结。
