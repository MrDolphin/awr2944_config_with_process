# V0.4.59 PCB 板坐标到雷达坐标姿态候选

## 阶段目的

将 `PROC113D_ASCII.PcbDoc` 提取的 PCB 相对坐标，按照显式的旋转和平移参数变换到雷达坐标系，输出三维阵元候选坐标、板框坐标和阵元间基线。该阶段解决的是“坐标如何变换和复现”，不是“自动猜出真实安装姿态”。

## 输入和坐标约定

- PCB 输入：`simulation/hardware/awr2944pev/v04_35_mechanical_datum/rf_regions_board_relative.csv`。
- 板框输入：`simulation/hardware/awr2944pev/v04_35_mechanical_datum/board_outline.csv`。
- 雷达坐标系候选：`x=前方，y=左方，z=向上`。
- 旋转约定：`Rz(yaw) @ Ry(pitch) @ Rx(roll)`。
- 当前候选：`roll=90°，pitch=0°，yaw=0°，translation=(0,0,0) mm`。

## 本次结果

当前候选把 PCB 平面放为近似竖直平面：

- RF 区域：8 个（TX1–TX4、RX1–RX4）；
- 板框顶点：5 个；
- 变换后板面法向：雷达坐标 `(0,-1,0)`；
- 板面法向与竖直方向夹角：90°；
- 安装姿态确认：否；
- 电气相位中心确认：否。

因此，90° 表示“在这个输入候选中，板面法向水平、板面近似垂直”，不是从 STEP/PCB 文件自动证明了实物已经垂直安装。

## 输出文件

- `simulation/hardware/awr2944pev/v04_59_board_to_radar_pose/candidate_vertical_run/rf_regions_radar_coordinates.csv`
- `simulation/hardware/awr2944pev/v04_59_board_to_radar_pose/candidate_vertical_run/board_outline_radar_coordinates.csv`
- `simulation/hardware/awr2944pev/v04_59_board_to_radar_pose/candidate_vertical_run/array_baseline_metrics.csv`
- `simulation/hardware/awr2944pev/v04_59_board_to_radar_pose/candidate_vertical_run/coordinate_schema.json`
- `simulation/hardware/awr2944pev/v04_59_board_to_radar_pose/candidate_vertical_run/summary.json`
- `simulation/hardware/awr2944pev/v04_59_board_to_radar_pose/candidate_vertical_run/output_analysis.md`

## 如何使用这些结果

1. 用 `rf_regions_radar_coordinates.csv` 检查阵元是否被放到了预期的前/左/上方向。
2. 用 `board_outline_radar_coordinates.csv` 检查板框和安装遮挡方向。
3. 用 `array_baseline_metrics.csv` 检查阵元间距；例如 RX1–RX2 的候选距离约为 1.90 mm，但这仍是 PCB 铜区几何中心间距。
4. 替换 `candidate_vertical_pose.json` 中的 RPY、平移和 `installation_pose_confirmed`，即可复现不同安装候选。

## 结论边界

目前只能得到“板级机械坐标经过指定姿态后的候选三维结果”。还不能由此确认 AWR2944P 的真实天线相位中心、天线方向图、板面法向或 TI SDK 的真实 AoA 几何。下一步应使用装配基准/实物测量/IMU 外参，并用已知角度目标验证变换后的 AoA 误差。
