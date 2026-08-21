# V0.4.35 PCB 机械坐标基准提取

## 实际结果

从 `PROC113D_ASCII.PcbDoc` 提取到：

```text
PCB 原点：3337.6697 mil, 2034.0003 mil
板框尺寸：约 85.000 mm × 125.000 mm
板框顶点：5 个记录（其中包含重复顶点）
RF 区域：8 个
机械参考候选：23 个
```

8 个 RF 区域被转换到相对板框原点的毫米坐标。例如：

```text
RX1: (54.6106 mm, 83.5626 mm)
RX2: (56.5106 mm, 83.5626 mm)
RX3: (58.4106 mm, 83.5626 mm)
RX4: (60.3106 mm, 83.5626 mm)
TX1: (67.0902 mm, 83.6890 mm)
TX2: (71.0082 mm, 86.8240 mm)
TX3: (74.9272 mm, 83.6890 mm)
TX4: (82.7642 mm, 83.6890 mm)
```

这些坐标是 PCB 板框坐标下的铜区质心，状态统一为：

```text
mechanical_board_relative_not_phase_center
```

## 装配图结论

装配图 PDF 文本层主要是元件位号和装配标识，没有提取出可直接用于天线电气相位中心或船体安装姿态的尺寸标注。因此本阶段没有把装配图当作相位中心证据。

## 结果文件

```text
simulation/hardware/awr2944pev/v04_35_mechanical_datum/
```

- `mechanical_datum.json`：板框原点、尺寸和状态；
- `board_outline.csv`：板框顶点及相对原点坐标；
- `rf_regions_board_relative.csv`：RF 区域板框相对坐标；
- `mechanical_reference_candidates.csv`：FID、连接器、安装相关候选；
- `output_analysis.md`：边界说明。

## 结论边界

当前已经有一个可复现的 PCB 机械坐标基准，可用于坐标变换和安装配准筛查；但仍不能确认：阵面法向、板面相对于船体的姿态、垂直地面 90° 安装角、天线相位中心或真实 AoA 坐标。下一步需要机械装配测量或带安装基准的 CAD/装配图。
