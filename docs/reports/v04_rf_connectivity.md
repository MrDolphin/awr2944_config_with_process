# V0.4.15 RF 网络几何连通性

## 目的

将 ASCII PCB 中的 RF 网络对象连接成几何图：

```text
Pad → Track / Arc → Region 边界
```

用于检查芯片端 RF Pad 是否能沿 PCB 几何对象连接到对应天线铜区，并计算几何路径长度。

## 输出

结果目录：

`simulation/stages/v04_aoa_cfar_point_cloud/output/v04_15_rf_connectivity/`

核心文件为 `rf_connectivity_summary.csv`。

## 解释边界

- `pad_to_region_connected=true` 只代表 ASCII 几何对象在 1 mil 容差下连通；
- 几何路径长度不是传输线电长度；
- 没有计算介质有效介电常数、过孔寄生、阻抗过渡、互耦或相位延迟；
- 因此该结果不能直接修改 AoA 相位，也不能生成真实相位中心。

## 下一步

若要进入相位模型，需要获取 PCB 层叠介质参数和 RF 走线层信息，再进行传输线/全波仿真，或用角反射器实测校准每个 TX/RX 通道的等效幅相。
