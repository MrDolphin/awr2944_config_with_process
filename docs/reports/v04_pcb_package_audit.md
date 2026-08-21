# V0.4.11 AWR2944P PCB/CAD 资料包审计

## 审计对象

本阶段审计用户提供的两个资料目录：

- `C:\Users\56461\Downloads\2944p资料\sprr440a (1)`：含 3D STEP、装配 PDF、原理图 PDF 和 BOM；
- `C:\Users\56461\Downloads\2944p资料\sprr441a`：含 Altium PCB、ASCII PCB、层叠 PDF、工程文件和原理图。

逐文件清单、大小、类型和 SHA-256 见 `simulation/stages/v04_aoa_cfar_point_cloud/output/pcb_package_inventory.md`。该文件不把外部资料复制进仓库，只记录引用和哈希。

## 能够直接提取的数据

`PROC113D_ASCII.PcbDoc` 是可自动解析的 Altium ASCII PCB。现有提取器可识别 TX1～TX4、RX1～RX4 八个 RF 网络，并输出每个铜区的顶点数、包围盒、几何中心和毫米单位坐标。这些数据已经进入 `simulation/hardware/awr2944pev/pcb_antenna_regions.csv`，并用于生成 `cad_virtual_array_coordinates.csv`。

## 不能直接等同于天线相位中心的数据

- 二进制 `PROC113D_BRD.PcbDoc`：文件格式为 Altium OLE；应先从 Altium 导出 ASCII 或 IPC-2581/ODB++，否则不能保证对象和坐标解析正确。
- `PROC113D_BRD.step`：适合检查板框、安装面和机械位置；STEP 实体没有自动保证 TX/RX 的电气相位中心标识。
- 装配图、层叠图和原理图：可以确认方向、层号、馈电关系和版本，但仍需结合 PCB 坐标。
- BOM：可确认器件版本，不包含阵列相位中心。

## 对 AoA 仿真的影响

当前 PCB 铜区中心只能作为第二种阵列模型：

```text
理想半波长阵列
    vs
PCB 铜区几何中心近似
    vs
真实电气相位中心/实测校准阵列
```

此前 V0.4.5 的 PCB 近似模型出现较大的 AoA RMSE，说明“铜区几何中心直接当作相位中心”不能作为真实 EVM 精度结论。下一步应先完成坐标原点、板面朝向、馈电点和 CFG 通道映射，再用角反射器或全波/阵列电磁仿真校正。

## V0.4.11 验收结论

- 资料包已完成可追溯清单和文件格式分类；
- ASCII PCB 已具备 8 个 TX/RX RF 区域的自动提取路径；
- 二进制 PCB 和 STEP 已明确可用范围与限制；
- 阵列坐标仍标记为几何近似，尚未升级为真实相位中心；
- 下一步是导出/解析馈电点与板面坐标，并接入多模型 AoA 对比。
