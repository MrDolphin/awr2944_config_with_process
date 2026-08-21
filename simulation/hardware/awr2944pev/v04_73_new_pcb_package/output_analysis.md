# V0.4.73 新增 SPRR440/441 PCB 资料分析

## 输入与可提取性

- `sprr441a/SPRR441/PROC113D_ASCII.PcbDoc`：Altium ASCII PCB，解析得到 8 个 RF 网络（TX1–TX4、RX1–RX4），每个网络包含 1 个 Pad、1 个 Region，并统计到 Arc/Track 对象。它可以提供 PCB 坐标、网络名、铜区顶点、走线端点和层叠字段。
- `sprr440a (1)`：当前目录主要包含 STEP、装配 PDF、原理图 PDF 和 BOM；适合板框/安装方向、连接关系和版本追溯，不能单独提供电气相位中心。

## 已生成文件

- `pcb_antenna_regions.csv`：8 个 TX/RX 铜区的几何中心、包围盒和源文件。
- `rf_ports/rf_net_pads.csv`、`rf_ports/rf_net_geometry_summary.csv`：RF 网络 Pad 与 Track 端点候选。
- `layers/layer_stack.csv`、`layers/rf_object_layers.csv`：层叠和 RF 对象所在层。
- `cad_virtual_array_coordinates.csv`：由 TX/RX 铜区几何中心求和得到的 16 个虚拟通道坐标。
- `package_audit.md`：两套资料的文件类型、大小和 SHA-256 追溯清单。

## 当前结论

这批资料足以把当前 AoA 仿真的“候选阵列几何”从抽象 mapping 提升为 CAD-derived 几何输入，并可比较理想半波长阵列与 PCB 几何阵列的影响。但铜区几何中心不是天线电气相位中心，不能据此宣称真实 AoA 精度或真实方向图；下一步仍需用 HFSS/CST 提取端口/相位中心，或用角反射器在已知方位上完成通道幅相和阵列坐标校准。

`sprr440a (1)` 中的 STEP 可用于机械坐标系和安装基准核对；若要把它用于阵列相位中心，还需要与 ASCII PCB 坐标系建立原点、单位、板面法向和旋转变换。
