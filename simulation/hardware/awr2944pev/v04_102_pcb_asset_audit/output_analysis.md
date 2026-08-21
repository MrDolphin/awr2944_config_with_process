# V0.4.102 EVM PCB 资产审计

输入：`C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_ASCII.PcbDoc`

## 已提取
- 格式：Altium ASCII PcbDoc；芯片：U29 / XA2944BGALT。
- 芯片封装 TX/RX 网络焊盘：8 个；坐标单位同时保存为 mil 和 mm。
- 板级原点：3337.6697mil、2034.0003mil；板框顶点字段已写入 `summary.json`。

## 不能直接等同的内容

这些坐标是 AWR2944 芯片封装引脚坐标，不是 PCB 天线单元的相位中心。若直接把它们当作阵列坐标，会把封装内部连接误当成辐射单元，导致 AoA 和波束图结论失真。

下一步需要从 TX/RX 网络继续追踪 PCB 铜箔/过孔/天线单元，或取得 TI 的天线版图/EM 模型；再与已知角度采集和校准矩阵闭环。
