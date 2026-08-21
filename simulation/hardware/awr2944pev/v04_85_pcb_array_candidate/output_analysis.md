# V0.4.85 PCB/CAD 阵列候选提取报告

## 输入资料

- Altium ASCII PCB：`C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_ASCII.PcbDoc`
- SHA-256：`eba1791d75a42fe02ad2aa9672398f9252721cc243366510d27e3b170a9ceda6`
- 解析对象：`RF net -> chip Pad -> antenna Region`

## 已确认的几何证据

ASCII PCB 中存在 8 条 RF 网络：TX1–TX4、RX1–RX4。每条网络都能关联到 AWR 芯片焊盘和一个 TOP 层 Region，因此本阶段不是根据图片估计阵元位置，而是从 CAD 导出的坐标字段直接提取。

- RX 阵元局部 x 坐标约为 54.611、56.511、58.411、60.311 mm。
- RX 相邻间距约为 1.900 mm，即约 0.49 λ（77 GHz）。
- TX1/TX3/TX4 位于近似同一 y 行，横向相邻间距约为 7.837 mm，即约 2.01 λ；这与 3 个方位 TX 的候选布局一致。
- TX2 的 y 坐标相对该行偏移约 3.135 mm，且 CFG 的 3Azim/1Elev 配置将 TX2 作为独立通道，因此 TX2 是俯仰 TX 候选。最终俯仰相位中心仍需实测确认。

## 如何接入仿真

`rf_array_candidates.csv` 可作为 TX/RX 几何候选输入，`virtual_array_candidates.csv` 给出 4×4 虚拟通道的质心和坐标。当前只替换阵元位置，不替换 TI SDK 的通道校准、相位中心、天线方向图或生产测试数据。建议先比较理想半波长阵列与本候选几何的波束图、AoA 偏差和海杂波点云扩散，再用角反射器实测冻结坐标变换。

## 限制

本结果仍不是“已校准实测阵列”：PCB Region 的几何中心不一定等于电磁相位中心；板面坐标到雷达前视坐标的旋转、板面法向、天线极化、封装影响和 TX/RX 通道幅相校准尚未由该文件证明。`hardware_validated` 保持为 `false`。
