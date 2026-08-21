# V0.4.103 TX/RX PCB 网络追踪

输入：`C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_ASCII.PcbDoc`

共找到 65 个目标网络图元：Arc=26, Pad=8, Track=31。

## 结论

TX/RX 网络已经可以从 AWR2944 芯片封装焊盘追踪到 Top 层 RF 走线和圆弧。当前导出中没有出现带目标网络名的 Region、Polygon 或独立天线 Pad，因此不能把走线终点直接当作天线相位中心。

本结果适合作为阵面建模的几何线索和资料审计证据，不足以单独生成真实方向图或校准 AoA。

完整图元表见 `rf_network_primitives.csv`；机器可读结论见 `summary.json`。
