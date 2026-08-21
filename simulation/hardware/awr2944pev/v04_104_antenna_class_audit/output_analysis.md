# V0.4.104 Antenna 网络类审计

输入：`C:\Users\56461\Downloads\2944p资料\sprr441a\SPRR441\PROC113D_ASCII.PcbDoc`

## 直接证据

PCB 中存在 `Antenna` 网络类，成员为：TX4, TX3, TX2, TX1, RX4, RX3, RX2, RX1。

文件中还有启用的 `Antenna` Width 规则，作用域是 TX/RX 天线网络。该规则显示最小线宽 8.4 mil、最大线宽 10 mil，Top 层偏好 8.4 mil。

## 解释边界

这证明 TI 将 8 条 TX/RX 网络按天线 RF 网络进行 PCB 设计约束；它不能单独给出天线单元坐标、相位中心、方向图或 AoA 校准矩阵。

下一步应将该网络类证据与天线版图/EM 模型或实测方向图对应起来。
