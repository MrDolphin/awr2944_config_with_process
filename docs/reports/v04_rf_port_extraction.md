# V0.4.14 RF 网络与馈电候选提取

## 提取内容

本阶段从 `PROC113D_ASCII.PcbDoc` 的 8 个 RF 网络中提取：

- Pad 的网络名、网络号、器件编号、Pad 名、层和坐标；
- Track 端点范围；
- 每个 TX/RX 网络的对象计数。

结果目录：

`simulation/stages/v04_aoa_cfar_point_cloud/output/v04_14_rf_port_extraction/`

## 关键发现

RF Pad 坐标可以作为芯片端/器件端的连接参考。例如 TX 网络出现同一器件 `COMPONENT=727` 的 B3、B5、B7、B9 Pad，RX 网络也有对应的 F2、H2、K2、M2 Pad。这些坐标比整块铜区几何中心更接近“馈电链路参考点”，但它们仍不是天线辐射单元的相位中心。

Track 端点范围能够帮助区分从芯片端引出的走线方向和进入天线铜区的另一端。当前脚本只做对象级提取，没有把 Track、Arc、Via 建成完整连通图，也没有推断电磁相位延迟。

## 使用边界

- Pad/Track 坐标可用于追踪 RF 拓扑和定义候选参考点；
- 不能直接替代天线相位中心；
- 不能由几何距离推断 77 GHz 的真实相位延迟；
- 下一步需要把 Track/Arc/Via 连通关系建图，并结合层叠介质和电磁仿真或角反射器标定。
