# V0.4.80 原始 IQ→TDM→RD→AoA 端到端回放

本阶段先生成合成 4 TX/4 RX 虚拟 IQ，按 TX 序列 0,1,2,3 展平为原始 chirp，使用 little-endian int16 I/Q 编码，再解码和 TDM 重组。重组后的虚拟 IQ 同时进入距离-多普勒和 AoA 处理。

原始 IQ 形状：`(16, 64, 4)`；虚拟 IQ 形状：`(4, 64, 4, 4)`；int16 往返最大误差：`0.694`。

已知角度：az=5.000°、el=2.000°；估计角度：az=4.999°、el=2.013°。

## 边界

这是软件端到端回放闭环，不是 DCA1000 实测数据；HDF5 中明确记录 `channel_order_verified=false` 和合成校准状态。后续可替换 `/radar/raw_iq` 的来源文件，保留同一重组、RD、AoA 和报告接口。
