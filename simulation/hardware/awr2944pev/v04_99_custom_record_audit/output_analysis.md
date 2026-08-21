# V0.4.99 AWR UART record 文件结构审计

扫描文件：3 个。

## 结论

record_01.bin、record_02.bin、record_03.bin 均符合 legacy `radar_server.py` 的 UART 输出帧格式：8 字节 TI magic + 8 个 uint32，共 40 字节帧头；头部包含包长、平台标识 0x2944、帧号、检测目标数和 TLV 数量。它们不是 DCA1000 ADC 原始 IQ。

因此 `standard_dca_payload_ready=false` 的含义已明确：这些文件属于板载 UART 点云/中间结果记录，不能送入 V0.4.91 原始 IQ AoA 解码器。

## 下一步

应使用 UART TLV 解码器提取点云，作为板载点云对标数据；DCA1000 原始 IQ 仍需单独采集并保存为 payload-only BIN 或带明确元数据的 HDF5。
