# V0.4.100 UART TLV record 解码

本阶段按 legacy `radar_server.py` 的 40 字节 UART frame header 和 TLV type 1 点云格式解码 record 文件。

解码帧数：2742；点数：34423。这些点是板载 UART 输出的 detected points，不是 DCA1000 ADC IQ，不能直接用于原始 AoA 重建。
