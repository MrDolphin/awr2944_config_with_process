# V0.4.149 原始 capture.bin 几何审计

状态：`raw_capture_geometry_incomplete`

文件大小：`0` bytes；期望每帧：`671744` bytes；完整帧：`0`；余数：`0`。

问题：`capture_missing`。

本阶段只做字节几何和 UART 头识别，不证明文件一定是有效 ADC IQ，不验证通道顺序、TI 校准或硬件 AoA。
