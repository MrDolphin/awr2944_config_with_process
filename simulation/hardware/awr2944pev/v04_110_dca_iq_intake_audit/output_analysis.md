# V0.4.110 DCA1000 ADC IQ 接入审计

本阶段只读扫描采集目录，按文件格式和配置尺寸进行保守分类，不修改原始数据。‘候选’不等于已解码，也不等于已完成 AoA 验证。

扫描文件：3；UART 点云记录：3；DCA1000 原始 IQ 候选：0。

## 分类含义

- `uart_point_cloud_record`：识别到旧版 radar_server 的帧/TLV 点云记录，只能用于点云回放或协议对照，不能当作 ADC IQ。
- `dca1000_raw_iq_candidate`：不是已知 UART 记录且文件大小符合 complex-int16 ADC 字节几何，只能进入下一步 manifest、端序、LVDS wire order 和帧边界检查。
- `unknown_requires_manifest`：缺少足够证据，不能进入 AoA。

## 本阶段验收门

1. 保存采集 CFG 快照和 DCA1000 配置；2. 记录 ADC 位宽、I/Q 端序、RX 数量、样本数、chirp/frame；3. 通过已知角度目标确认通道顺序；4. 记录 TI 校准命令、校准结果和幅相矩阵；5. 绑定时间戳、IMU 姿态和安装角度。

## 证据边界

文件尺寸只能说明‘可能符合’，无法证明文件中确实是 ADC IQ。没有 `channel_order_verified` 和 `ti_calibration_verified` 时，不得把后续角度结果写成 AWR2944P 硬件精度。
