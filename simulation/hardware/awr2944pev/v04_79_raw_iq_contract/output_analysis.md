# V0.4.79 DCA1000/LVDS 原始 IQ 接入准备

本阶段定义了真实采集数据进入海杂波/AoA 管线前的最小契约。契约文件见 `raw_iq_contract.json`，验证器见 `simulation/run_v04_79_raw_iq_contract.py`。

## 输入形状

- 原始 ADC IQ：`(chirp, sample, rx)`，复数解码后 RX 维度必须为 4；
- TDM 重组后的虚拟 IQ：`(frame, sample, rx, tx)`，RX 和 TX 都必须为 4；
- HDF5 数据集名称支持 `/radar/iq` 或 `/recovered/virtual_iq`；NPZ 支持 `iq` 或 `raw_iq`。

## 必须随数据保存的证据

CFG 快照、ADC 类型和端序、LVDS wire order、TDM TX 序列、通道顺序验证状态、TI 校准状态与幅相矩阵、采集时间戳、IMU/船体姿态参考和安装角度。

## 边界

验证器通过只表示文件形状、类型和元数据满足输入契约，不表示通道顺序、相位中心、AoA 精度或海杂波模型已经被硬件验证。`channel_order_verified=false` 的数据只能进入 V0.4.77/V0.4.78 排列敏感性分析，不能直接作为实板角度结论。
