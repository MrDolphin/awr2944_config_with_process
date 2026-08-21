# V0.4.50 DCA1000 参考目标校准回归

本目录是合成 `.bin` 回归输入，不是实测 DCA1000 数据。

## 输入

- 64 chirp、128 samples/chirp、4 RX；
- 单 TX 序列 `0`；
- 合成参考目标距离 `21.079157 m`，速度 `0 m/s`。

## 结果

- 距离-Doppler 峰值：`21.079157 m`、`0 m/s`；
- ADC RMS：约 `1200.244`；
- reference power：约 `5.7637813395408e12`（合成 FFT 单位）；
- channel order verified：`false`。

## 结论

解码、TDM/通道数据结构、FFT 和校准 JSON 字段均已通过合成回归。真实 DCA1000 文件接入后，必须重新核对 LVDS 字节顺序、TX 序列、CFG 采样参数和已知目标测量来源，不能直接沿用本合成结果。
