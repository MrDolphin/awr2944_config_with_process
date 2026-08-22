# V0.4.114 V0.4.113 采集完整性检查

计划工况：30；基础文件齐全：0；满足 V0.4.112 解码门：0。

## 通过条件

每个案例必须有 manifest、capture.bin/HDF5、manifest 指定的 CFG，且 CFG SHA-256 与 manifest 一致；同时 samples/chirp、chirps/frame、RX 和 TDM TX 顺序必须为当前 CFG 提取值。

## 结果解释

基础文件齐全不等于 ADC IQ 有效；满足解码门也不等于通道顺序、TI 校准和硬件 AoA 已验证。

当前下一步：collect missing capture.bin/HDF5 files。
