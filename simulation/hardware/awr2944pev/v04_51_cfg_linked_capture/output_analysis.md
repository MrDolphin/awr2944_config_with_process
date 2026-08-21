# V0.4.51 CFG 联动回归分析

本目录使用项目基准 CFG 和合成整帧 `.bin`，不是实测 DCA1000 数据。

## 图文数据分析方法

1. 先查看 `cfg_linked_result.cfg_contract.json`，确认 CFG 推导的 RX、chirp、sample、TX 顺序和单帧字节数；
2. 检查输入 `.bin` 是否为单帧字节数的整数倍；
3. 对 HDF5 中 `/range_doppler/power_linear` 找已知距离附近峰值；
4. 比较 `expected_range_m` 与 `peak_range_m`；
5. 最后查看 `channel_order_verified`，为 `false` 时不能把 AoA 当成真实硬件角度精度。

## 本次结果

- CFG：`profile_3d_3Azim_1ElevTx_awr2944P.cfg`；
- RX：4；chirps/frame：64；samples/chirp：656；
- TX 序列：`0,2,3,1`；
- 单帧 bytes：`671744`；
- 峰值距离：`0.522900 m`；
- 峰值速度：`0 m/s`；
- channel order verified：`false`。

## 结论

CFG→采集参数→DCA1000 解码→距离-Doppler 校准入口已闭环。真实采集接入时，如果文件大小不是整帧字节数整数倍，程序会提前拒绝，避免生成不可追溯的功率或 AoA 结果。
