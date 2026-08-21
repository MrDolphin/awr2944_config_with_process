# V0.4.51 CFG 联动 DCA1000 采集校验

## 本阶段目的

将 AWR2944P 的 CFG 作为 DCA1000 离线解码的单一参数来源，避免手工填写 chirp 数、ADC sample 数和 TX 顺序导致错位。处理前自动检查：

- RX 数量；
- 每帧 chirp 数；
- ADC samples/chirp；
- TDM TX 序列；
- 单帧 payload 字节数；
- 起始频率、斜率、采样率和 chirp 周期。

## 基准 CFG 推导结果

来自 `Config/profile_3d_3Azim_1ElevTx_awr2944P.cfg`：

| 参数 | 推导值 |
|---|---:|
| RX | 4 |
| 每帧 chirp | 64 |
| ADC samples/chirp | 656 |
| TX 序列 | TX0 → TX2 → TX3 → TX1 |
| 单帧 payload | 671,744 bytes |
| 起始频率 | 77 GHz |
| sweep bandwidth | 3.9998 GHz |
| sample rate | 13.349 MSPS |

## 合成整帧回归结果

使用与 CFG 字节数一致的合成 `.bin`：

- 文件大小：`671,744 bytes`；
- 目标距离：`0.522900 m`；
- 目标速度：`0 m/s`；
- 解码后峰值距离：`0.522900 m`；
- 解码后峰值速度：`0 m/s`；
- TX 序列正确传递为 `[0,2,3,1]`。

这证明了 CFG 解析、整帧字节数检查、DCA1000 解码、TDM 虚拟通道重排、距离-Doppler 峰值提取和校准 JSON 输出能够串联运行。

## 真实采集时的使用方式

```powershell
python -m simulation.run_v04_51_cfg_linked_capture `
  --cfg Config\profile_3d_3Azim_1ElevTx_awr2944P.cfg `
  --input path\real_capture.bin `
  --output simulation\hardware\awr2944pev\v04_51_cfg_linked_capture\measured_reference.h5 `
  --expected-range-m 20 `
  --expected-velocity-mps 0
```

如果 `.bin` 不是整帧大小的整数倍，程序会在 FFT 之前拒绝处理，避免静默地产生错位 AoA 或功率结果。

## 当前边界

合成回归没有验证真实 LVDS 字节顺序、RX 物理通道顺序、TX 相位中心和 TI SDK 通道重排；`channel_order_verified` 仍为 `false`。真实 AoA 和绝对功率校准仍需要已知角度目标、实测 DCA1000 IQ 以及测量来源记录。

## 输出文件

- [cfg_synthetic_capture.bin](../../simulation/hardware/awr2944pev/v04_51_cfg_linked_capture/cfg_synthetic_capture.bin)
- [cfg_linked_result.h5](../../simulation/hardware/awr2944pev/v04_51_cfg_linked_capture/cfg_linked_result.h5)
- [cfg_linked_result.cfg_contract.json](../../simulation/hardware/awr2944pev/v04_51_cfg_linked_capture/cfg_linked_result.cfg_contract.json)

