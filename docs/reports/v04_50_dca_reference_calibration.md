# V0.4.50 DCA1000 参考目标功率校准入口

## 本阶段目的

本阶段把“等待 DCA1000 实测校准”落实为一个可执行的数据入口：

```text
DCA1000 .bin
    ↓
int16 I/Q 解码
    ↓
TDM 虚拟通道重排
    ↓
距离-Doppler FFT
    ↓
已知距离附近峰值提取
    ↓
reference_power_linear、ADC RMS、校准比例
```

当前输出使用的是项目内生成的合成 `.bin` 回归文件，状态明确为 `capture_processed_channel_order_unverified`，不代表真实硬件校准完成。

## 合成回归结果

本次合成参考目标设置为：

- 原始数据：64 chirp × 128 sample/chirp × 4 RX；
- TX 序列：单通道回归 `0`；
- 参考目标距离：`21.0792 m`；
- 目标 Doppler：`0 m/s`；
- 参考 ADC RMS：约 `1200.24`；
- 距离-Doppler 峰值：`21.0792 m`、`0 m/s`。

该结果证明了解码、FFT、峰值提取和 JSON 校准输出接口能够闭环。

## 为什么仍不能叫“已校准”

当前 `calibration_schema.json` 中仍然有：

```json
{
  "channel_order_verified": false,
  "measurement_provenance_required": true
}
```

原因是以下信息尚未由真实采集确认：

- DCA1000 LVDS 的实际字节/通道顺序；
- TX 轮询顺序和 TDM 配置；
- AWR2944P CFG 与采样数、chirp 数的对应关系；
- 已知目标真实距离和 RCS；
- 采集时的增益、窗函数和 FFT 定义；
- 真实目标的安装角度和方向图影响。

因此合成回归中的 `synthetic_to_measured_power_ratio` 仅验证字段计算，不可用于把当前海杂波仿真直接换算成 dBm。

## 换成真实 DCA1000 文件的命令

假设真实文件为 `capture.bin`，CFG 使用 128 ADC samples，DCA1000 解码需要知道完整 chirp 数：

```powershell
python -m simulation.run_v04_50_dca_reference_calibration `
  --input path\capture.bin `
  --output simulation\hardware\awr2944pev\v04_50_dca_reference_calibration\measured_reference.h5 `
  --chirps 256 `
  --samples-per-chirp 128 `
  --rx-count 4 `
  --expected-range-m 20 `
  --expected-velocity-mps 0 `
  --tx-sequence 0,1,2,3
```

如果已有合成参考功率，可额外提供：

```powershell
--synthetic-reference-power-linear <value>
```

脚本会将实测参考功率与该值的比值写入校准 JSON；但在 `channel_order_verified` 和测量来源确认之前，项目仍会保留“未完成真实 AoA/功率校准”的状态。

## 输出文件

- [synthetic_reference.bin](../../simulation/hardware/awr2944pev/v04_50_dca_reference_calibration/synthetic_reference.bin)
- [synthetic_reference.h5](../../simulation/hardware/awr2944pev/v04_50_dca_reference_calibration/synthetic_reference.h5)
- [synthetic_reference.calibration.json](../../simulation/hardware/awr2944pev/v04_50_dca_reference_calibration/synthetic_reference.calibration.json)
- [synthetic_reference.csv](../../simulation/hardware/awr2944pev/v04_50_dca_reference_calibration/synthetic_reference.csv)

